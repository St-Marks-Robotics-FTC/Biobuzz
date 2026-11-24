package org.firstinspires.ftc.teamcode.io;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.LockSupport;

/**
 * Runs every hub and sensor transaction on one background thread, so the OpMode loop never waits on hardware.
 *
 * <p>Each I/O cycle: clear the hubs' bulk caches, read every due device and publish the readings, wake the loop
 * thread ({@link #awaitNewCycle}), then write changed commands by score inside {@link #IO_WRITE_BUDGET_MS}.
 *
 * <p>One I/O thread, not one per hub: the SDK holds a single lock per USB/UART link for the whole request/response
 * of every Lynx command, and an RS485-chained Expansion Hub shares the Control Hub's link, so a second hardware
 * thread would only wait on that lock.
 *
 * <p>Lifecycle, loop thread: construct, build devices (they register themselves), {@link #start()}, call
 * {@link #awaitNewCycle()} at the top of every loop, {@link #stop()} from the OpMode's stop.
 */
@Configurable
public final class IoThread implements IoRegistry {
    /** configurables **/

    public static long IO_WRITE_BUDGET_MS = 10; // per I/O cycle: lower-score writes wait for the next cycle once this is spent
    public static long IO_STALL_TIMEOUT_MS = 500; // crash the OpMode if the I/O thread finishes no cycle for this long
    public static double STALENESS_REDUCTION = 1; // ms of waiting that doubles a pending write's score; higher values make waiting matter less
    public static boolean SOFT_SCHEDULING = false; // if true: write a device past its max staleness even when the write budget is spent

    /** end configurables **/

    private static final String TAG = "IoThread";
    private static final long MIN_CYCLE_NANOS = 1_000_000; // idle cycles (nothing to read or write) sleep the rest of 1 ms instead of spinning a core
    private static final long JOIN_TIMEOUT_MILLIS = 1000;
    private static final long STATUS_PERIOD_NANOS = 250_000_000; // rebuild the telemetry status at most 4x per second

    private enum Lifecycle { NEW, RUNNING, STOPPED }

    private final HardwareMap hw;
    private final List<LynxModule> hubs;
    private final List<IoDevice> devices = new ArrayList<>();
    private WriteScheduler scheduler; // created in start(), once the device list is final

    private Lifecycle lifecycle = Lifecycle.NEW; // guarded by this
    private boolean autoScheduling = false; // guarded by this; set before start, then only read by the I/O thread
    private Thread thread;
    private OpModeManagerImpl opModeManager;
    private volatile boolean haltRequested = false;
    private volatile Throwable failure = null;

    // cycle hand-off to the loop thread
    private final Object cycleLock = new Object();
    private long completedCycles = 0; // guarded by cycleLock
    private long lastSeenCycle = 0; // loop thread only

    // stats published by the I/O thread
    private volatile long lastCycleNanos = 0, lastReadNanos = 0, lastWriteNanos = 0, maxCycleNanos = 0;
    private volatile boolean statusRequested = false;
    private volatile String status = "I/O status: waiting for the first report";
    private long lastStatusNanos = 0; // I/O thread only

    /** stops the I/O thread if the OpMode ends without calling {@link #stop()} (crash, missing stop) */
    private final OpModeManagerNotifier.Notifications stopSafetyNet = new OpModeManagerNotifier.Notifications() {
        @Override public void onOpModePreInit(OpMode opMode) {}
        @Override public void onOpModePreStart(OpMode opMode) {}
        @Override public void onOpModePostStop(OpMode opMode) { halt(); }
    };

    /** switches every hub to manual bulk caching; the I/O thread clears the caches once per cycle */
    public IoThread(HardwareMap hw) {
        this.hw = hw;
        this.hubs = hw.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    /**
     * Forces soft scheduling on regardless of {@link #SOFT_SCHEDULING}, so overdue writes always go through
     * even when the write budget is spent. Autonomous OpModes call this for consistent timing.
     * @throws IllegalStateException if the I/O thread already started
     */
    public synchronized void setAutoScheduling() {
        if (lifecycle != Lifecycle.NEW) throw new IllegalStateException("cannot change scheduling: I/O thread is " + lifecycle + ", expected NEW");
        autoScheduling = true;
    }

    /** raw hardware map for device constructors; subsystems must go through devices */
    HardwareMap hw() { return hw; }

    @Override
    public synchronized void register(IoDevice device) {
        if (lifecycle != Lifecycle.NEW) throw new IllegalStateException("cannot add " + device.name + ": the I/O thread already started");
        for (IoDevice existing : devices) {
            if (existing.name.equals(device.name)) throw new IllegalStateException("two I/O devices are named " + device.name);
        }
        devices.add(device);
    }

    /**
     * Runs one full cycle on the calling thread (every due read, every pending write, no budget) so readings exist
     * and init positions reach the hardware, then starts the I/O thread.
     * @throws IllegalStateException if already started or stopped
     */
    public synchronized void start() {
        if (lifecycle != Lifecycle.NEW) throw new IllegalStateException("I/O thread is " + lifecycle + ", expected NEW");
        scheduler = new WriteScheduler(devices, System::nanoTime);
        readAll();
        synchronized (cycleLock) { completedCycles++; }
        scheduler.runAll();

        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (opModeManager == null) throw new IllegalStateException("no OpModeManager: the I/O thread can only run inside an OpMode");
        opModeManager.registerListener(stopSafetyNet);

        thread = new Thread(this::runLoop, TAG);
        lifecycle = Lifecycle.RUNNING;
        thread.start();
    }

    /**
     * Blocks until the I/O thread publishes readings newer than the ones this method last returned for, so every
     * loop iteration works on fresh data and the loop runs at the I/O cycle rate. Returns early if interrupted
     * (the OpMode is stopping) with the interrupt flag kept.
     * @throws IllegalStateException if the I/O thread is not running, crashed (cause attached), or finished no cycle
     *                               within {@link #IO_STALL_TIMEOUT_MS}
     */
    public void awaitNewCycle() {
        synchronized (this) {
            if (lifecycle != Lifecycle.RUNNING) throw new IllegalStateException("I/O thread is " + lifecycle + ", expected RUNNING");
        }
        long timeoutNanos = IO_STALL_TIMEOUT_MS * 1_000_000L;
        long deadline = System.nanoTime() + timeoutNanos;
        synchronized (cycleLock) {
            while (completedCycles == lastSeenCycle) {
                throwIfFailed();
                if (haltRequested) throw new IllegalStateException("I/O thread was stopped");
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0) throw new IllegalStateException("I/O thread finished no cycle in " + IO_STALL_TIMEOUT_MS + " ms");
                try {
                    cycleLock.wait(remaining / 1_000_000, (int) (remaining % 1_000_000));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            lastSeenCycle = completedCycles;
        }
        throwIfFailed();
    }

    /**
     * Stops the I/O thread, then writes every still-pending command from the calling thread so commands made in
     * the OpMode's stop() reach the hardware. Idempotent. Skips the flush if the I/O thread crashed.
     * @throws IllegalStateException if the I/O thread does not exit within one second
     */
    public synchronized void stop() {
        if (lifecycle == Lifecycle.NEW) {
            lifecycle = Lifecycle.STOPPED;
            return;
        }
        if (lifecycle == Lifecycle.STOPPED) return;
        halt();
        opModeManager.unregisterListener(stopSafetyNet);
        if (failure == null) scheduler.runAll();
    }

    /** stops the I/O thread without writing anything else; safe to call twice */
    private synchronized void halt() {
        if (lifecycle != Lifecycle.RUNNING) return;
        haltRequested = true;
        try {
            thread.join(JOIN_TIMEOUT_MILLIS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if (thread.isAlive()) throw new IllegalStateException("I/O thread did not stop within " + JOIN_TIMEOUT_MILLIS + " ms");
        lifecycle = Lifecycle.STOPPED;
    }

    /** duration of the newest complete I/O cycle in milliseconds */
    public double lastCycleMillis() { return lastCycleNanos / 1e6; }

    /**
     * Per-device scheduling report for telemetry, rebuilt by the I/O thread at most every 250 ms after the first
     * call. Costly to display; show it only while debugging.
     */
    public String status() {
        statusRequested = true;
        return status;
    }

    private void throwIfFailed() {
        Throwable cause = failure;
        if (cause != null) throw new IllegalStateException("I/O thread crashed: " + cause, cause);
    }

    private void runLoop() {
        try {
            while (!haltRequested) {
                long start = System.nanoTime();
                runCycle(start);
                long elapsed = System.nanoTime() - start;
                if (elapsed < MIN_CYCLE_NANOS) LockSupport.parkNanos(MIN_CYCLE_NANOS - elapsed);
            }
        } catch (Throwable t) {
            failure = t;
            RobotLog.ee(TAG, t, "I/O thread crashed");
        } finally {
            synchronized (cycleLock) { cycleLock.notifyAll(); } // wake a waiting loop thread so it sees the failure
        }
    }

    private void runCycle(long start) {
        readAll();
        long readEnd = System.nanoTime();
        synchronized (cycleLock) {
            completedCycles++;
            cycleLock.notifyAll();
        }

        long budgetNanos = IO_WRITE_BUDGET_MS * 1_000_000L;
        scheduler.runBudgeted(budgetNanos, STALENESS_REDUCTION, autoScheduling || SOFT_SCHEDULING);
        long end = System.nanoTime();

        lastReadNanos = readEnd - start;
        lastWriteNanos = end - readEnd;
        lastCycleNanos = end - start;
        if (end - start > maxCycleNanos) maxCycleNanos = end - start;

        if (statusRequested && end - lastStatusNanos >= STATUS_PERIOD_NANOS) {
            status = buildStatus(end);
            lastStatusNanos = end;
        }
    }

    private void readAll() {
        for (LynxModule hub : hubs) hub.clearBulkCache(); // next encoder read on each hub triggers one fresh bulk read
        for (IoDevice device : devices) {
            long start = System.nanoTime();
            device.readIfDue(start);
            device.totalReadNanos += System.nanoTime() - start;
        }
    }

    private String buildStatus(long nowNanos) {
        StringBuilder out = new StringBuilder();
        out.append("-----I/O thread-----\n");
        out.append("last cycle: ").append(lastCycleNanos / 1e6).append(" ms (read ").append(lastReadNanos / 1e6)
                .append(", write ").append(lastWriteNanos / 1e6).append(")\n");
        out.append("max cycle: ").append(maxCycleNanos / 1e6).append(" ms\n");
        for (IoDevice device : devices) {
            out.append("---").append(device.name).append("---\n");
            out.append("score: ").append(device.score).append(", waiting: ").append(device.waitingMillis(nowNanos)).append(" ms\n");
            out.append("writes: ").append(device.writeCount).append(", est write: ").append(device.estimatedWriteNanos / 1e6)
                    .append(" ms, total write: ").append(device.totalWriteNanos / 1e6).append(" ms\n");
            out.append("total read: ").append(device.totalReadNanos / 1e6).append(" ms\n");
        }
        return out.toString();
    }
}

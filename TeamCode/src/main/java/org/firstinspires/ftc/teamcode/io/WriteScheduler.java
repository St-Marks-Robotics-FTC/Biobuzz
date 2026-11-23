package org.firstinspires.ftc.teamcode.io;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Picks and performs one I/O cycle's device writes, highest score first, inside a time budget.
 * Single-threaded: only the I/O thread (or the loop thread while the I/O thread is not running) may call it.
 */
final class WriteScheduler {
    private static final Comparator<IoDevice> HIGHEST_SCORE_FIRST = (a, b) -> Double.compare(b.score, a.score);

    private final IoDevice[] order; // reused every cycle so sorting never allocates a new array
    private final LongSupplier clockNanos;

    /** @param clockNanos monotonic clock in nanoseconds ({@code System::nanoTime} on the robot) */
    WriteScheduler(List<IoDevice> devices, LongSupplier clockNanos) {
        this.order = devices.toArray(new IoDevice[0]);
        this.clockNanos = clockNanos;
    }

    /**
     * Writes pending devices by descending score until {@code budgetNanos} is spent. A device is skipped when its
     * estimated write no longer fits in what is left. With {@code softScheduling}, devices waiting at least their
     * {@link WritePolicy} max staleness are written first and ignore the budget, so nothing waits forever.
     * @return number of devices written
     * @throws IllegalStateException if a device misreports its pending change or stalenessReductionMillis is not positive
     */
    int runBudgeted(long budgetNanos, double stalenessReductionMillis, boolean softScheduling) {
        long start = clockNanos.getAsLong();
        for (IoDevice device : order) device.refreshScore(start, stalenessReductionMillis);
        Arrays.sort(order, HIGHEST_SCORE_FIRST);

        int writes = 0;
        if (softScheduling) {
            for (IoDevice device : order) {
                if (device.score > 0 && device.isOverdue(start)) writes += write(device);
            }
        }
        for (IoDevice device : order) {
            if (device.score == 0) break; // sorted: everything after this has nothing to write
            if (device.writtenThisCycle) continue;
            long remaining = budgetNanos - (clockNanos.getAsLong() - start);
            if (remaining <= 0) break;
            if (device.estimatedWriteNanos <= remaining) writes += write(device);
        }
        return writes;
    }

    /**
     * Writes every device with a pending change, ignoring the budget. Used for the first cycle (so init positions
     * reach the hardware) and the final flush when the OpMode stops.
     * @return number of devices written
     */
    int runAll() {
        long start = clockNanos.getAsLong();
        int writes = 0;
        for (IoDevice device : order) {
            device.refreshScore(start, 1);
            if (device.score > 0) writes += write(device);
        }
        return writes;
    }

    private int write(IoDevice device) {
        long start = clockNanos.getAsLong();
        device.write();
        device.recordWrite(clockNanos.getAsLong() - start);
        return 1;
    }
}

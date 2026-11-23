package org.firstinspires.ftc.teamcode.io;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class WriteSchedulerTest {
    private static final long MS = 1_000_000L;

    /** a fake clock that only moves when a fake device "talks to hardware" */
    private static final class FakeClock {
        long nanos = 0;
    }

    /** a device whose pending change is set by the test; each write takes writeNanos and clears the change */
    private static final class FakeDevice extends IoDevice {
        private final FakeClock clock;
        private final long writeNanos;
        private final List<String> writeLog;
        double change;

        FakeDevice(List<IoDevice> registry, FakeClock clock, List<String> writeLog, String name, WritePolicy policy, long writeNanos, double change) {
            super(registry::add, name, policy, writeNanos / 1e6);
            this.clock = clock;
            this.writeNanos = writeNanos;
            this.writeLog = writeLog;
            this.change = change;
        }

        @Override double pendingChange() { return change; }

        @Override void write() {
            clock.nanos += writeNanos;
            writeLog.add(name);
            change = 0;
        }
    }

    private final FakeClock clock = new FakeClock();
    private final List<IoDevice> devices = new ArrayList<>();
    private final List<String> writes = new ArrayList<>();

    private FakeDevice device(String name, WritePolicy policy, long writeNanos, double change) {
        return new FakeDevice(devices, clock, writes, name, policy, writeNanos, change);
    }

    private WriteScheduler scheduler() {
        return new WriteScheduler(devices, () -> clock.nanos);
    }

    // If this fails: writes are no longer ordered by score, so small changes can delay big ones
    @Test
    public void writesLargestChangeFirst() {
        device("small", WritePolicy.DEFAULT, 2 * MS, 0.1);
        device("large", WritePolicy.DEFAULT, 2 * MS, 0.9);
        scheduler().runBudgeted(10 * MS, 1, false);
        assertEquals(Arrays.asList("large", "small"), writes);
    }

    // If this fails: the write budget is ignored and the I/O cycle grows without bound
    @Test
    public void stopsWritingOnceBudgetIsSpent() {
        device("a", WritePolicy.DEFAULT, 3 * MS, 0.9);
        device("b", WritePolicy.DEFAULT, 3 * MS, 0.5);
        device("c", WritePolicy.DEFAULT, 3 * MS, 0.1);
        int count = scheduler().runBudgeted(6 * MS, 1, false);
        assertEquals(2, count);
        assertEquals(Arrays.asList("a", "b"), writes);
    }

    // If this fails: one slow device that doesn't fit blocks cheaper devices that still fit
    @Test
    public void skipsWriteThatDoesNotFitButKeepsGoing() {
        device("slow", WritePolicy.DEFAULT, 8 * MS, 0.9);
        device("fast", WritePolicy.DEFAULT, 1 * MS, 0.1);
        scheduler().runBudgeted(5 * MS, 1, false);
        assertEquals(Collections.singletonList("fast"), writes);
    }

    // If this fails: IMMEDIATE devices (drivetrain, Pinpoint pose) can be starved by the budget
    @Test
    public void softSchedulingWritesOverdueDeviceEvenWithNoBudget() {
        device("drive", WritePolicy.IMMEDIATE, 8 * MS, 0.01);
        scheduler().runBudgeted(0, 1, true);
        assertEquals(Collections.singletonList("drive"), writes);
    }

    // If this fails: turning soft scheduling off no longer makes the budget a hard limit
    @Test
    public void withoutSoftSchedulingOverdueDeviceRespectsBudget() {
        device("drive", WritePolicy.IMMEDIATE, 8 * MS, 0.01);
        scheduler().runBudgeted(0, 1, false);
        assertTrue(writes.isEmpty());
    }

    // If this fails: a device with nothing to write is written anyway, wasting a Lynx command per cycle
    @Test
    public void idleDeviceIsNeverWritten() {
        device("idle", WritePolicy.IMMEDIATE, 1 * MS, 0);
        scheduler().runBudgeted(100 * MS, 1, true);
        assertTrue(writes.isEmpty());
    }

    // If this fails: a small change that keeps losing to big ones waits forever instead of gaining score
    @Test
    public void waitingRaisesScoreUntilOldSmallChangeBeatsFreshLargeOne() {
        FakeDevice old = device("old", new WritePolicy(1, 10_000), 2 * MS, 0.2);
        WriteScheduler scheduler = scheduler();
        scheduler.runBudgeted(0, 1, false); // nothing fits; "old" starts waiting at t=0
        clock.nanos += 10 * MS; // waited 10 ms: score 0.2 * (1 + 10) = 2.2
        device("fresh", new WritePolicy(1, 10_000), 2 * MS, 1.0); // score 1.0 * (1 + 0) = 1.0
        WriteScheduler rebuilt = new WriteScheduler(devices, () -> clock.nanos);
        rebuilt.runBudgeted(2 * MS, 1, false);
        assertEquals(Collections.singletonList("old"), writes);
        assertEquals(0, old.change, 0);
    }

    // If this fails: init positions or the final stop() commands may never reach the hardware
    @Test
    public void runAllIgnoresBudgetAndWritesEveryPendingDevice() {
        device("a", WritePolicy.DEFAULT, 50 * MS, 0.1);
        device("b", WritePolicy.DEFAULT, 50 * MS, 1.0);
        device("idle", WritePolicy.DEFAULT, 50 * MS, 0);
        int count = scheduler().runAll();
        assertEquals(2, count);
        assertTrue(writes.containsAll(Arrays.asList("a", "b")));
    }

    // If this fails: a device's drift is no longer validated and NaN silently ranks it last forever
    @Test(expected = IllegalStateException.class)
    public void rejectsNaNPendingChange() {
        device("broken", WritePolicy.DEFAULT, MS, Double.NaN);
        scheduler().runBudgeted(10 * MS, 1, false);
    }

    // If this fails: a pending change above 1 is accepted and distorts every score comparison
    @Test(expected = IllegalStateException.class)
    public void rejectsPendingChangeAboveOne() {
        device("broken", WritePolicy.DEFAULT, MS, 1.5);
        scheduler().runBudgeted(10 * MS, 1, false);
    }

    // If this fails: stalenessReduction = 0 divides by zero and every score becomes NaN or infinite
    @Test(expected = IllegalStateException.class)
    public void rejectsZeroStalenessReduction() {
        device("a", WritePolicy.DEFAULT, MS, 0.5);
        scheduler().runBudgeted(10 * MS, 0, false);
    }
}

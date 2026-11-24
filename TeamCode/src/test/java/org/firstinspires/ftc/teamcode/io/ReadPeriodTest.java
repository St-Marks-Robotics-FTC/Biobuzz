package org.firstinspires.ftc.teamcode.io;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ReadPeriodTest {
    private static final long MS = 1_000_000L;
    private static final long LAST = 1_000 * MS; // arbitrary nanoTime of the previous read

    // If this fails: a device configured to never read (e.g. drive motor current) costs a Lynx command anyway
    @Test
    public void neverIsNeverDue() {
        assertFalse(ReadPeriod.never().isDue(false, 0, LAST));
        assertFalse(ReadPeriod.never().isEnabled());
    }

    // If this fails: per-cycle reads (encoders, Pinpoint) skip cycles and the loop sees stale data
    @Test
    public void everyCycleIsAlwaysDue() {
        assertTrue(ReadPeriod.everyCycle().isDue(true, LAST, LAST));
    }

    // If this fails: interval reads never take their first reading and the getter throws forever
    @Test
    public void intervalIsDueBeforeFirstRead() {
        assertTrue(ReadPeriod.millis(100).isDue(false, 0, 0));
    }

    // If this fails: interval reads happen early (wasted Lynx commands)
    @Test
    public void intervalIsNotDueOneNanoBeforeInterval() {
        assertFalse(ReadPeriod.millis(100).isDue(true, LAST, LAST + 100 * MS - 1));
    }

    // If this fails: interval reads happen late or never
    @Test
    public void intervalIsDueExactlyAtInterval() {
        assertTrue(ReadPeriod.millis(100).isDue(true, LAST, LAST + 100 * MS));
    }

    // If this fails: a live configurable interval is read once at construction instead of every cycle
    @Test
    public void liveIntervalFollowsSupplier() {
        long[] interval = {100};
        ReadPeriod period = ReadPeriod.millis(() -> interval[0]);
        assertFalse(period.isDue(true, LAST, LAST + 50 * MS));
        interval[0] = 40;
        assertTrue(period.isDue(true, LAST, LAST + 50 * MS));
    }

    // If this fails: a live interval tuned to 0 stops reading instead of reading every cycle
    @Test
    public void liveIntervalAtZeroReadsEveryCycle() {
        assertTrue(ReadPeriod.millis(() -> 0).isDue(true, LAST, LAST));
    }

    // If this fails: a fixed zero/negative interval is silently accepted instead of pointing at everyCycle()
    @Test(expected = IllegalArgumentException.class)
    public void fixedZeroIntervalIsRejected() {
        ReadPeriod.millis(0);
    }

    // If this fails: a write policy with no priority is accepted and its device can never win a write slot
    @Test(expected = IllegalArgumentException.class)
    public void writePolicyRejectsZeroPriority() {
        new WritePolicy(0, 100);
    }

    // If this fails: NaN priority is accepted and poisons the score sort
    @Test(expected = IllegalArgumentException.class)
    public void writePolicyRejectsNaNPriority() {
        new WritePolicy(Double.NaN, 100);
    }

    // If this fails: a negative max staleness is accepted and makes the device permanently "overdue"
    @Test(expected = IllegalArgumentException.class)
    public void writePolicyRejectsNegativeStaleness() {
        new WritePolicy(1, -1);
    }
}

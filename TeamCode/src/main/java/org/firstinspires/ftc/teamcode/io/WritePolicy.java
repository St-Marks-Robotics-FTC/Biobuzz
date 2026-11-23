package org.firstinspires.ftc.teamcode.io;

/** How the I/O thread ranks one device's writes against the others. Immutable. */
public final class WritePolicy {
    /** written in the cycle its command changes, even past the write budget (while soft scheduling is on) */
    public static final WritePolicy IMMEDIATE = new WritePolicy(1, 0);
    /** competes for the write budget; forced through after 100 ms of waiting */
    public static final WritePolicy DEFAULT = new WritePolicy(1, 100);

    final double priority;
    final long maxStalenessNanos;

    /**
     * @param priority           multiplies the write score; higher values are written first
     * @param maxStalenessMillis how long a changed command may wait before it is written regardless of the budget
     * @throws IllegalArgumentException if priority is not positive or maxStalenessMillis is negative
     */
    public WritePolicy(double priority, long maxStalenessMillis) {
        if (!(priority > 0)) throw new IllegalArgumentException("priority must be positive, got " + priority);
        if (maxStalenessMillis < 0) throw new IllegalArgumentException("max staleness must be >= 0, got " + maxStalenessMillis + " ms");
        this.priority = priority;
        this.maxStalenessNanos = maxStalenessMillis * 1_000_000L;
    }
}

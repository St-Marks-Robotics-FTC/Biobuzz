package org.firstinspires.ftc.teamcode.io;

/** One number read from hardware, stamped with {@link System#nanoTime()} right after the read. Immutable. */
public final class TimedValue {
    private final double value;
    private final long nanoTime;

    TimedValue(double value, long nanoTime) {
        this.value = value;
        this.nanoTime = nanoTime;
    }

    public double value() { return value; }

    /** {@link System#nanoTime()} right after the read */
    public long nanoTime() { return nanoTime; }

    /** milliseconds since the read */
    public double ageMillis() { return (System.nanoTime() - nanoTime) / 1e6; }
}

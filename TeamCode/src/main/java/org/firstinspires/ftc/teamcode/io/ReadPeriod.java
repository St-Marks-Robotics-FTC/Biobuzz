package org.firstinspires.ftc.teamcode.io;

import java.util.function.LongSupplier;

/**
 * How often the I/O thread reads one value from hardware. Immutable.
 *
 * <p>Pick the slowest period that still works: every read the hub can't answer from its bulk cache
 * (motor current, hub voltage, I2C sensors) costs at least one Lynx command (~2 ms), which delays
 * every other read and write in the same I/O cycle.
 */
public final class ReadPeriod {
    private enum Kind { NEVER, EVERY_CYCLE, INTERVAL }

    private static final ReadPeriod NEVER = new ReadPeriod(Kind.NEVER, null);
    private static final ReadPeriod EVERY_CYCLE = new ReadPeriod(Kind.EVERY_CYCLE, null);

    private final Kind kind;
    private final LongSupplier intervalMillis; // only set for INTERVAL

    private ReadPeriod(Kind kind, LongSupplier intervalMillis) {
        this.kind = kind;
        this.intervalMillis = intervalMillis;
    }

    /** never read */
    public static ReadPeriod never() { return NEVER; }

    /** read once per I/O cycle */
    public static ReadPeriod everyCycle() { return EVERY_CYCLE; }

    /**
     * read when at least {@code millis} have passed since the previous read
     * @throws IllegalArgumentException if millis is not positive (use {@link #everyCycle()} instead)
     */
    public static ReadPeriod millis(long millis) {
        if (millis <= 0) throw new IllegalArgumentException("read interval must be positive, got " + millis + " ms");
        return new ReadPeriod(Kind.INTERVAL, () -> millis);
    }

    /**
     * read when at least {@code millis.getAsLong()} have passed since the previous read; the supplier is
     * re-read every cycle so it can point at a live {@code Tunables} field. A value &lt;= 0 reads every cycle.
     */
    public static ReadPeriod millis(LongSupplier millis) {
        if (millis == null) throw new IllegalArgumentException("interval supplier is null");
        return new ReadPeriod(Kind.INTERVAL, millis);
    }

    /** false only for {@link #never()} */
    public boolean isEnabled() { return kind != Kind.NEVER; }

    /**
     * @param hasRead       whether this value has been read before
     * @param lastReadNanos {@link System#nanoTime()} of the previous read; ignored when {@code hasRead} is false
     * @param nowNanos      {@link System#nanoTime()} now
     * @return true if the value should be read this cycle
     */
    boolean isDue(boolean hasRead, long lastReadNanos, long nowNanos) {
        switch (kind) {
            case NEVER: return false;
            case EVERY_CYCLE: return true;
            case INTERVAL:
                if (!hasRead) return true;
                long millis = intervalMillis.getAsLong();
                return millis <= 0 || nowNanos - lastReadNanos >= millis * 1_000_000L;
            default: throw new IllegalStateException("unhandled read period kind " + kind);
        }
    }
}

package org.firstinspires.ftc.teamcode.io;

/**
 * One piece of hardware owned by the I/O thread.
 *
 * <p>Thread contract: the constructor runs on the loop thread before {@link IoThread#start()} and may configure
 * hardware directly. After start, only the I/O thread calls the package-private hooks ({@link #readIfDue},
 * {@link #pendingChange}, {@link #write}) and touches hardware. Public methods (the loop thread's API) may only
 * write volatile command fields and read volatile, immutable published readings.
 *
 * <p>To add a new kind of hardware, subclass this in the {@code io} package: register in the constructor, publish
 * readings from {@link #readIfDue}, report command drift from {@link #pendingChange}, and send it in {@link #write}.
 */
public abstract class IoDevice {
    private static final double WRITE_ESTIMATE_SMOOTHING = 0.2; // how far each measured write moves the estimate (0-1)

    final String name;
    final WritePolicy writePolicy;

    // scheduling bookkeeping, I/O thread only (WriteScheduler)
    private boolean pending = false; // a changed command is waiting to be written
    private long pendingSinceNanos = 0; // when the current wait started
    double score = 0; // write ranking for this cycle; 0 when nothing is pending
    boolean writtenThisCycle = false;
    double estimatedWriteNanos;
    long totalWriteNanos = 0;
    long writeCount = 0;
    long totalReadNanos = 0; // maintained by IoThread

    /** read-only device: never writes */
    IoDevice(IoRegistry registry, String name) {
        this(registry, name, WritePolicy.DEFAULT, 0);
    }

    /**
     * @param initialWriteEstimateMillis expected duration of one {@link #write}, used until real writes are measured
     * @throws IllegalArgumentException if name is empty or the estimate is negative
     */
    IoDevice(IoRegistry registry, String name, WritePolicy writePolicy, double initialWriteEstimateMillis) {
        if (name == null || name.isEmpty()) throw new IllegalArgumentException("device name is empty");
        if (writePolicy == null) throw new IllegalArgumentException(name + ": write policy is null");
        if (!(initialWriteEstimateMillis >= 0)) throw new IllegalArgumentException(name + ": write estimate must be >= 0, got " + initialWriteEstimateMillis);
        this.name = name;
        this.writePolicy = writePolicy;
        this.estimatedWriteNanos = initialWriteEstimateMillis * 1e6;
        registry.register(this);
    }

    public final String name() { return name; }

    /**
     * Reads every value whose {@link ReadPeriod} is due and publishes the results. I/O thread only.
     * @param nowNanos {@link System#nanoTime()} at the start of this cycle's reads
     */
    void readIfDue(long nowNanos) {}

    /**
     * How far the hardware is from the newest command: 0 means nothing to write, 1 is the largest possible change.
     * Must report 1 until the first write so every device writes at least once. I/O thread only.
     */
    double pendingChange() { return 0; }

    /** Sends the newest command to hardware. Only called after {@link #pendingChange} returned &gt; 0. I/O thread only. */
    void write() {
        throw new IllegalStateException(name + " reported a pending change but has no writes");
    }

    /**
     * Snapshots this cycle's write score: priority * change * (1 + waitingMillis / stalenessReductionMillis).
     * Waiting time only grows while a change is pending, so an idle device never builds up score.
     * @throws IllegalStateException if {@link #pendingChange} left [0, 1] or stalenessReductionMillis is not positive
     */
    final void refreshScore(long nowNanos, double stalenessReductionMillis) {
        if (!(stalenessReductionMillis > 0)) throw new IllegalStateException("stalenessReduction must be positive, got " + stalenessReductionMillis);
        writtenThisCycle = false;
        double change = pendingChange();
        if (!(change >= 0 && change <= 1)) throw new IllegalStateException(name + " reported pending change " + change + " outside [0, 1]");
        if (change == 0) {
            pending = false;
            score = 0;
            return;
        }
        if (!pending) {
            pending = true;
            pendingSinceNanos = nowNanos;
        }
        score = writePolicy.priority * change * (1 + waitingMillis(nowNanos) / stalenessReductionMillis);
    }

    /** true if a change has waited at least {@link WritePolicy#maxStalenessNanos} (valid after {@link #refreshScore}) */
    final boolean isOverdue(long nowNanos) {
        return pending && nowNanos - pendingSinceNanos >= writePolicy.maxStalenessNanos;
    }

    /** how long the current change has waited, 0 if nothing is pending */
    final double waitingMillis(long nowNanos) {
        return pending ? (nowNanos - pendingSinceNanos) / 1e6 : 0;
    }

    /** books one finished {@link #write} that took durationNanos */
    final void recordWrite(long durationNanos) {
        estimatedWriteNanos += (durationNanos - estimatedWriteNanos) * WRITE_ESTIMATE_SMOOTHING;
        totalWriteNanos += durationNanos;
        writeCount++;
        writtenThisCycle = true;
        pending = false;
    }
}

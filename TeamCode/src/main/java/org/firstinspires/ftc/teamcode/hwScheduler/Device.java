/** device wrapper used by HWScheduler **/
/** all time values are in nanoseconds **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.pedropathing.utils.Timer;

public abstract class Device {
    private static final double UPDATE_LENGTH_SMOOTHING = 0.2;

    protected final double priorityMultiplier;
    protected final String name;
    protected long estimatedUpdateLength;
    protected long maxStaleness = 100 * 1000000;
    protected long totalUpdateTime = 0;
    boolean isUpdated = false;
    private boolean hasWritten = false;
    private Timer stalenessTimer;
    private Timer writeTimer; // measures how long a write takes

    private double priority = 0;
    private long staleness = 0;
    double score = 0;

    public Device(String name, double priorityMultiplier) {
        this.name = name;
        this.priorityMultiplier = priorityMultiplier;

        stalenessTimer = new Timer();
        writeTimer = new Timer();
    }

    /** called by HWScheduler once per loop before sorting **/
    void refresh(double stalenessReduction) {
        isUpdated = false;

        priority = hasWritten ? getChange() * priorityMultiplier : priorityMultiplier;
        if (priority == 0) { stalenessTimer.reset(); } // prevent staleness increasing while there is nothing to write
        staleness = stalenessTimer.nanoseconds();

        score = priority * (1 + staleness / stalenessReduction);
    }

    public double getPriority() {
        return priority;
    }

    public long getStaleness() {
        return staleness;
    }

    public long getEstimatedUpdateLength() {
        return estimatedUpdateLength;
    }

    public long getMaxStaleness() {
        return maxStaleness;
    }

    public long getTotalUpdateTime() {
        return totalUpdateTime;
    }

    public void forceUpdate() {
        update();
    }

    protected void update() {
        writeTimer.reset();

        write();

        long updateLength = writeTimer.nanoseconds();

        if (updateLength > 2 * 1000000) { // reject impossible update times
            estimatedUpdateLength = (long) (estimatedUpdateLength + (updateLength - estimatedUpdateLength) * UPDATE_LENGTH_SMOOTHING);
        }
        totalUpdateTime += updateLength;
        isUpdated = true;
        hasWritten = true;
        stalenessTimer.reset(); // so we know how long since our last update
    }

    protected abstract double getChange(); // this must be normalized to 0-1

    protected abstract void write();
}

/** device wrapper used by HWScheduler **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.pedropathing.utils.Timer;

public abstract class Device {
    protected final double priorityMultiplier;
    protected final String name;
    protected double lastUpdateLength; // both in millis
    protected double maxStaleness = 100;
    protected double totalUpdateTime = 0; // total amount of time in ms that we have spent updating this device
    boolean isUpdated = false;
    private Timer updateTimer; // measures time to update and time since last update (staleness)

    public Device(String name, double priorityMultiplier) {
        this.name = name;
        this.priorityMultiplier = priorityMultiplier;

        updateTimer = new Timer();
    }

    public void resetUpdateBoolean() {
        isUpdated = false;
    }

    public double getStaleness() {
        if (getPriority() == 0) { updateTimer.reset(); } // prevent staleness increasing while priority remains at zero
        return updateTimer.milliseconds();
    }

    public double getLastUpdateLength() {
        return lastUpdateLength;
    }

    public double getMaxStaleness() {
        return maxStaleness;
    }

    public double getTotalUpdateTime() {
        return totalUpdateTime;
    }

    protected void update() {
        updateTimer.reset();

        write();

        lastUpdateLength = updateTimer.milliseconds();
        totalUpdateTime += lastUpdateLength;
        isUpdated = true;
        updateTimer.reset(); // so we know how long since our last update
    }

    protected abstract double getPriority();

    protected abstract void write();
}

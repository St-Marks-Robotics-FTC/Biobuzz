/** soft real-time scheduler for hardware writes **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tunables;

import java.util.ArrayList;

public class HWScheduler {
    private HardwareMap hardwareMap;
    private ArrayList<Device> deviceList = new ArrayList<>();
    private Timer updateTimer;


    public HWScheduler(HardwareMap hw) {
        hardwareMap = hw;
        updateTimer = new Timer();
    }

    public HardwareMap hw() {
        return hardwareMap;
    }

    public Device getDevice(String name) {
        for (Device device : deviceList) {
            if (device.name.equals(name)){
                return device;
            }
        }
        return null;
    }

    public void addDevice(Device device) {
        deviceList.add(device);
    }

    public double update(long elapsedTime) { // take in time we have already spent and return time our scheduler took
        // elapsedTime is in nanoseconds
        updateTimer.reset();

        // snapshot priority, staleness and score for every device (also resets isUpdated)
        for (Device device : deviceList) {
            device.refresh(Tunables.stalenessReduction);
        }

        // sort device list by score, highest first
        deviceList.sort((d1, d2) -> Double.compare(d2.score, d1.score));

        if (Tunables.softScheduling) {
            // first update any devices that are past their max stale time
            // this ignores our time budget - otherwise a slow loop would let devices go stale forever
            for (Device device : deviceList) {
                if (device.getPriority() > 0 && device.getStaleness() >= device.getMaxStaleness()) {
                    device.update();
                }
            }
        }

        // this for loop will go from highest to least priority
        for (Device device : deviceList) {
            double remainingLoopTime = (Tunables.targetLoopLength * 1000) - elapsedTime - updateTimer.nanoseconds();

            if (remainingLoopTime <= 0) break; // out of time - everything else waits for next loop

            if (device.isUpdated || device.getPriority() == 0) {
                continue;
            } // already written or nothing to write

            // check if we have enough time to update this device
            if (device.getEstimatedUpdateLength() <= remainingLoopTime) {
                device.update();
            }
        }

        return updateTimer.milliseconds(); // return time we took updating in milliseconds
    }

    public void forceUpdate() { // force update all devices
        for (Device device : deviceList) {
            device.forceUpdate();
        }
    }

    public String getStatus() {
        StringBuilder deviceStats = new StringBuilder();
        StringBuilder overallStats = new StringBuilder();

        double sumUpdateTime = 0;
        double highestStaleness = 0;

        // calc sum update time
        for (Device device : deviceList) {
            sumUpdateTime += device.getTotalUpdateTime();
        }

        for (Device device : deviceList) {
            double updatePercent = sumUpdateTime > 0 ? device.getTotalUpdateTime() / sumUpdateTime * 100 : 0;

            if (device.getStaleness() >= highestStaleness) highestStaleness = device.getStaleness();

            deviceStats.append("---" + device.name + "---\n");
            deviceStats.append("priority: " + device.getPriority() + "\n");
            deviceStats.append("estimated update length: " + device.getEstimatedUpdateLength() + "\n");
            deviceStats.append("staleness: " + device.getStaleness() + "\n");
            deviceStats.append("total update time: " + device.getTotalUpdateTime() + "\n");
            deviceStats.append("total update time percent: " + updatePercent + "%\n");
        }

        overallStats.append("-----overall metrics-----\n");
        overallStats.append("sum update time: " + sumUpdateTime + "\n");
        overallStats.append("highest staleness: " + highestStaleness + "\n");

        overallStats.append(deviceStats.toString());

        return overallStats.toString();
    }
}

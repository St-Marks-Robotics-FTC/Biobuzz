/** soft real-time scheduler for hardware writes **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.pedropathing.utils.Timer;

import java.util.ArrayList;

public class HWScheduler {
    private ArrayList<Device> deviceList = new ArrayList();

    private double targetLoopLength = 20; // target loop length in ms

    private double stalenessReduction = 2; // how important staleness is. higher values are less important

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

    public double update(double elapsedTime) { // take in time we have already spent and return time our scheduler took
        Timer updateTimer = new Timer();

        // reset all isUpdated
        for (Device device : deviceList) {
            device.resetUpdateBoolean();
        }

        // sort device list by priority
        deviceList.sort((d1, d2) ->
                Double.compare(
                        d2.getPriority() * (d2.getStaleness() / stalenessReduction) * d2.priorityMultiplier,
                        d1.getPriority() * (d2.getStaleness() / stalenessReduction) * d1.priorityMultiplier
                ));

        // first check if any devices are above their max stale time
        for (Device device : deviceList) {
            double remainingLoopTime = targetLoopLength - elapsedTime - updateTimer.milliseconds();

            if (device.getStaleness() >= device.getMaxStaleness() && device.getLastUpdateLength() <= remainingLoopTime) {
                device.update();
            }
        }

        // this for loop will go from highest to least priority
        for (Device device : deviceList) {
            double remainingLoopTime = targetLoopLength - elapsedTime - updateTimer.milliseconds();

            if (!device.isUpdated && remainingLoopTime > 0) {
                // check if we have enough time to update this device
                if (device.getLastUpdateLength() <= remainingLoopTime) {
                    device.update();
                }
            } else {
                break;
            }
        }

        return updateTimer.milliseconds(); // return time we took updating
    }

    public String getStatus() {
        StringBuilder returnString = new StringBuilder();
        double sumUpdateTime = 0;

        // calc sum update time
        for (Device device : deviceList) {
            sumUpdateTime += device.getTotalUpdateTime();
        }

        for (Device device : deviceList) {
            returnString.append("---" + device.name + "---");
            returnString.append("priority: " + device.getPriority() + "\n");
            returnString.append("last update length: " + device.getLastUpdateLength() + "\n");
            returnString.append("staleness: " + device.getStaleness() + "\n");
            returnString.append("total update time: " + device.getTotalUpdateTime() + "\n");
            returnString.append("total update time percent: " + device.getTotalUpdateTime() / sumUpdateTime + "%\n");
        }

        returnString.append("-----overall metrics-----\n");
        returnString.append("sum update time: " + sumUpdateTime);

        return returnString.toString();
    }
}

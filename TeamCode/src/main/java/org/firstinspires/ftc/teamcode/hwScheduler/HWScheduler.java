/** soft real-time scheduler for hardware writes **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import android.bluetooth.BluetoothClass;

import com.pedropathing.utils.Timer;

import java.util.ArrayList;

// TODO: add ability to get hardwareMap from scheduler when adding devices
public class HWScheduler {
    private ArrayList<Device> deviceList = new ArrayList<>();

    private double targetLoopLength = 15; // target loop length in ms

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

        // snapshot priority, staleness and score for every device (also resets isUpdated)
        for (Device device : deviceList) {
            device.refresh(stalenessReduction);
        }

        // sort device list by score, highest first
        deviceList.sort((d1, d2) -> Double.compare(d2.score, d1.score));

        // first update any devices that are past their max stale time
        // this ignores our time budget - otherwise a slow loop would let devices go stale forever
        for (Device device : deviceList) {
            if (device.getPriority() > 0 && device.getStaleness() >= device.getMaxStaleness()) {
                device.update();
            }
        }

        // this for loop will go from highest to least priority
        for (Device device : deviceList) {
            double remainingLoopTime = targetLoopLength - elapsedTime - updateTimer.milliseconds();

            if (remainingLoopTime <= 0) break; // out of time - everything else waits for next loop

            if (device.isUpdated || device.getPriority() == 0) {
                continue;
            } // already written or nothing to write

            // check if we have enough time to update this device
            if (device.getEstimatedUpdateLength() <= remainingLoopTime) {
                device.update();
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
            double updatePercent = sumUpdateTime > 0 ? device.getTotalUpdateTime() / sumUpdateTime * 100 : 0;

            returnString.append("---" + device.name + "---\n");
            returnString.append("priority: " + device.getPriority() + "\n");
            returnString.append("estimated update length: " + device.getEstimatedUpdateLength() + "\n");
            returnString.append("staleness: " + device.getStaleness() + "\n");
            returnString.append("total update time: " + device.getTotalUpdateTime() + "\n");
            returnString.append("total update time percent: " + updatePercent + "%\n");
        }

        returnString.append("-----overall metrics-----\n");
        returnString.append("sum update time: " + sumUpdateTime);

        return returnString.toString();
    }
}

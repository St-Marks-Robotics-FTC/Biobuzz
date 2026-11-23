/** soft real-time scheduler for hardware writes **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

@Configurable
public class HWScheduler {
    /** configurables **/

    public static long TELE_TARGET_LOOP_LENGTH = 20; // target loop length in ms
    public static long AUTO_TARGET_LOOP_LENGTH = 25; // autos generally have longer loops

    public static double STALENESS_REDUCTION = 1; // how important staleness is. higher values are less important
    public static boolean SOFT_SCHEDULING = true; // if true (or isDebugging is true): update device once max staleness has been exceeded

    /** end configurables **/
    private HardwareMap hardwareMap;
    private ArrayList<Device> deviceList = new ArrayList<>();
    private Timer updateTimer;
    private boolean isAuto = false; // use auto loop length instead


    public HWScheduler(HardwareMap hw) {
        hardwareMap = hw;
        updateTimer = new Timer();
    }

    public HardwareMap hw() {
        return hardwareMap;
    }

    public void setAutoLoopLength() {
        isAuto = true;
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
            device.refresh(STALENESS_REDUCTION);
        }

        // sort device list by score, highest first
        deviceList.sort((d1, d2) -> Double.compare(d2.score, d1.score));

        if (SOFT_SCHEDULING || isAuto) { // force soft scheduling in auto to increase consistency
            // first update any devices that are past their max stale time
            // this ignores our time budget - otherwise a slow loop would let devices go stale forever
            for (Device device : deviceList) {
                if (device.getChange() != 0 && device.getStaleness() >= device.getMaxStaleness()) {
                    device.write();
                }
            }
        }

        // this for loop will go from highest to least priority
        for (Device device : deviceList) {
            long target; // target loop length
            if (isAuto) {
                target = AUTO_TARGET_LOOP_LENGTH * 1000000; // convert millis to nanoseconds
            } else {
                target = TELE_TARGET_LOOP_LENGTH * 1000000; // convert millis to nanoseconds
            }

            long remainingLoopTime = target - elapsedTime - updateTimer.nanoseconds();

            if (remainingLoopTime <= 0) break; // out of time - everything else waits for next loop

            if (device.isUpdated || device.getChange() == 0) {
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
            deviceStats.append("estimated update length: " + device.getEstimatedUpdateLength() / 1000000 + "\n");
            deviceStats.append("staleness: " + device.getStaleness() / 1000000 + "\n");
            deviceStats.append("total update time: " + device.getTotalUpdateTime() / 1000000 + "\n");
            deviceStats.append("total update time percent: " + updatePercent + "%\n");
        }

        overallStats.append("-----overall metrics-----\n");
        overallStats.append("sum update time: " + sumUpdateTime / 1000000 + "\n");
        overallStats.append("highest staleness: " + highestStaleness / 1000000 + "\n");

        overallStats.append(deviceStats.toString());

        return overallStats.toString();
    }
}

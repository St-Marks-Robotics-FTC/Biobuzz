/** hardware-scheduled motor wrapper **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SDualMotor extends Device {
    public DcMotorEx m1, m2;
    private double lastPower = 0; // last power we wrote to the motor
    private double setPower = 0; // power we want to write to the motor

    public SDualMotor(String name1, String name2, double priorityMultiplier, HardwareMap hw) {
        super(name1 + "-" + name2, priorityMultiplier); // create Device

        estimatedUpdateLength = 6 * 1000000; // estimated time

        m1 = hw.get(DcMotorEx.class, name1);
        m2 = hw.get(DcMotorEx.class, name2);
    }

    public void setPower(double newPower) {
        setPower = newPower;
    }

    public double getPower() {
        return lastPower;
    }

    protected double getChange() {
        // motors map from -1<->1 so we halve them to align with servos 0-1
        return Math.abs(lastPower - setPower) / 2;
    }

    protected void write() {
        m1.setPower(setPower);
        m2.setPower(setPower);
        lastPower = setPower;
    }
}

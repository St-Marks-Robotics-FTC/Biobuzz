/** hardware-scheduled motor wrapper **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SMotor extends Device {
    public DcMotorEx m;
    private double lastPower = 0; // last power we wrote to the motor
    private double setPower = 0; // power we want to write to the motor

    public SMotor(String name, double priorityMultiplier, HardwareMap hw) {
        super(name, priorityMultiplier); // create Device

        estimatedUpdateLength = 3; // estimated time

        m = hw.get(DcMotorEx.class, name);
    }

    public void setPower(double newPower) {
        setPower = newPower;
    }

    public double getPower() {
        return lastPower;
    }

    protected double getChange() {
        return Math.abs(lastPower - setPower);
    }

    protected void write() {
        m.setPower(setPower);
        lastPower = setPower;
    }
}

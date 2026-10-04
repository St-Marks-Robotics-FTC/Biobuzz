package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Tunables.*;

public class Flywheel {
    private static final double TICKS_PER_REV = 28; // bare motor encoder resolution, adjust for actual flywheel motor

    private final DcMotorEx motor;
    private double targetRPM = 0;
    public boolean isRunning = false;
    private final PIDF pidf;
    public Flywheel(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "launchMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidf = new PIDF(flywheelP, flywheelI, flywheelD, flywheelF); // tune once flywheel is mounted
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return motor.getPower();
    }

    /** true once our current RPM is within {@link org.firstinspires.ftc.teamcode.Tunables#flywheelRPMMargin} of the target RPM.
     * always false while the target RPM is 0 (flywheel not spun up / commanded to shoot). **/
    public boolean isWithinMargin() {
        return targetRPM > 0 && Math.abs(getRPM() - targetRPM) <= flywheelRPMMargin;
    }

    public void update() {
        pidf.updateTerms(flywheelP, flywheelI, flywheelD, flywheelF);
        if (targetRPM == 0) {
            motor.setPower(targetRPM);
        } else {
            motor.setPower(pidf.calc(targetRPM, getRPM()));
        }
    }
}

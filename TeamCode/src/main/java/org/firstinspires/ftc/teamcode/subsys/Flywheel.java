package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private static final double TICKS_PER_REV = 28; // bare motor encoder resolution, adjust for actual flywheel motor

    private final DcMotorEx motor;
    private final PIDF pidf;
    private double targetRPM = 0;

    public Flywheel(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "flywheel");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidf = new PIDF(0, 0, 0, 0); // tune once flywheel is mounted
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return (motor.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public void update() {
        motor.setPower(pidf.calc(targetRPM, getRPM()));
    }
}

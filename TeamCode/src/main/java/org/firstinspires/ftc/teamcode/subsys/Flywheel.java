package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.PIDF;

public class Flywheel {
    public static final int MOTOR_TICKS_PER_MOTOR_REV = 28; // encoder ticks per motor revolution
    public static final double LAUNCH_RATIO = 1; // motor->flywheel ratio (output rotations / motor rotations)

    /** hardware **/
    private DcMotorEx launchMotor;

    /** stuff that changes **/
    private PIDF pidf;

    public Flywheel(HardwareMap hw) {
        // launch motors (all are DcMotorEx for current monitoring)
        launchMotor = hw.get(DcMotorEx.class, "launchMotor");

        // set up hardware
        launchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor

        // set up PIDF
        pidf = new PIDF(Tunables.flywheelP, Tunables.flywheelI, Tunables.flywheelD, Tunables.flywheelF); // create our PIDF controller for our launch motors
    }

    public void update(double RPM, double voltage) {
        pidf.updateTerms(Tunables.flywheelP, Tunables.flywheelI, Tunables.flywheelD, Tunables.flywheelF);
        double currentTPS = getTPS();
        double setpointTPS = RPMToTPS(RPM);
        double newPower;

        if (setpointTPS == 0) {
            newPower = 0.0;
        } else newPower = pidf.calc(setpointTPS, currentTPS) * getVoltageCompensation(voltage); // calc new motor power

        double clippedPower = Range.clip(newPower, -Tunables.maxFlywheelBraking, 1.0);

        launchMotor.setPower(clippedPower);
    }

    public void powerUpdate(double newPower) {
        double clippedPower = Range.clip(newPower, 0.0, 1.0);

        launchMotor.setPower(clippedPower);
    }

    public double getTPS() {
        // get TPS of flywheel in the fastest way while also being able to fall back between encoders
        return Math.abs(launchMotor.getVelocity());
    }

    /** getter methods **/

    public double getRPM() {
        return TPSToRPM(launchMotor.getVelocity());
    }

    // these should only be used for tuning
    public double getPower() {
        return launchMotor.getPower();
    }

    public double getCurrent() { // return sum current in amps
        return launchMotor.getCurrent(CurrentUnit.AMPS);
    }

    private static double getMotorTicksPerOutputRev() {
        return MOTOR_TICKS_PER_MOTOR_REV / LAUNCH_RATIO;
    }

    // return how much to multiply power by to compensate for voltage fall
    private double getVoltageCompensation(double voltage) {
        return Range.scale(voltage,
                Tunables.flywheelVoltageStart, Tunables.flywheelVoltageEnd,
                Tunables.flywheelCompensationMax, 1.0);  // reverse to scale inversely
    }

    public double TPSToRPM(double TPS) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (TPS / ticksPerOutputRev) * 60; // output RPM
    }

    public double RPMToTPS(double outputRPM) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return (outputRPM / 60) * ticksPerOutputRev; // motor TPS
    }
}

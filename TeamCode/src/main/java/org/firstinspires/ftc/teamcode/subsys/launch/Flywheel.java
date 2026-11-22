package org.firstinspires.ftc.teamcode.subsys.launch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.MotorDevice;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.io.WritePolicy;
import org.firstinspires.ftc.teamcode.subsys.PIDF;

public class Flywheel {
    /** configurables **/
    public static double PRIORITY_MULTIPLIER = 1; // PIDF makes small power changes, so boost them to keep up with bigger changes

    // PIDF tuned 9-28-2026
    public static double FLYWHEEL_P = 0.006;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 0.0004;
    public static double MAX_BRAKING = 1.0; // max amount of breaking (negative power) to apply to launch motors
    public static double VOLTAGE_START = 12.0; // start voltage of compensation scale
    public static double VOLTAGE_END = 13.5;   // end voltage of compensation scale
    public static double VOLTAGE_COMPENSATION_MAX = 1.2; // max amount to scale voltage
    public static boolean USING_VOLTAGE_COMPENSATION= false;

    /** end configurables **/

    public static final int MOTOR_TICKS_PER_MOTOR_REV = 28; // encoder ticks per motor revolution
    public static final double LAUNCH_RATIO = 1; // motor->flywheel ratio (output rotations / motor rotations)


    /** hardware **/
    private final MotorDevice launchMotor; // writes and encoder reads run on the I/O thread

    /** stuff that changes **/
    private final PIDF pidf;
    private double lastSetRPM = 0;

    public Flywheel(IoThread io, String ballType) {
        launchMotor = new MotorDevice(io, ballType + "FlywheelMotor", new MotorDevice.Config()
                .direction(DcMotorSimple.Direction.REVERSE)
                .zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT) // don't brake when we turn off the motor
                .readEncoder()
                .writePolicy(new WritePolicy(PRIORITY_MULTIPLIER, 100)));

        pidf = new PIDF(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F); // create our PIDF controller for our launch motors
    }

    /** read current draw (tuning only: each read costs the I/O thread a Lynx command) **/
    public void readCurrent(ReadPeriod period) {
        launchMotor.setCurrentReads(period);
    }

    public void update(double RPM, double voltage) {
        lastSetRPM = RPM;
        pidf.updateTerms(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        MotorDevice.EncoderReading reading = launchMotor.encoder();
        double currentTPS = Math.abs(reading.velocityTps());
        double setpointTPS = RPMToTPS(RPM);
        double newPower;

        if (setpointTPS == 0) {
            newPower = 0.0;
        } else newPower = pidf.calc(setpointTPS, currentTPS, reading.nanoTime()); // time the velocity was measured, not now
        if (USING_VOLTAGE_COMPENSATION) {
            newPower *= getVoltageCompensation(voltage);
        }

        double clippedPower = Range.clip(newPower, -MAX_BRAKING, 1.0);

        launchMotor.setPower(clippedPower);
    }

    public void powerUpdate(double newPower) {
        double clippedPower = Range.clip(newPower, 0.0, 1.0);

        launchMotor.setPower(clippedPower);
    }

    /** speed from the newest I/O reading **/
    public double getTPS() {
        return Math.abs(launchMotor.encoder().velocityTps());
    }

    /** getter methods **/

    public double getRPM() {
        return TPSToRPM(launchMotor.encoder().velocityTps());
    }

    public double getLastSetRPM() { return lastSetRPM; }

    // these should only be used for tuning
    public double getPower() {
        return launchMotor.writtenPower(); // last power the I/O thread actually wrote to the motor
    }

    /** amps; requires {@link #readCurrent} **/
    public double getCurrent() {
        return launchMotor.current().value();
    }

    private static double getMotorTicksPerOutputRev() {
        return MOTOR_TICKS_PER_MOTOR_REV / LAUNCH_RATIO;
    }

    // return how much to multiply power by to compensate for voltage fall
    private double getVoltageCompensation(double voltage) {
        return Range.scale(voltage,
                VOLTAGE_START, VOLTAGE_END,
                VOLTAGE_COMPENSATION_MAX, 1.0);  // reverse to scale inversely
    }

    public double TPSToRPM(double TPS) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return TPS * 60.0 / ticksPerOutputRev;
    }

    public double RPMToTPS(double outputRPM) {
        double ticksPerOutputRev = getMotorTicksPerOutputRev();
        return outputRPM * ticksPerOutputRev / 60.0;
    }
}

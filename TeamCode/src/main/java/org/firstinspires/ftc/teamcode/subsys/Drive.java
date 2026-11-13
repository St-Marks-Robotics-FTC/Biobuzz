/** mecanum drivetrain with every drive motor written through the hardware scheduler **/

package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SMotor;

public class Drive {
    private static final double PRIORITY_MULTIPLIER = 1;

    private SMotor frontLeft, frontRight, backLeft, backRight;

    public Drive(HWScheduler scheduler) {
        frontLeft = new SMotor("frontLeft", PRIORITY_MULTIPLIER, scheduler.hw());
        frontRight = new SMotor("frontRight", PRIORITY_MULTIPLIER, scheduler.hw());
        backLeft = new SMotor("backLeft", PRIORITY_MULTIPLIER, scheduler.hw());
        backRight = new SMotor("backRight", PRIORITY_MULTIPLIER, scheduler.hw());

        frontLeft.m.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.m.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.m.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.m.setDirection(DcMotorEx.Direction.FORWARD);

        for (SMotor motor : new SMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            if (Tunables.teleBrake) {
                motor.m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else {
                motor.m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
            motor.m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            scheduler.addDevice(motor);
        }
    }

    /** robot-centric drive. speedMultiplier scales every input (1 = full speed, use less for slow mode) **/
    public void drive(double forward, double strafe, double turn, double speedMultiplier) {
        forward *= speedMultiplier;
        strafe *= speedMultiplier;
        turn *= speedMultiplier;

        double fl = forward - strafe - turn;
        double bl = forward + strafe - turn;
        double fr = forward + strafe + turn;
        double br = forward - strafe + turn;

        double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    public void driveFieldCentric(double forward, double strafe, double turn, double heading, double speedMultiplier) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;

        drive(robotForward, robotStrafe, turn, speedMultiplier);
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }

    // write zero power skipping scheduler
    public void stopNow() {
        stop();
        for (SMotor motor : new SMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.forceUpdate();
        }
    }
}

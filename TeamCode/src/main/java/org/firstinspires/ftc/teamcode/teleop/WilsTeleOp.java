/** fully manual four-wheel drive: each joystick axis drives exactly one motor, no mixing **/

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "WilsTeleOp", group = "TeleOp")
public class WilsTeleOp extends OpMode {
    private static final double SLOW_MODE_MIN_SCALE = 0.3; // power scale at full left trigger
    private static final double STOPPED_THRESHOLD = 0.05; // |power| below this counts as "not moving"

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // tracks each motor's currently-applied zero power behavior so we only
    // send a hardware command when it actually needs to change.
    private DcMotor.ZeroPowerBehavior frontLeftBehavior;
    private DcMotor.ZeroPowerBehavior frontRightBehavior;
    private DcMotor.ZeroPowerBehavior backLeftBehavior;
    private DcMotor.ZeroPowerBehavior backRightBehavior;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // matches the physical wiring: gearboxes are mirrored left/right, so the
        // right-side motors must spin the opposite direction to drive the robot straight.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        frontRightBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        backLeftBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        backRightBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        frontLeft.setZeroPowerBehavior(frontLeftBehavior);
        frontRight.setZeroPowerBehavior(frontRightBehavior);
        backLeft.setZeroPowerBehavior(backLeftBehavior);
        backRight.setZeroPowerBehavior(backRightBehavior);
    }

    @Override
    public void loop() {
        // left trigger: slow mode, scales every motor down to SLOW_MODE_MIN_SCALE at full pull.
        double slowMode = 1.0 - (1.0 - SLOW_MODE_MIN_SCALE) * gamepad1.left_trigger;

        // right trigger: held motors (commanded power ~0) brake instead of coasting;
        // motors that are actually being driven are unaffected either way since zero
        // power behavior only applies once their power drops to zero.
        boolean brakeWhenStopped = gamepad1.right_trigger > 0.5;

        // each stick axis drives exactly one motor directly - no mecanum mixing.
        double frontLeftPower = -gamepad1.left_stick_y * slowMode;   // left stick up/down -> front left
        double frontRightPower = -gamepad1.right_stick_y * slowMode; // right stick up/down -> front right
        double backLeftPower = gamepad1.left_stick_x * slowMode;     // left stick right = forward, left = backward
        double backRightPower = -gamepad1.right_stick_x * slowMode;  // right stick left = forward, right = backward

        frontLeftBehavior = applyPower(frontLeft, frontLeftPower, frontLeftBehavior, brakeWhenStopped);
        frontRightBehavior = applyPower(frontRight, frontRightPower, frontRightBehavior, brakeWhenStopped);
        backLeftBehavior = applyPower(backLeft, backLeftPower, backLeftBehavior, brakeWhenStopped);
        backRightBehavior = applyPower(backRight, backRightPower, backRightBehavior, brakeWhenStopped);

        telemetry.addData("slow mode", "%.0f%%", slowMode * 100);
        telemetry.addData("brake when stopped", brakeWhenStopped);
        telemetry.addData("frontLeft", "%.2f", frontLeftPower);
        telemetry.addData("frontRight", "%.2f", frontRightPower);
        telemetry.addData("backLeft", "%.2f", backLeftPower);
        telemetry.addData("backRight", "%.2f", backRightPower);
        telemetry.update();
    }

    /**
     * Sets the motor's zero power behavior to BRAKE if it's not moving and brakeWhenStopped is
     * true, or FLOAT otherwise, then applies power. The hardware write for zero power behavior
     * is skipped unless it actually needs to change.
     *
     * @return the zero power behavior now applied to the motor, for tracking on the next loop.
     */
    private DcMotor.ZeroPowerBehavior applyPower(DcMotor motor, double power, DcMotor.ZeroPowerBehavior currentBehavior, boolean brakeWhenStopped) {
        boolean isMoving = Math.abs(power) > STOPPED_THRESHOLD;
        DcMotor.ZeroPowerBehavior desiredBehavior = (!isMoving && brakeWhenStopped)
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;

        if (desiredBehavior != currentBehavior) {
            motor.setZeroPowerBehavior(desiredBehavior);
        }

        motor.setPower(power);
        return desiredBehavior;
    }
}

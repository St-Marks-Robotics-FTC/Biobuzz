package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.GoalTargeting;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = true; // start in field-centric mode

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);

        follower.setPose(HandoffState.pose);

        telemetryM.debug("init time: " + loopTimer.get(TimeUnit.MILLISECONDS) + "ms");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        loopTimer.reset();

        double slowModeMultiplier = 1.0 - (gamepad1.left_trigger * 0.7); // 1.0 normally, down to 0.3 while held (slow mode)

        if (gamepad1.startWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }

        if (gamepad1.aWasPressed()) {
            if (robot.intake.getState() == Intake.State.OFF) {
                robot.intake.forward();
            } else {
                robot.intake.off();
            }
        }

        if (gamepad1.xWasPressed()) {
            if (robot.intake.getState() == Intake.State.FORWARD){
                robot.intake.reverse();
            } else if (robot.intake.getState() == Intake.State.REVERSE) {
                robot.intake.forward();
            }
        }

        if (gamepad1.bWasPressed()) {
            if (robot.flywheel.isRunning) {
                robot.flywheel.setRPM(0);
                robot.flywheel.isRunning = false;
            } else {
                robot.flywheel.setRPM(3000);
                robot.flywheel.isRunning = true;
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (!robot.flywheel.isRunning) {
                robot.flywheel.setRPM(3000);
                robot.flywheel.isRunning = true;
            } else {
                robot.flywheel.setRPM(
                        robot.flywheel.getTargetRPM() + 20
                );
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            if (!robot.flywheel.isRunning) {
                robot.flywheel.setRPM(3000);
                robot.flywheel.isRunning = true;
            } else {
                robot.flywheel.setRPM(
                        robot.flywheel.getTargetRPM() - 20
                );
            }
        }

        if (gamepad1.rightBumperWasPressed() && robot.transfer.getState() == Transfer.State.CLOSED) {
            robot.transfer.open();
        }
        if (gamepad1.rightBumperWasReleased() && robot.transfer.getState() == Transfer.State.OPEN) {
            robot.transfer.close();
        }

        robot.flywheel.update(); // re-run PID every loop so RPM actually converges on target

        if (isRobotCentric) { // robot-centric control
            double forward = -gamepad1.left_stick_y * slowModeMultiplier;
            double strafe = -gamepad1.left_stick_x * slowModeMultiplier;
            double turn = -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier; // reduce speed by our turn rate

            follower.manual(forward, strafe, turn);
        } else { // field-centric control
            double flipControl;
            if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
            else flipControl = 1; // red team doesn't need to be flipped

            double forward = -gamepad1.left_stick_y * slowModeMultiplier * flipControl;
            double strafe = -gamepad1.left_stick_x * slowModeMultiplier * flipControl;
            double turn = -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier; // reduce speed by our turn rate

            DrivePowers powers = ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading());
            follower.manual(powers);
        }

        follower.update();
        updateGoalTelemetry();
        updateLaunchSystemTelemetry();

        if (Tunables.isDebugging) updateTelemetry();

        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }

    private void updateTelemetry() {
        // odo
        telemetryM.debug("current heading: " + follower.pose().heading());
        telemetryM.addData("odo x", follower.pose().x());
        telemetryM.addData("odo y", follower.pose().y());
    }

    private void updateGoalTelemetry() {
        telemetryM.addData("--- Blue Goals ---", "");
        for (FieldConstants.Goal g : FieldConstants.BLUE_GOALS) {
            reportGoal(g);
        }

        telemetryM.addData("--- Red Goals ---", "");
        for (FieldConstants.Goal g : FieldConstants.RED_GOALS) {
            reportGoal(g);
        }
    }

    private void updateLaunchSystemTelemetry() {
        telemetryM.addData("--- DC Motors ---", "");
        telemetryM.addData("Flywheel Current RPM", robot.flywheel.getRPM());
        telemetryM.addData("Flywheel Target RPM", robot.flywheel.getTargetRPM());
        telemetryM.addData("Intake State", robot.intake.getState());
        telemetryM.addData("Transfer Servo State", robot.transfer.getState());
    }

    private void reportGoal(FieldConstants.Goal g) {
        double dist = GoalTargeting.distanceTo(follower.pose(), g);
        boolean inRange = GoalTargeting.isInRange(follower.pose(), g);
        boolean inAngle = GoalTargeting.isInAngle(follower.pose(), g);

        telemetryM.addData(g.name + " dist (in)", String.format("%.1f", dist));
        telemetryM.addData(g.name + " can score", (inRange && inAngle) ? "YES" : (inRange ? "angle bad" : "out of range"));
    }

}


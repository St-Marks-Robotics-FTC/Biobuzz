package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Field;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Field field;
    private Vision vision;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = true; // start in robot-centric mode
    private double setRPM = Tunables.manualStartingRPM;
    private boolean flywheelOn = true;
    private boolean isTurning = false;

    private boolean flowerIsDown = true;
    private Pose lastTurnPose = new Pose(0, 0, 0); // don't be null cause i don't like crashes
    private Field.ScoringData lastScoringData = null;

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        robot = new Robot(hardwareMap);
        field = new Field(isBlueTeam());
        vision = new Vision(hardwareMap, isBlueTeam());
        follower = Constants.create(hardwareMap);

        follower.setPose(HandoffState.pose);

        robot.transfer.close();

        telemetryM.addLine("init time: " + loopTimer.milliseconds() + "ms");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        robot.intake.forward();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        vision.update();

        handleDrive();
        handleFlywheel();
        handleIntake();
        handleShoot();
        handleFlower();


        if (gamepad1.xWasPressed()) { // reset field centric heading
            follower.setPose(follower.pose().withHeading(Math.toRadians(90)));
        }

        follower.update();

        updateTelemetry();
    }

    private void handleDrive() {
        if (gamepad1.startWasPressed()) isRobotCentric = !isRobotCentric;

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode

        if (!isTurning) {
            if (isRobotCentric) { // robot-centric control
                follower.manual(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier // reduce speed by our turn rate
                );
            } else { // field-centric control
                double flipControl;
                if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
                else flipControl = 1; // red team doesn't need to be flipped

                DrivePowers powers = ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y * slowModeMultiplier * flipControl,
                        -gamepad1.left_stick_x * slowModeMultiplier * flipControl,
                        -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                        follower.pose().heading()
                );

                follower.manual(powers);
            }
        } else {
            // under what conditions to exit turning
            if (Math.abs(follower.pose().heading() - lastTurnPose.heading()) < Tunables.shootHeadingMargin) isTurning = false;
            // emergency exit if pose is invalid
            if (follower.pose() == null || follower.pose().equals(new Pose(0, 0, 0))) {
                isTurning = false;
            }

            if (gamepad1.backWasPressed()) isTurning = false; // emergency exit
        }
    }

    private void handleFlywheel() {
        if (gamepad1.bWasPressed()) flywheelOn = !flywheelOn;

        if (gamepad1.dpadUpWasPressed()) setRPM += Tunables.adjustRPM; // increment by adjustRPM
        if (gamepad1.dpadDownWasPressed()) setRPM -= Tunables.adjustRPM; // decrement by adjustRPM
        if (gamepad1.dpadLeftWasPressed()) setRPM -= (Tunables.adjustRPM / 2.0); // decrement by half of adjustRPM
        if (gamepad1.dpadRightWasPressed()) setRPM += (Tunables.adjustRPM / 2.0); // increment by half of adjustRPM

        if (setRPM < 0) setRPM = 0;

        if (flywheelOn) {
            robot.flywheel.update(setRPM, robot.getExpansionHubVoltage());
        } else {
            robot.flywheel.update(0, robot.getExpansionHubVoltage());
        }
    }

    private void handleFlower() {
        if (gamepad1.right_trigger_pressed) robot.flower.goDown();
        else robot.flower.goUp();
    }

    private void handleIntake() {
        if (gamepad1.aWasPressed()) robot.intake.toggle();
        if (gamepad1.xWasPressed()) robot.intake.toggleReverse();
    }

    private void handleShoot() {
        boolean shootPressed = gamepad1.rightBumperWasPressed();

        if (shootPressed) robot.startLaunch();

        robot.update();

        // TODO: add odo fallback on top of vision hive state data
        if (vision.getLastHiveState() == Vision.HiveState.AUDIENCE_UP) {
            lastScoringData = field.getScoringData(true, follower.pose());
        } else if (vision.getLastHiveState() == Vision.HiveState.SCORING_UP) {
            lastScoringData = field.getScoringData(false, follower.pose());
        } else {
            return; // hive state unknown - can't auto turn
        }


        if (gamepad1.leftBumperWasPressed() && lastScoringData != null) {
            // they got rid of follower.turnTo() so we have to make it ourselves
            lastTurnPose = new Pose(follower.pose().x(), follower.pose().y(), lastScoringData.angle());
            follower.hold(lastTurnPose);
            isTurning = true;
        }
    }

    private void updateTelemetry() {
        // only output debug values if we're debugging - these kill our loop times
        if (Tunables.isDebugging) {
            // intake
            telemetryM.addData("power save", robot.intake.getPowerSaveTrigger());
            // flywheel
            telemetryM.addData("current RPM", robot.flywheel.getRPM());
            telemetryM.addData("flywheel pwr", robot.flywheel.getPower());

            // odo
            telemetryM.debug("heading (deg): " + Math.toDegrees(follower.pose().heading()));
            telemetryM.addData("odo x", follower.pose().x());
            telemetryM.addData("odo y", follower.pose().y());

            // Field
            if (lastScoringData != null) {
                telemetryM.addLine(lastScoringData.toString());
            } else {
                telemetryM.addLine("lastScoringData is null!");
            }
        }

        // always output these to telemetry
        telemetryM.addData("desired RPM", setRPM);
        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }
}


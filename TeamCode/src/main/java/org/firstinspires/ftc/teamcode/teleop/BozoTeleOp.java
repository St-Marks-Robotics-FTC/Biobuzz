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
    private boolean isRobotCentric = true; // start in field-centric mode
    private double setRPM = 3600;
    private boolean flywheelOn = true;
    private boolean isTurning = false;

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        robot = new Robot(hardwareMap);
        field = new Field(isBlueTeam());
        vision = new Vision(hardwareMap, isBlueTeam());
        follower = Constants.create(hardwareMap);

        follower.setPose(HandoffState.pose);

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
        handleTransfer();
        handleAutoTurn();

        follower.update();

        if (Tunables.isDebugging) updateTelemetry();

        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
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
            //if (gamepad1.leftBumperWasPressed() || !follower.isBusy()) isTurning = false;
            if (gamepad1.backWasPressed()) isTurning = false;
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
            robot.flywheel.update(setRPM);
        } else {
            robot.flywheel.update(0);
        }
    }

    private void handleIntake() {
        if (gamepad1.aWasPressed()) robot.intake.toggle();
        if (gamepad1.xWasPressed()) robot.intake.toggleReverse();
    }

    private void handleTransfer() {
        if (gamepad1.right_bumper) {
            robot.transfer.open();
            robot.intake.forwardLaunching();
        } else {
            robot.transfer.close();
            if (robot.intake.getState() == Intake.State.FORWARD_LAUNCHING) {
                robot.intake.forward();
            }
        }
    }

    private void handleAutoTurn() {
        Field.ScoringData scoringData;
        if (vision.getLastHiveState() == Vision.HiveState.AUDIENCE_UP) {
            scoringData = field.getScoringData(true, follower.pose());
        } else if (vision.getLastHiveState() == Vision.HiveState.SCORING_UP) {
            scoringData = field.getScoringData(false, follower.pose());
        } else {
            return; // hive state unknown - can't auto turn
        }

        telemetryM.addLine(scoringData.toString());

        if (gamepad1.leftBumperWasPressed()) {
            // they got rid of follower.turnTo() so we have to make it ourselves
            Pose turnPose = new Pose(follower.pose().x(), follower.pose().y(), scoringData.angle());
            follower.hold(turnPose);
            isTurning = true;
        }
    }

    private void updateTelemetry() {
        // flywheel
        telemetryM.addData("desired RPM", setRPM);
        telemetryM.addData("current RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel pwr", robot.flywheel.getPower());

        // odo
        telemetryM.debug("heading (deg): " + Math.toDegrees(follower.pose().heading()));
        telemetryM.addData("odo x", follower.pose().x());
        telemetryM.addData("odo y", follower.pose().y());
    }
}


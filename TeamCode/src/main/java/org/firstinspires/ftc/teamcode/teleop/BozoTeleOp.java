package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Drive;
import org.firstinspires.ftc.teamcode.subsys.Field;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Drive drive;
    private Field field;
    private Vision vision;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = true; // start in robot-centric mode
    private double setRPM = 3600;
    private boolean flywheelOn = true;
    private boolean isTurning = false;
    private Pose lastTurnPose = new Pose(0, 0, 0); // don't be null cause i don't like crashes

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        robot = new Robot(hardwareMap);
        drive = new Drive(hardwareMap, robot.scheduler); // teleop only - auto drives through the follower
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


        if (gamepad1.backWasPressed()) { // reset field centric heading
            follower.setPose(follower.pose().withHeading(Math.toRadians(90)));
        }

        follower.update();

        robot.update(loopTimer.milliseconds()); // write subsystem hardware with whatever time is left in the loop

        double loopTime = loopTimer.get(TimeUnit.MILLISECONDS);

        if (Tunables.isDebugging) updateTelemetry();

        telemetryM.addData("loop time (before telemetry)", loopTime);
        telemetryM.update(telemetry);
    }

    private void handleDrive() {
        if (gamepad1.startWasPressed()) isRobotCentric = !isRobotCentric;

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode

        if (!isTurning) {
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x * Tunables.turnRateMultiplier; // reduce speed by our turn rate

            if (isRobotCentric) { // robot-centric control
                drive.drive(forward, strafe, turn, slowModeMultiplier);
            } else { // field-centric control
                double flipControl;
                if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
                else flipControl = 1; // red team doesn't need to be flipped

                drive.driveFieldCentric(forward * flipControl, strafe * flipControl, turn, follower.pose().heading(), slowModeMultiplier);
            }
        } else {
            boolean exitTurning = false;

            // under what conditions to exit turning
            if (Math.abs(follower.pose().heading() - lastTurnPose.heading()) < Tunables.shootHeadingMargin) exitTurning = true;
            // emergency exit if pose is invalid
            if (follower.pose() == null || follower.pose().equals(new Pose(0, 0, 0))) {
                exitTurning = true;
            }

            if (exitTurning) endTurn();
        }
    }

    // hand drive motors from follower back to Drive
    private void endTurn() {
        follower.stop(); // follower goes idle so it stops computing hold powers
        follower.drivetrain.stop(); // zero the motors now so pedro's cache, ours (zeroed in startTurn) and the hardware all match
        isTurning = false;
    }

    // hand motors from Drive to follower
    private void startTurn(Pose target) {
        // zero the motors now so pedro's cache, ours and the hardware all match
        // skip if the follower already has the motors, otherwise we'd write over its hold powers
        if (!isTurning) drive.stopNow();
        lastTurnPose = target;
        follower.hold(lastTurnPose);
        isTurning = true;
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

    private void handleShoot() {
        boolean shootPressed = gamepad1.rightBumperWasPressed();

        if (shootPressed) robot.startLaunch();

        robot.updateLaunch();

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
            startTurn(new Pose(follower.pose().x(), follower.pose().y(), scoringData.angle()));
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

        // hardware scheduler
        telemetryM.addLine(robot.scheduler.getStatus());
    }
}


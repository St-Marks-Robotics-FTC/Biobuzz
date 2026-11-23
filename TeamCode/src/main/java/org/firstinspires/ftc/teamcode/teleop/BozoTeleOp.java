package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Field;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private IoThread io;
    private Robot robot;
    private Field field;
    private Vision vision;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = true; // start in robot-centric mode
    private double setRPM = Tunables.manualStartingRPM;
    private boolean flywheelOn = true;
    private boolean isHolding = false;
    private boolean isManualRPM = false;
    private boolean rumble = false;
    private Timer rumbleTimer;
    private Pose lastTurnPose = new Pose(0, 0, 0); // don't be null cause i don't like crashes
    private Field.ScoringData lastScoringData;
    private boolean lastCanScore = false;
    private long previousNanoTime = 0L;

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        rumbleTimer = new Timer();

        io = new IoThread(hardwareMap);
        robot = new Robot(io);
        field = new Field(isBlueTeam());
        vision = new Vision(hardwareMap, isBlueTeam());
        follower = Constants.create(io); // drives through follower.manual(); turn-to-goal uses follower.hold()

        follower.setPose(HandoffState.pose);

        robot.transfer.close();
        io.start();

        telemetryM.addLine("init time: " + loopTimer.milliseconds() + "ms");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        robot.intake.forward();
    }

    @Override
    public void loop() {
        io.awaitNewCycle(); // fresh readings; the loop runs at the I/O cycle rate
        loopTimer.reset();

        vision.update();

        updateLastScoringData();
        handleDrive();
        handleFlywheel();
        handleIntake();
        handleShoot();
        handleFlower();


        if (gamepad1.backWasPressed()) { // reset field centric heading
            relocalizeOdo();
        }

        follower.update();

        updateTelemetry();
    }

    @Override
    public void stop() {
        if (io != null) io.stop(); // stop the I/O thread and flush the last commands
    }

    private void handleDrive() {
        if (gamepad1.startWasPressed()) isRobotCentric = !isRobotCentric;

        double slowModeMultiplier = Range.scale(gamepad1.left_trigger, 0, 1, 1, 0.5);

        if (!isHolding) {
            double forward = -gamepad1.left_stick_y * slowModeMultiplier;
            double strafe = -gamepad1.left_stick_x * slowModeMultiplier;
            double turn = -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier; // reduce speed by our turn rate

            if (gamepad1.left_bumper) {
                // auto aim without hold
                long nanoTime = System.nanoTime();
                double deltaTime = 0;
                if (previousNanoTime != 0L) deltaTime = (nanoTime - previousNanoTime) / 1e9;
                previousNanoTime = nanoTime;
                Pose turnPose = new Pose(follower.pose().x(), follower.pose().y(), lastScoringData.angle());
                DrivePowers pedroPowers = follower.algorithm().calculateHold(follower.drivetrain, turnPose, follower.localizer.state(), true, deltaTime);
                turn = pedroPowers.turn();
            }

            if (isRobotCentric) { // robot-centric control
                follower.manual(forward, strafe, turn);
            } else { // field-centric control
                double flipControl;
                if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
                else flipControl = 1; // red team doesn't need to be flipped

                driveFieldCentric(forward * flipControl, strafe * flipControl, turn, follower.pose().heading());
            }
        } else {
            // we're holding so do nothing with drivetrain
            // emergency exit
            if (follower.pose() == null || follower.pose().equals(new Pose(0, 0, 0))) {
                endHold();
            }
        }
    }

    /** rotate a field-relative stick vector into the robot's frame, then drive it **/
    private void driveFieldCentric(double forward, double strafe, double turn, double heading) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;

        follower.manual(robotForward, robotStrafe, turn);
    }

    // hand the drivetrain back to the driver: the follower idles until handleDrive calls manual() next loop
    private void endHold() {
        follower.stop(); // follower goes idle so it stops computing hold powers
        follower.drivetrain.stop(); // zero the motors now so pedro's cache and the hardware match
        isHolding = false;
    }

    // hold position while turning to the target heading
    private void startHold(Pose target) {
        lastTurnPose = target;
        follower.hold(lastTurnPose);
        isHolding = true;
        previousNanoTime = 0L; // reset on first press to avoid crazy dt skips
    }

    private void relocalizeOdo() {
        if (isBlueTeam()) {
            follower.setPose(new Pose(144 - Tunables.robotWidthToCenter, Tunables.robotLengthToCenter, Math.toRadians(90)));
        } else {
            follower.setPose(new Pose(Tunables.robotWidthToCenter, Tunables.robotLengthToCenter, Math.toRadians(90)));
        }

    }

    private void handleFlywheel() {
        if (gamepad1.bWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.yWasPressed()) isManualRPM = !isManualRPM;

        if (gamepad1.dpadUpWasPressed()) setRPM += Tunables.adjustRPM; // increment by adjustRPM
        if (gamepad1.dpadDownWasPressed()) setRPM -= Tunables.adjustRPM; // decrement by adjustRPM
        if (gamepad1.dpadLeftWasPressed()) setRPM -= (Tunables.adjustRPM / 2.0); // decrement by half of adjustRPM
        if (gamepad1.dpadRightWasPressed()) setRPM += (Tunables.adjustRPM / 2.0); // increment by half of adjustRPM

        if (setRPM < 0) setRPM = 0;

        if (flywheelOn) {
            if (isManualRPM) {
                robot.flywheel.update(setRPM, robot.getExpansionHubVoltage());
            } else { // automatic RPM
                robot.flywheel.update(lastScoringData.idealRPM(), robot.getExpansionHubVoltage());
            }
        } else {
            robot.flywheel.update(0, robot.getExpansionHubVoltage());
        }
    }

    private void handleFlower() {
        double trig = gamepad1.right_trigger;
        if (trig <= 0.1) {
            robot.flower.goUp();
        } else if (trig <= .8) {
            robot.flower.startShake();
        } else {
            robot.flower.goDown();
        }
    }

    private void handleIntake() {
        if (gamepad1.aWasPressed()) robot.intake.toggle();
        if (gamepad1.xWasPressed()) robot.intake.reverse();
        if (gamepad1.xWasReleased()) robot.intake.forward();
    }

    private void handleShoot() {
        if (gamepad1.rightBumperWasPressed()) {
            robot.startLaunch();
            // they got rid of follower.turnTo() so we have to make it ourselves
            startHold(new Pose(follower.pose().x(), follower.pose().y(), lastScoringData.angle()));
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.stopLaunch();
            endHold();
        }

        boolean canScore = lastScoringData.canScore();

        if (!lastCanScore && canScore) { // start rumbling
            rumble = true;
            rumbleTimer.reset();
            gamepad1.rumble(150);
        } else if (lastCanScore && !canScore) { // stop rumbling
            rumble = false;
            rumbleTimer.reset();
            gamepad1.stopRumble();
        }

        if (rumbleTimer.milliseconds() > 120 && rumble) { // keep rumbling
            rumbleTimer.reset();
            gamepad1.rumble(150);
        }

        lastCanScore = canScore;

        robot.update();
    }

    private void updateLastScoringData() {
        lastScoringData = field.getScoringData(follower.pose());
    }

    private void updateTelemetry() {
        // super secret isDebugging toggle
        if (gamepad1.left_stick_button && gamepad1.right_stick_button) Tunables.isDebugging = !Tunables.isDebugging;

        double loopTime = loopTimer.get(TimeUnit.MILLISECONDS);

        // always output these to telemetry
        telemetryM.addData("desired RPM", robot.flywheel.getLastSetRPM());
        telemetryM.addData("loop time (before telemetry)", loopTime);
        telemetryM.addData("I/O cycle (ms)", io.lastCycleMillis());

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
            telemetryM.addData("last hive state", vision.getLastHiveState());
            if (lastScoringData != null) {
                telemetryM.addLine(lastScoringData.toString());
            } else {
                telemetryM.addLine("lastScoringData is null!");
            }

            // I/O thread
            telemetryM.addLine(io.status());
        }

        telemetryM.update(telemetry);
    }
}


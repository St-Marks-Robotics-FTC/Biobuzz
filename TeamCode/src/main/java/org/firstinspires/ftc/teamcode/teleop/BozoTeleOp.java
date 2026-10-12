package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
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
    private boolean isRobotCentric = true; // start in robot-centric mode (start button toggles field-centric)
    private double setRPM = 3100;
    private boolean flywheelOn = true;
    private boolean isTurning = false;
    private Pose lastTurnPose = new Pose(0, 0, 0); // don't be null cause i don't like crashes
    private Timer turnTimer; // time since the current auto turn started
    private Timer settleTimer; // time the heading has been on target during the current auto turn
    private Timer fuseTimer; // time since odometry was last corrected by the Limelight
    private boolean turnSettled = false; // heading is currently within the margin
    private String aimInfo = "none yet"; // what the last auto turn aimed at (for telemetry)
    private boolean headingKnown; // odometry heading matches the real robot (auto handed it off, or X was pressed)
    private boolean positionKnown; // odometry x/y matches the real robot (auto handed it off, or the Limelight corrected it)

    @Override
    public void init() {
        loopTimer = new Timer();
        turnTimer = new Timer();
        settleTimer = new Timer();
        fuseTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        robot = new Robot(hardwareMap);
        field = new Field(isBlueTeam());
        vision = new Vision(hardwareMap, isBlueTeam());
        follower = Constants.create(hardwareMap);

        follower.setPose(HandoffState.pose);
        headingKnown = positionKnown = HandoffState.fromAuto; // without an auto the pose is just a default guess until X / the Limelight fix it

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
        handleLocalization();

        handleDrive();
        handleFlywheel();
        handleIntake();
        handleShoot();

        if (gamepad1.xWasPressed()) { // reset field centric heading
            follower.setPose(follower.pose().withHeading(Math.toRadians(90)));
            headingKnown = true;
        }

        follower.update();

        if (Tunables.isDebugging) updateTelemetry();

        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }

    // Pull odometry x/y toward the Limelight position. Skipped while moving (frame latency would skew the fix) and while
    // auto turning (the turn holds a fixed point, so shifting the pose would make the robot chase it).
    private void handleLocalization() {
        if (!Tunables.visionLocalization || isTurning || !vision.hasNewPose()) return;
        if (fuseTimer.milliseconds() < Tunables.visionFuseIntervalMs) return;
        // hasNewPose() stays true until taken, so a fix from before the camera lost the tags would otherwise be applied late
        if (vision.getStaleness() > Tunables.visionMaxStalenessMs) return;

        Velocity velocity = follower.velocity();
        if (Math.hypot(velocity.vx, velocity.vy) > Tunables.visionMaxSpeed || Math.abs(velocity.omega) > Tunables.visionMaxTurnRate) return;

        Pose fix = vision.takePose();
        Pose odo = follower.pose();
        double dx = fix.x() - odo.x();
        double dy = fix.y() - odo.y();

        // small gaps are smoothed out, big ones (e.g. odometry started from the wrong pose) are fixed immediately
        double alpha = Math.hypot(dx, dy) > Tunables.visionSnapDistance ? 1.0 : Tunables.visionBlend;

        follower.setPose(new Pose(odo.x() + dx * alpha, odo.y() + dy * alpha, odo.heading())); // heading stays with odometry
        positionKnown = true;
        fuseTimer.reset();
    }

    private void handleDrive() {
        if (gamepad1.startWasPressed()) isRobotCentric = !isRobotCentric;
        boolean backPressed = gamepad1.backWasPressed(); // always read: a press made while not turning would otherwise cancel the next auto turn instantly

        double slowModeMultiplier = 1 - gamepad1.left_trigger * (1 - Tunables.slowModeMin); // 1.0 released, slowModeMin fully pressed (never 0: the robot must still move)

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
            // headings wrap at 0/2pi, so compare with Angle instead of subtracting
            boolean onTarget = Angle.smallestDifference(follower.pose().heading(), lastTurnPose.heading()) < Tunables.shootHeadingMargin;

            if (!onTarget) {
                turnSettled = false;
            } else if (!turnSettled) {
                turnSettled = true; // just arrived: wait for the robot to stop moving before handing back the sticks
                settleTimer.reset();
            } else if (settleTimer.milliseconds() >= Tunables.aimSettleMs) {
                isTurning = false;
            }

            if (turnTimer.milliseconds() > Tunables.aimTimeoutMs) isTurning = false; // never trap the driver

            if (backPressed) isTurning = false; // emergency exit
        }
    }

    private void handleFlywheel() {
        if (gamepad1.bWasPressed()) flywheelOn = !flywheelOn;

        if (gamepad1.dpadUpWasPressed()) setRPM += Tunables.adjustRPM; // increment by adjustRPM
        if (gamepad1.dpadDownWasPressed()) setRPM -= Tunables.adjustRPM; // decrement by adjustRPM

        if (setRPM < 0) setRPM = 0;

        if (flywheelOn) {
            robot.flywheel.update(setRPM);
        } else {
            robot.flywheel.update(0);
        }
    }

    private void handleIntake() {
        if (gamepad1.aWasPressed()) robot.intake.toggle();
        if (gamepad1.yWasPressed()) robot.intake.toggleReverse(); // Y, not X: X is the heading reset (see loop)
    }

    private void handleShoot() {
        boolean shootPressed = gamepad1.rightBumperWasPressed();
        if (shootPressed) robot.startLaunch();

        robot.updateLaunch();

        // always read the bumper: if we skipped it, a press made earlier would be remembered and start a turn later
        if (gamepad1.leftBumperWasPressed()) startAutoTurn();
    }

    // turn in place to face the goal the Limelight says is up
    private void startAutoTurn() {
        // the target is an absolute field heading, so a pose that doesn't match the real robot turns it the wrong way / at the wrong goal
        if (!headingKnown || (!positionKnown && Tunables.visionLocalization)) {
            aimInfo = "blocked: " + (!headingKnown
                    ? "press X with the robot facing the top of the field"
                    : "no Limelight position fix yet (needs " + Tunables.visionMinTags + "+ tags in view while slow)");
            gamepad1.rumbleBlips(3);
            return;
        }

        Pose pose = follower.pose();

        boolean audienceUp;
        String source;
        if (vision.getLastHiveState() == Vision.HiveState.AUDIENCE_UP) {
            audienceUp = true;
            source = "vision";
        } else if (vision.getLastHiveState() == Vision.HiveState.SCORING_UP) {
            audienceUp = false;
            source = "vision";
        } else {
            // the camera hasn't confirmed the hive yet -> best guess is the closest goal
            audienceUp = field.nearestGoalIsAudience(pose);
            source = "nearest goal (hive unknown)";
        }

        Field.ScoringData scoringData = field.getScoringData(audienceUp, pose);

        aimInfo = (audienceUp ? "audience" : "scoring") + " goal from " + source + ", " + scoringData;

        // they got rid of follower.turnTo() so we have to make it ourselves
        lastTurnPose = new Pose(pose.x(), pose.y(), scoringData.angle());

        if (scoringData.canScore()){gamepad1.rumble(250);}

        follower.hold(lastTurnPose);
        isTurning = true;
        turnSettled = false;
        turnTimer.reset();
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

        // vision
        telemetryM.addData("vision tags", vision.getLastTagCount());
        telemetryM.addData("hive state", vision.getLastHiveState() + " (" + (int) vision.getHiveAgeMs() + " ms ago)");
        telemetryM.addData("hive tilt (deg)", Math.toDegrees(vision.getLastHiveTilt()));
        telemetryM.addData("last auto turn", aimInfo);
    }
}


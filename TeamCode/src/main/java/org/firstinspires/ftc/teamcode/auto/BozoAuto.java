package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;

/**
 * Autonomous: start at the start pose, drive linearly to the shoot pose (flywheel spinning up on the way), shoot,
 * drive linearly to an intermediate pose and then to the refuel pose to pick up balls, drive back to the shoot pose,
 * then (hive tipped) curve to the scoring side and shoot again. For each of two flowers it then drives at full speed
 * to an intermediate pose (a line for flower 1, a curve for flower 2), drives a straight line at reduced speed into
 * the flower, collects, backs out to the same intermediate pose. After flower 1 it returns by line to the scoring-side
 * shoot pose and shoots; after flower 2 it instead returns by line to the original shoot pose, shoots, and then
 * curves to the park pose. If the hive did not tip, the auto reports a failure
 * in telemetry (TEMP_FAILED) and stops.
 *
 * Sequence: TRAVEL_TO_FIRST_SHOOT -> SPIN_UP_FIRST_SHOOT -> FEED_FIRST_SHOOT -> TRAVEL_TO_INTERMEDIATE
 *           -> TRAVEL_TO_REFUEL -> REFUEL -> TRAVEL_TO_SHOOT
 *           -> TRAVEL_TO_TIPPED_SHOOT -> SPEED_UP_SECOND_SHOOT -> FEED_SECOND_SHOOT
 *           -> TRAVEL_TO_FLOWER_INTERMEDIATE -> TRAVEL_TO_FLOWER -> COLLECT_FLOWER -> TRAVEL_FROM_FLOWER_TO_INTERMEDIATE
 *           -> TRAVEL_TO_SHOOT_2 -> SPEED_UP_SHOOT_2 -> FEED_SHOOT_2
 *           -> TRAVEL_TO_FLOWER_2_INTERMEDIATE -> TRAVEL_TO_FLOWER_2 -> COLLECT_FLOWER_2 -> TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE
 *           -> TRAVEL_TO_SHOOT_3 -> SPEED_UP_SHOOT_3 -> FEED_SHOOT_3
 *           -> TRAVEL_TO_PARK -> DONE
 *           (not tipped: TRAVEL_TO_SHOOT -> TEMP_FAILED -> DONE)
 */
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();

    /** @return true if this OpMode runs on the blue alliance, false for red; selects the Limelight pipeline. **/
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Vision vision;
    private Follower follower;
    private Timer stateTimer, loopTimer, telemetryTimer;
    private TelemetryManager telemetryM;
    private Pose startPose, shootPose;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    /** true once the flywheel is within {@link Tunables#flywheelRPMMargin} of a nonzero target RPM **/
    private boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= Tunables.flywheelRPMMargin;
    }

    private enum State {
        TRAVEL_TO_FIRST_SHOOT, // driving linearly from the start pose to the shoot pose; flywheel already spinning up
        SPIN_UP_FIRST_SHOOT,   // at the shoot pose; waiting for flywheel to reach target RPM
        FEED_FIRST_SHOOT,      // Robot launch feeds the preloaded balls through the flywheel
        TRAVEL_TO_INTERMEDIATE, // driving linearly to the intermediate pose with intake running; flywheel keeps spinning
        TRAVEL_TO_REFUEL,      // driving linearly from the intermediate pose to the refuel pose with intake running
        REFUEL,                // sitting on the refuel pose with intake running to pick up balls
        TRAVEL_TO_SHOOT,       // driving back to the shoot pose; intake running, flywheel keeps spinning

        // hive tipped, go to the scoring side and shoot the second round there
        TRAVEL_TO_TIPPED_SHOOT, // go to scoring side to shoot second round
        SPEED_UP_SECOND_SHOOT, // arrived, wait for flywheel to speed up
        FEED_SECOND_SHOOT, // Robot launch sequence...

        // first flower: line to the intermediate pose, slow line into the flower, collect, back out to the intermediate pose, line to the tipped shoot pose, shoot
        TRAVEL_TO_FLOWER_INTERMEDIATE, // driving a full-speed line from the tipped shoot pose to the flower's intermediate pose; intake running, flywheel keeps spinning
        TRAVEL_TO_FLOWER,      // driving a straight line at reduced speed from the intermediate pose into the flower
        COLLECT_FLOWER,        // sitting on the flower with intake running to pick up balls
        TRAVEL_FROM_FLOWER_TO_INTERMEDIATE, // backing out of the flower to its intermediate pose
        TRAVEL_TO_SHOOT_2,     // driving a straight line from the intermediate pose back to the tipped shoot pose
        SPEED_UP_SHOOT_2,      // arrived, wait for flywheel to speed up
        FEED_SHOOT_2,          // Robot launch sequence...

        // second flower: curve to the intermediate pose, otherwise the same shape as the first
        TRAVEL_TO_FLOWER_2_INTERMEDIATE, // driving a full-speed curve (1 control point) from the tipped shoot pose to the second flower's intermediate pose
        TRAVEL_TO_FLOWER_2,    // driving a straight line at reduced speed from the intermediate pose into the second flower
        COLLECT_FLOWER_2,      // sitting on the second flower with intake running to pick up balls
        TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE, // backing out of the second flower to its intermediate pose
        TRAVEL_TO_SHOOT_3,     // driving a straight line from the intermediate pose to the ORIGINAL shoot pose (not the tipped one)
        SPEED_UP_SHOOT_3,      // arrived, wait for flywheel to speed up
        FEED_SHOOT_3,          // Robot launch sequence...

        TRAVEL_TO_PARK,        // everything off; driving a curve (1 control point) from the original shoot pose to the park pose
        DONE,                  // parked; waiting for the OpMode to be stopped

        TEMP_FAILED            // TEMPORARY: hive did not flip; report failure, then DONE
    }

    private State state = State.TRAVEL_TO_FIRST_SHOOT;

    /** startPose -> shootPose, then shootPose -> intermediatePose -> refuelPose and back. Built in {@link #init()}. **/
    private Path startToShootPath, shootToIntermediatePath, intermediateToRefuelPath, refuelToShootPath;

    /** shootPose -> scoring side (bent by {@link AutoConfig#tippedControlPose1} and {@link AutoConfig#tippedControlPose2}). Built in {@link #init()}. **/
    private Path shootToTippedShootPath;

    /**
     * tippedShootPose -> flowerIntermediatePose (line, full speed) -> flowerPose (line, {@link #FLOWER_APPROACH_SPEED})
     * -> flowerIntermediatePose (line) -> tippedShootPose (line), then tippedShootPose -> flower2IntermediatePose
     * (curve, full speed) -> flower2Pose (line, {@link #FLOWER_APPROACH_SPEED}) -> flower2IntermediatePose (line)
     * -> shootPose (line, the ORIGINAL shoot pose), then shootPose -> parkPose (curve). Built in {@link #init()}.
     **/
    private Path tippedShootToFlowerIntermediatePath, flowerIntermediateToFlowerPath,
            flowerToFlowerIntermediatePath, flowerIntermediateToTippedShootPath,
            tippedShootToFlower2IntermediatePath, flower2IntermediateToFlower2Path,
            flower2ToFlower2IntermediatePath, flower2IntermediateToShootPath,
            shootToParkPath;

    private static final double REFUEL_ROTATE_START = 0.7;

    /** fraction of max path speed for the final straight run from a flower's intermediate pose into the flower **/
    private static final double FLOWER_APPROACH_SPEED = 0.2;

    private void buildPaths() {
        startToShootPath = line(startPose, shootPose).linear(startPose, shootPose);
        shootToIntermediatePath = line(shootPose, config.intermediatePose).linear(shootPose, config.intermediatePose);
        intermediateToRefuelPath = line(config.intermediatePose, config.refuelPose)
                .heading(Interpolator.piecewise()
                        .until(REFUEL_ROTATE_START, Interpolator.constant(config.intermediatePose))
                        .until(1.0, Interpolator.linear(config.intermediatePose, config.refuelPose)))
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.35)
                );
        refuelToShootPath = line(config.refuelPose, shootPose).linear(config.refuelPose, shootPose);
        // curve(start, control, end): the control point is not driven through, it only pulls the path toward it.
        // Toward the scoring side (bent by the two control poses), then a line to park.
        shootToTippedShootPath = curve(shootPose, config.tippedControlPose1, config.tippedControlPose2, config.tippedShootPose)
                .linear(shootPose, config.tippedShootPose);
        // After the tipped shoot, per flower: full-speed approach to an intermediate pose (flower 1 a line, flower 2 a
        // curve with 1 control point), then a slow straight line into the flower. It backs out to the same intermediate
        // pose, then drives a line to the tipped shoot pose.
        tippedShootToFlowerIntermediatePath = line(config.tippedShootPose, config.flowerIntermediatePose)
                .linear(config.tippedShootPose, config.flowerIntermediatePose);
        flowerIntermediateToFlowerPath = line(config.flowerIntermediatePose, config.flowerPose)
                .linear(config.flowerIntermediatePose, config.flowerPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED));
        flowerToFlowerIntermediatePath = line(config.flowerPose, config.flowerIntermediatePose)
                .linear(config.flowerPose, config.flowerIntermediatePose);
        flowerIntermediateToTippedShootPath = line(config.flowerIntermediatePose, config.tippedShootPose)
                .linear(config.flowerIntermediatePose, config.tippedShootPose);
        tippedShootToFlower2IntermediatePath = curve(config.tippedShootPose, config.flower2ControlPose, config.flower2IntermediatePose)
                .linear(config.tippedShootPose, config.flower2IntermediatePose);
        flower2IntermediateToFlower2Path = line(config.flower2IntermediatePose, config.flower2Pose)
                .linear(config.flower2IntermediatePose, config.flower2Pose)
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED));
        flower2ToFlower2IntermediatePath = line(config.flower2Pose, config.flower2IntermediatePose)
                .linear(config.flower2Pose, config.flower2IntermediatePose);
        flower2IntermediateToShootPath = line(config.flower2IntermediatePose, shootPose)
                .linear(config.flower2IntermediatePose, shootPose);
        shootToParkPath = curve(shootPose, config.parkControlPose, config.parkPose)
                .linear(shootPose, config.parkPose);
    }

    private void autoUpdate() {
        switch (state) {
            case TRAVEL_TO_FIRST_SHOOT:
                if (!follower.isBusy()) {
                    setState(State.SPIN_UP_FIRST_SHOOT);
                }
                break;
            case SPIN_UP_FIRST_SHOOT:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    robot.startLaunch();
                    setState(State.FEED_FIRST_SHOOT);
                }
                break;
            case FEED_FIRST_SHOOT:
                if (!robot.isLaunching()) {
                    follower.follow(shootToIntermediatePath);
                    setState(State.TRAVEL_TO_INTERMEDIATE);
                }
                break;
            case TRAVEL_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    follower.follow(intermediateToRefuelPath);
                    setState(State.TRAVEL_TO_REFUEL);
                }
                break;
            case TRAVEL_TO_REFUEL:
                if (!follower.isBusy()
                        // make sure it times out after getting stuck on a ball
                        || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime) {
                    setState(State.REFUEL);
                }
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.forward();
                    follower.follow(refuelToShootPath);
                    setState(State.TRAVEL_TO_SHOOT);
                }
                break;
            case TRAVEL_TO_SHOOT:
                if (!follower.isBusy()) {
                    if (Vision.limelight.getLatestResult().getBotposeTagCount() > Tunables.tagThreshold) {
                        // hive did not flip; that case no longer exists, so this is a failure
                        shutOff();
                        setState(State.TEMP_FAILED);
                    } else {
                        follower.follow(shootToTippedShootPath);
                        setState(State.TRAVEL_TO_TIPPED_SHOOT);
                    }
                }
                break;

            // hive tipped, curve to the scoring side and shoot there
            case TRAVEL_TO_TIPPED_SHOOT:
                if (!follower.isBusy()) {
                    setState(State.SPEED_UP_SECOND_SHOOT);
                }
                break;
            case SPEED_UP_SECOND_SHOOT:
                if (isFlywheelWithinMargin()) {
                    robot.startLaunch();
                    setState(State.FEED_SECOND_SHOOT);
                }
                break;
            case FEED_SECOND_SHOOT:
                if (!robot.isLaunching()) {
                    // keep the flywheel spinning: we shoot again after the flower
                    robot.intake.forward();
                    follower.follow(tippedShootToFlowerIntermediatePath);
                    setState(State.TRAVEL_TO_FLOWER_INTERMEDIATE);
                }
                break;

            // collect at the first flower, then shoot again from the same tipped shoot pose
            case TRAVEL_TO_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    follower.follow(flowerIntermediateToFlowerPath);
                    setState(State.TRAVEL_TO_FLOWER);
                }
                break;
            case TRAVEL_TO_FLOWER:
                if (!follower.isBusy()
                        // make sure it times out after getting stuck on a ball
                        || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime) {
                    setState(State.COLLECT_FLOWER);
                }
                break;
            case COLLECT_FLOWER:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.flower.goUp(); // raise as soon as we leave the flower
                    follower.follow(flowerToFlowerIntermediatePath);
                    setState(State.TRAVEL_FROM_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case TRAVEL_FROM_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    follower.follow(flowerIntermediateToTippedShootPath);
                    setState(State.TRAVEL_TO_SHOOT_2);
                }
                break;
            case TRAVEL_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    setState(State.SPEED_UP_SHOOT_2);
                }
                break;
            case SPEED_UP_SHOOT_2:
                if (isFlywheelWithinMargin()) {
                    robot.startLaunch();
                    setState(State.FEED_SHOOT_2);
                }
                break;
            case FEED_SHOOT_2:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    follower.follow(tippedShootToFlower2IntermediatePath);
                    setState(State.TRAVEL_TO_FLOWER_2_INTERMEDIATE);
                }
                break;

            // collect at the second flower, then return to the original shoot pose for the third shot
            case TRAVEL_TO_FLOWER_2_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    follower.follow(flower2IntermediateToFlower2Path);
                    setState(State.TRAVEL_TO_FLOWER_2);
                }
                break;
            case TRAVEL_TO_FLOWER_2:
                if (!follower.isBusy()
                        || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime) {
                    setState(State.COLLECT_FLOWER_2);
                }
                break;
            case COLLECT_FLOWER_2:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.flower.goUp(); // raise as soon as we leave the flower
                    follower.follow(flower2ToFlower2IntermediatePath);
                    setState(State.TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE);
                }
                break;
            case TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    follower.follow(flower2IntermediateToShootPath);
                    setState(State.TRAVEL_TO_SHOOT_3);
                }
                break;
            case TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) {
                    setState(State.SPEED_UP_SHOOT_3);
                }
                break;
            case SPEED_UP_SHOOT_3:
                if (isFlywheelWithinMargin()) {
                    robot.startLaunch();
                    setState(State.FEED_SHOOT_3);
                }
                break;
            case FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    shutOff(); // last shot; everything off for park
                    follower.follow(shootToParkPath);
                    setState(State.TRAVEL_TO_PARK);
                }
                break;

            case TRAVEL_TO_PARK:
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;
            case DONE:
                requestOpModeStop();
                break;
            case TEMP_FAILED:
                telemetryM.debug("FAILED: im a failure");
                setState(State.DONE);
                break;
        }
    }

    private void shutOff() {
        robot.transfer.close();
        robot.intake.off();
        targetRPM = 0;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        vision.update();
        follower.update();
        robot.flywheel.update(targetRPM); // re-run PIDF every loop so RPM actually converges on target also dih
        robot.update(); // advances the shooting state machine and intake power-save
        autoUpdate();
        if (Tunables.isDebugging) {
            sendTelemetry(false);
        }
        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }

    @Override
    public void init() {
        config = buildConfig();
        loopTimer = new Timer();
        stateTimer = new Timer();
        telemetryTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap);
        vision = new Vision(hardwareMap, isBlueTeam());
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        startPose = config.startPose;
        shootPose = config.shootPose;
        follower.setPose(startPose);
        buildPaths();
        sendTelemetry(true);
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        sendTelemetry(true);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        targetRPM = Tunables.shootRPM; // begin spinning up immediately
        telemetryTimer.reset();
        loopTimer.reset(); // so the first loop() doesn't report init time as loop time
        follower.follow(startToShootPath);
        setState(State.TRAVEL_TO_FIRST_SHOOT);
    }

    @Override
    public void stop() {
        if (robot != null) {
            shutOff();
            robot.flywheel.update(0);
        }
        updateHandoff();
    }
    public void updateHandoff() {
        if (follower != null) {
            HandoffState.pose = follower.pose();
        }
    }

    public void sendTelemetry(boolean sendInitTime) {
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("Init time (millis): " + loopTimer.get(TimeUnit.SECONDS));
        } else {
            if (targetRPM > 0 && !isFlywheelWithinMargin()) telemetryM.debug("WARNING: FLYWHEEL OUT OF MARGIN");
        }
        Pose pose = follower.pose(); // read once; each call may allocate
        telemetryM.debug("State: " + state);
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", targetRPM);
        telemetryM.addData("x", pose.x());
        telemetryM.addData("y", pose.y());
        telemetryM.addData("Heading", pose.heading());
        telemetryM.addData("tags detected", vision.getLatestResult().getBotposeTagCount());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

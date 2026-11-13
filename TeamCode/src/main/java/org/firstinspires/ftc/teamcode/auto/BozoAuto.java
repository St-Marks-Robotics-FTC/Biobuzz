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
 * Autonomous. Poses are documented in {@link AutoConfig}; the state machine is {@link State}.
 *
 * <pre>
 * SHARED   start -> shoot -> intermediate -> refuel -> shoot -> (look at the hive)
 *
 * There are only two physical flowers: the scoring flower (scoring side) and the audience flower (audience side).
 *
 * hive TIPPED  -> CASE1: tippedShoot, scoring flower, tippedShoot, audience flower, checkpoint (every "shoot" fires the flywheel)
 *     made all 8      -> CASE1A: original shoot pose, shoot, park
 *     missed some     -> CASE1B: tippedShoot, shoot, park
 *
 * hive NOT tipped -> CASE2: shoot again, audience flower, checkpoint
 *     tipped now      -> CASE2A: tippedShoot, shoot, scoring flower, tippedShoot, shoot, park
 *     still not       -> CASE2B: original shoot pose, shoot, scoring flower, park
 * </pre>
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

    /**
     * Naming: {@code TRAVEL_*} = driving a path; {@code SPEED_UP_*} = waiting for the flywheel; {@code FEED_*} =
     * launching; {@code COLLECT_*} = sitting on a flower or refuel pose with the intake running.
     * "_2"/"_3"/"_4" count the shots (1 = preloaded, 2 = after refuel, ...). {@code SCORING_FLOWER} and
     * {@code AUDIENCE_FLOWER} name the physical flower.
     */
    private enum State {
        // ================================================================ SHARED
        TRAVEL_TO_FIRST_SHOOT, // driving linearly from the start pose to the shoot pose; flywheel already spinning up
        SPIN_UP_FIRST_SHOOT,   // at the shoot pose; waiting for flywheel to reach target RPM
        FEED_FIRST_SHOOT,      // Robot launch feeds the preloaded balls through the flywheel
        TRAVEL_TO_INTERMEDIATE, // driving linearly to the intermediate pose with intake running; flywheel keeps spinning
        TRAVEL_TO_REFUEL,      // driving linearly from the intermediate pose to the refuel pose with intake running
        REFUEL,                // sitting on the refuel pose with intake running to pick up balls
        TRAVEL_TO_SHOOT,       // driving back to the shoot pose; intake running. On arrival the hive is judged: tipped -> CASE1, not -> CASE2

        // ================================================================ CASE1: HIVE TIPPED FIRST TRY
        CASE1_TRAVEL_TO_SHOOT_2,   // curve from the shoot pose to the scoring-side (tipped) shoot pose
        CASE1_SPEED_UP_SHOOT_2,    // arrived, wait for flywheel to speed up
        CASE1_FEED_SHOOT_2,        // Robot launch sequence...

        // scoring flower: line to the intermediate pose, slow line into the flower, collect, back out, line to the tipped shoot pose, shoot
        CASE1_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE,      // full-speed line from the tipped shoot pose to the scoring flower's intermediate pose
        CASE1_TRAVEL_TO_SCORING_FLOWER,                   // reduced-speed line from the intermediate pose into the scoring flower
        CASE1_COLLECT_SCORING_FLOWER,                     // sitting on the scoring flower with intake running
        CASE1_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE, // backing out of the scoring flower to its intermediate pose
        CASE1_TRAVEL_TO_SHOOT_3,                          // line from the intermediate pose back to the tipped shoot pose
        CASE1_SPEED_UP_SHOOT_3,
        CASE1_FEED_SHOOT_3,

        // audience flower: curve to the intermediate pose, otherwise the same shape as the scoring flower; then on to the checkpoint
        CASE1_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE,      // full-speed curve (1 control point) from the tipped shoot pose
        CASE1_TRAVEL_TO_AUDIENCE_FLOWER,                   // reduced-speed line into the audience flower
        CASE1_COLLECT_AUDIENCE_FLOWER,
        CASE1_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE, // backing out of the audience flower
        CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT,      // line to a spot where the Limelight can judge the hive; decides 1A vs 1B

        // ---- CASE1A: we made all 8 and the hive flipped a second time -> shoot from the ORIGINAL shoot pose and park
        CASE1A_TRAVEL_TO_SHOOT_4,  // line from the checkpoint to the original shoot pose
        CASE1A_SPEED_UP_SHOOT_4,
        CASE1A_FEED_SHOOT_4,
        CASE1A_TRAVEL_TO_PARK,     // everything off; curve (1 control point) to the park pose

        // ---- CASE1B: we did not make all 8 -> go back to the SCORING side and shoot again
        CASE1B_TRAVEL_TO_SHOOT_4,  // curve (1 control point) from the checkpoint to the tipped shoot pose
        CASE1B_SPEED_UP_SHOOT_4,
        CASE1B_FEED_SHOOT_4,
        CASE1B_TRAVEL_TO_PARK,     // everything off; curve (1 control point) from the tipped shoot pose to the park pose

        // ================================================================ CASE2: HIVE DID NOT TIP FIRST TRY
        // we are still at the original shoot pose, so there is no travel before the second shot
        CASE2_SPEED_UP_SHOOT_2,
        CASE2_FEED_SHOOT_2,

        // audience flower, then the checkpoint
        CASE2_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE,      // full-speed line from the shoot pose to the audience flower's intermediate pose
        CASE2_TRAVEL_TO_AUDIENCE_FLOWER,
        CASE2_COLLECT_AUDIENCE_FLOWER,
        CASE2_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE,
        CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT,      // line to the audience-side checkpoint; decides 2A vs 2B

        // ---- CASE2A: hive finally tipped -> shoot on the SCORING side, collect the scoring flower there, shoot, park
        CASE2A_TRAVEL_TO_SHOOT_3,   // line from the checkpoint to the tipped shoot pose
        CASE2A_SPEED_UP_SHOOT_3,
        CASE2A_FEED_SHOOT_3,
        CASE2A_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE,      // full-speed line from the tipped shoot pose
        CASE2A_TRAVEL_TO_SCORING_FLOWER,
        CASE2A_COLLECT_SCORING_FLOWER,
        CASE2A_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE,
        CASE2A_TRAVEL_TO_SHOOT_4,   // line from the scoring flower's intermediate pose to the tipped shoot pose
        CASE2A_SPEED_UP_SHOOT_4,
        CASE2A_FEED_SHOOT_4,
        CASE2A_TRAVEL_TO_PARK,      // everything off; curve from the tipped shoot pose to the park pose

        // ---- CASE2B: hive still not tipped -> shoot AUDIENCE side, collect the scoring flower, park
        CASE2B_TRAVEL_TO_SHOOT_3,   // line from the checkpoint to the original shoot pose
        CASE2B_SPEED_UP_SHOOT_3,
        CASE2B_FEED_SHOOT_3,
        CASE2B_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE,      // full-speed cubic curve (2 control points) from the shoot pose
        CASE2B_TRAVEL_TO_SCORING_FLOWER,
        CASE2B_COLLECT_SCORING_FLOWER,
        CASE2B_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE,
        CASE2B_TRAVEL_TO_PARK,      // everything off; curve (1 control point) from the intermediate pose to the park pose

        DONE,                  // parked; waiting for the OpMode to be stopped
    }

    private State state = State.TRAVEL_TO_FIRST_SHOOT;

    // ======================================================================== PATHS
    // Naming: <from>To<to>Path. "Intermediate" is the waypoint in front of a flower; "tippedShoot" is the scoring-side
    // shoot pose; "shoot" is the original (audience-side) shoot pose. There are only two physical flowers: the
    // "scoringFlower" (scoring side) and the "audienceFlower" (audience side). All paths are built once in buildPaths().

    // ---- SHARED: startPose -> shoot -> intermediate -> refuel -> shoot
    private Path startToShootPath, shootToIntermediatePath, intermediateToRefuelPath, refuelToShootPath;

    // ---- CASE1: hive tipped. shoot -> tippedShoot, scoring flower round trip, audience flower round trip, checkpoint
    private Path shootToTippedShootPath;
    private Path tippedShootToScoringFlowerIntermediatePath, scoringFlowerIntermediateToScoringFlowerPath,
            scoringFlowerToScoringFlowerIntermediatePath, scoringFlowerIntermediateToTippedShootPath;
    private Path tippedShootToAudienceFlowerIntermediatePath, audienceFlowerIntermediateToAudienceFlowerPath,
            audienceFlowerToAudienceFlowerIntermediatePath, audienceFlowerIntermediateToAudienceCheckpointPath;
    // CASE1A: made all 8 -> original shoot pose (audienceCheckpointToShootPath, shared with CASE2B) -> park
    private Path shootToParkPath;
    // CASE1B: missed some -> tippedShoot (audienceCheckpointToTippedShootPath, shared with CASE2A) -> park (park path also used by CASE2A)
    private Path tippedShootToParkPath;

    // ---- CASE2: hive not tipped. Shoot again from where we are, audience flower round trip, then to the checkpoint
    private Path shootToAudienceFlowerIntermediatePath;
    // (the audience flower creep in/out paths and the line to the checkpoint are CASE1's audienceFlowerIntermediateTo... paths)
    // CASE2A / CASE1B: tipped -> tippedShoot via a cubic curve -> scoring flower round trip (CASE1's scoring flower paths) -> tippedShoot -> park
    private Path audienceCheckpointToTippedShootPath;
    // CASE2B / CASE1A: not tipped -> original shoot pose -> scoring flower round trip -> park
    private Path audienceCheckpointToShootPath, shootToScoringFlowerIntermediatePath, scoringFlowerIntermediateToParkPath;

    private static final double REFUEL_ROTATE_START = 0.7;

    /** fraction of max path speed for the final straight run from a flower's intermediate pose into the flower **/
    private static final double FLOWER_APPROACH_SPEED = 0.2;

    /** a straight line {@code from} -> {@code to}, heading turning linearly between them **/
    private static Path plainLine(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    /** a straight line {@code from} -> {@code to} at {@link #FLOWER_APPROACH_SPEED}, for creeping into a flower **/
    private static Path creepLine(Pose from, Pose to) {
        return plainLine(from, to).with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED));
    }

    /**
     * Builds every path. curve(start, control..., end): the control points are not driven through, they only pull the
     * path toward them. Everything is built up front so a bad pose fails at init, not mid-match.
     */
    private void buildPaths() {
        // ---------------------------------------------------------------- SHARED
        startToShootPath = plainLine(startPose, shootPose);
        shootToIntermediatePath = plainLine(shootPose, config.intermediatePose);
        intermediateToRefuelPath = line(config.intermediatePose, config.refuelPose)
                .heading(Interpolator.piecewise()
                        .until(REFUEL_ROTATE_START, Interpolator.constant(config.intermediatePose))
                        .until(1.0, Interpolator.linear(config.intermediatePose, config.refuelPose)))
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.35)
                );
        refuelToShootPath = plainLine(config.refuelPose, shootPose);

        // ---------------------------------------------------------------- CASE1: hive tipped
        // shoot -> scoring side (cubic curve bent by the two tipped control poses)
        shootToTippedShootPath = curve(shootPose, config.tippedControlPose1, config.tippedControlPose2, config.tippedShootPose)
                .linear(shootPose, config.tippedShootPose);
        // scoring flower: full-speed line out, creep in, line back out, line to tippedShoot
        tippedShootToScoringFlowerIntermediatePath = plainLine(config.tippedShootPose, config.scoringFlowerIntermediatePose);
        scoringFlowerIntermediateToScoringFlowerPath = creepLine(config.scoringFlowerIntermediatePose, config.scoringFlowerPose);
        scoringFlowerToScoringFlowerIntermediatePath = plainLine(config.scoringFlowerPose, config.scoringFlowerIntermediatePose)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.35)
                );;
        scoringFlowerIntermediateToTippedShootPath = plainLine(config.scoringFlowerIntermediatePose, config.tippedShootPose);
        // audience flower: like the scoring flower but the full-speed approach is a curve (1 control point); then on to the checkpoint
        tippedShootToAudienceFlowerIntermediatePath = curve(config.tippedShootPose, config.tippedShootToAudienceFlowerControlPose, config.audienceFlowerIntermediatePose)
                .linear(config.tippedShootPose, config.audienceFlowerIntermediatePose);
        audienceFlowerIntermediateToAudienceFlowerPath = creepLine(config.audienceFlowerIntermediatePose, config.audienceFlowerPose);
        audienceFlowerToAudienceFlowerIntermediatePath = plainLine(config.audienceFlowerPose, config.audienceFlowerIntermediatePose)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.35)
                );;
        audienceFlowerIntermediateToAudienceCheckpointPath = plainLine(config.audienceFlowerIntermediatePose, config.audienceCheckpointPose);
        // CASE1A: audienceCheckpoint -> ORIGINAL shoot pose (plain line, shared with CASE2B) -> park (curve, 1 control point)
        shootToParkPath = curve(shootPose, config.parkControlPoseFromShoot, config.parkPose)
                .linear(shootPose, config.parkPose);
        // CASE1B: audienceCheckpoint -> tippedShoot (cubic curve, 2 control points, shared with CASE2A) -> park (curve, 1 control point)
        tippedShootToParkPath = curve(config.tippedShootPose, config.parkControlPoseFromTippedShoot, config.parkPose)
                .linear(config.tippedShootPose, config.parkPose);

        // ---------------------------------------------------------------- CASE2: hive not tipped
        // audience flower: full-speed line out, creep in and back out (CASE1's audience flower paths), line to the checkpoint
        shootToAudienceFlowerIntermediatePath = plainLine(shootPose, config.audienceFlowerIntermediatePose);
        // CASE2A: audienceCheckpoint -> tippedShoot (cubic curve, 2 control points, shared with CASE1B); the scoring flower round trip and the park reuse CASE1's paths
        audienceCheckpointToTippedShootPath = curve(config.audienceCheckpointPose, config.audienceCheckpointToTippedShootControlPose1, config.audienceCheckpointToTippedShootControlPose2, config.tippedShootPose)
                .linear(config.audienceCheckpointPose, config.tippedShootPose);
        // CASE2B: audienceCheckpoint -> ORIGINAL shoot pose (shared with CASE1A) -> scoring flower (cubic curve, 2 control points) -> park (curve, 1 control point)
        audienceCheckpointToShootPath = plainLine(config.audienceCheckpointPose, shootPose);
        shootToScoringFlowerIntermediatePath = curve(shootPose, config.shootToScoringFlowerControlPose1, config.shootToScoringFlowerControlPose2, config.scoringFlowerIntermediatePose)
                .linear(shootPose, config.scoringFlowerIntermediatePose);
        scoringFlowerIntermediateToParkPath = curve(config.scoringFlowerIntermediatePose, config.parkControlPoseFromScoringFlower, config.parkPose)
                .linear(config.scoringFlowerIntermediatePose, config.parkPose);
    }

    /** @return true once a path has finished, or {@link Tunables#maxTravelToRefuelTime} passed (stuck on a ball). **/
    private boolean finishedCreepingIntoFlower() {
        return !follower.isBusy() || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime;
    }

    /** @return true once we have sat on a flower with the intake running for {@link Tunables#flowerRefuelDurationMillis}. **/
    private boolean finishedCollecting() {
        return stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.flowerRefuelDurationMillis;
    }

    /** Starts driving {@code path} and switches to {@code next}; the one place a state change starts a path. **/
    private void driveThen(Path path, State next) {
        follower.follow(path);
        setState(next);
    }

    /** Once at speed, launches and switches to {@code feeding}. **/
    private void launchWhenAtSpeed(State feeding) {
        if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
            robot.startLaunch();
            setState(feeding);
        }
    }

    private void autoUpdate() {
        switch (state) {
            // ============================================================ SHARED
            case TRAVEL_TO_FIRST_SHOOT:
                if (!follower.isBusy()) setState(State.SPIN_UP_FIRST_SHOOT);
                break;
            case SPIN_UP_FIRST_SHOOT:
                launchWhenAtSpeed(State.FEED_FIRST_SHOOT);
                break;
            case FEED_FIRST_SHOOT:
                if (!robot.isLaunching()) driveThen(shootToIntermediatePath, State.TRAVEL_TO_INTERMEDIATE);
                break;
            case TRAVEL_TO_INTERMEDIATE:
                if (!follower.isBusy()) driveThen(intermediateToRefuelPath, State.TRAVEL_TO_REFUEL);
                break;
            case TRAVEL_TO_REFUEL:
                if (finishedCreepingIntoFlower()) setState(State.REFUEL);
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.forward();
                    driveThen(refuelToShootPath, State.TRAVEL_TO_SHOOT);
                }
                break;
            case TRAVEL_TO_SHOOT:
                if (!follower.isBusy()) {
                    // Limelight sees the tags => the hive did NOT flip => CASE2; otherwise it tipped => CASE1
                    if (Vision.limelight.getLatestResult().getBotposeTagCount() > Tunables.tagThreshold) {
                        setState(State.CASE2_SPEED_UP_SHOOT_2); // already at the shoot pose, no travel
                    } else {
                        driveThen(shootToTippedShootPath, State.CASE1_TRAVEL_TO_SHOOT_2);
                    }
                }
                break;

            // ============================================================ CASE1: hive tipped first try
            case CASE1_TRAVEL_TO_SHOOT_2:
                if (!follower.isBusy()) setState(State.CASE1_SPEED_UP_SHOOT_2);
                break;
            case CASE1_SPEED_UP_SHOOT_2:
                launchWhenAtSpeed(State.CASE1_FEED_SHOOT_2);
                break;
            case CASE1_FEED_SHOOT_2:
                if (!robot.isLaunching()) {
                    robot.intake.forward(); // flywheel keeps spinning: we shoot again after the scoring flower
                    driveThen(tippedShootToScoringFlowerIntermediatePath, State.CASE1_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE);
                }
                break;

            // scoring flower, then shoot again from the same tipped shoot pose
            case CASE1_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerIntermediateToScoringFlowerPath, State.CASE1_TRAVEL_TO_SCORING_FLOWER);
                }
                break;
            case CASE1_TRAVEL_TO_SCORING_FLOWER:
                if (finishedCreepingIntoFlower()) {
                    robot.flower.startShake();
                    setState(State.CASE1_COLLECT_SCORING_FLOWER);
                }
                break;
            case CASE1_COLLECT_SCORING_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerToScoringFlowerIntermediatePath, State.CASE1_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE1_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goUp();
                    driveThen(scoringFlowerIntermediateToTippedShootPath, State.CASE1_TRAVEL_TO_SHOOT_3);
                }
                break;
            case CASE1_TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) setState(State.CASE1_SPEED_UP_SHOOT_3);
                break;
            case CASE1_SPEED_UP_SHOOT_3:
                launchWhenAtSpeed(State.CASE1_FEED_SHOOT_3);
                break;
            case CASE1_FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(tippedShootToAudienceFlowerIntermediatePath, State.CASE1_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE);
                }
                break;

            // audience flower, then the checkpoint
            case CASE1_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(audienceFlowerIntermediateToAudienceFlowerPath, State.CASE1_TRAVEL_TO_AUDIENCE_FLOWER);
                }
                break;
            case CASE1_TRAVEL_TO_AUDIENCE_FLOWER:
                if (finishedCreepingIntoFlower()) {
                    robot.flower.startShake();
                    setState(State.CASE1_COLLECT_AUDIENCE_FLOWER);
                }
                break;
            case CASE1_COLLECT_AUDIENCE_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goDown();
                    driveThen(audienceFlowerToAudienceFlowerIntermediatePath, State.CASE1_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE1_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goUp();
                    driveThen(audienceFlowerIntermediateToAudienceCheckpointPath, State.CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT);
                }
                break;
            case CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT:
                if (!follower.isBusy()) {
                    if (vision.getLastHiveState() == Vision.HiveState.AUDIENCE_UP) driveThen(audienceCheckpointToShootPath, State.CASE1A_TRAVEL_TO_SHOOT_4);
                    else driveThen(audienceCheckpointToTippedShootPath, State.CASE1B_TRAVEL_TO_SHOOT_4);
                }
                break;

            // ---- CASE1A: made all 8 -> original shoot pose, shoot, park
            case CASE1A_TRAVEL_TO_SHOOT_4:
                if (!follower.isBusy()) setState(State.CASE1A_SPEED_UP_SHOOT_4);
                break;
            case CASE1A_SPEED_UP_SHOOT_4:
                launchWhenAtSpeed(State.CASE1A_FEED_SHOOT_4);
                break;
            case CASE1A_FEED_SHOOT_4:
                if (!robot.isLaunching()) {
                    shutOff(); // last shot; everything off for park
                    driveThen(shootToParkPath, State.CASE1A_TRAVEL_TO_PARK);
                }
                break;
            case CASE1A_TRAVEL_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ---- CASE1B: missed some -> back to the scoring side, shoot, park
            case CASE1B_TRAVEL_TO_SHOOT_4:
                if (!follower.isBusy()) setState(State.CASE1B_SPEED_UP_SHOOT_4);
                break;
            case CASE1B_SPEED_UP_SHOOT_4:
                launchWhenAtSpeed(State.CASE1B_FEED_SHOOT_4);
                break;
            case CASE1B_FEED_SHOOT_4:
                if (!robot.isLaunching()) {
                    shutOff();
                    driveThen(tippedShootToParkPath, State.CASE1B_TRAVEL_TO_PARK);
                }
                break;
            case CASE1B_TRAVEL_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ============================================================ CASE2: hive did not tip first try
            case CASE2_SPEED_UP_SHOOT_2:
                launchWhenAtSpeed(State.CASE2_FEED_SHOOT_2);
                break;
            case CASE2_FEED_SHOOT_2:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(shootToAudienceFlowerIntermediatePath, State.CASE2_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE);
                }
                break;

            // audience flower, then the checkpoint
            case CASE2_TRAVEL_TO_AUDIENCE_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(audienceFlowerIntermediateToAudienceFlowerPath, State.CASE2_TRAVEL_TO_AUDIENCE_FLOWER);
                }
                break;
            case CASE2_TRAVEL_TO_AUDIENCE_FLOWER:
                if (finishedCreepingIntoFlower()) {
                    robot.flower.startShake();
                    setState(State.CASE2_COLLECT_AUDIENCE_FLOWER);
                }
                break;
            case CASE2_COLLECT_AUDIENCE_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goDown();
                    driveThen(audienceFlowerToAudienceFlowerIntermediatePath, State.CASE2_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE2_TRAVEL_FROM_AUDIENCE_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goUp();
                    driveThen(audienceFlowerIntermediateToAudienceCheckpointPath, State.CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT);
                }
                break;
            case CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT:
                if (!follower.isBusy()) {
                    if (vision.getLastHiveState() == Vision.HiveState.SCORING_UP) driveThen(audienceCheckpointToTippedShootPath, State.CASE2A_TRAVEL_TO_SHOOT_3);
                    else driveThen(audienceCheckpointToShootPath, State.CASE2B_TRAVEL_TO_SHOOT_3);
                }
                break;

            // ---- CASE2A: hive finally tipped -> scoring side: shoot, scoring flower, shoot, park
            case CASE2A_TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) setState(State.CASE2A_SPEED_UP_SHOOT_3);
                break;
            case CASE2A_SPEED_UP_SHOOT_3:
                launchWhenAtSpeed(State.CASE2A_FEED_SHOOT_3);
                break;
            case CASE2A_FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(tippedShootToScoringFlowerIntermediatePath, State.CASE2A_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE);
                }
                break;
            case CASE2A_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerIntermediateToScoringFlowerPath, State.CASE2A_TRAVEL_TO_SCORING_FLOWER);
                }
                break;
            case CASE2A_TRAVEL_TO_SCORING_FLOWER:
                if (finishedCreepingIntoFlower()) {
                    robot.flower.startShake();
                    setState(State.CASE2A_COLLECT_SCORING_FLOWER);
                }
                break;
            case CASE2A_COLLECT_SCORING_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerToScoringFlowerIntermediatePath, State.CASE2A_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE2A_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goUp();
                    driveThen(scoringFlowerIntermediateToTippedShootPath, State.CASE2A_TRAVEL_TO_SHOOT_4);
                }
                break;
            case CASE2A_TRAVEL_TO_SHOOT_4:
                if (!follower.isBusy()) setState(State.CASE2A_SPEED_UP_SHOOT_4);
                break;
            case CASE2A_SPEED_UP_SHOOT_4:
                launchWhenAtSpeed(State.CASE2A_FEED_SHOOT_4);
                break;
            case CASE2A_FEED_SHOOT_4:
                if (!robot.isLaunching()) {
                    shutOff();
                    driveThen(tippedShootToParkPath, State.CASE2A_TRAVEL_TO_PARK);
                }
                break;
            case CASE2A_TRAVEL_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            // ---- CASE2B: hive still not tipped -> original shoot pose: shoot, scoring flower, park
            case CASE2B_TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) setState(State.CASE2B_SPEED_UP_SHOOT_3);
                break;
            case CASE2B_SPEED_UP_SHOOT_3:
                launchWhenAtSpeed(State.CASE2B_FEED_SHOOT_3);
                break;
            case CASE2B_FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(shootToScoringFlowerIntermediatePath, State.CASE2B_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE);
                }
                break;
            case CASE2B_TRAVEL_TO_SCORING_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerIntermediateToScoringFlowerPath, State.CASE2B_TRAVEL_TO_SCORING_FLOWER);
                }
                break;
            case CASE2B_TRAVEL_TO_SCORING_FLOWER:
                if (finishedCreepingIntoFlower()) {
                    robot.flower.startShake();
                    setState(State.CASE2B_COLLECT_SCORING_FLOWER);
                }
                break;
            case CASE2B_COLLECT_SCORING_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goDown();
                    driveThen(scoringFlowerToScoringFlowerIntermediatePath, State.CASE2B_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE2B_TRAVEL_FROM_SCORING_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goUp();
                    shutOff(); // last stop before park
                    driveThen(scoringFlowerIntermediateToParkPath, State.CASE2B_TRAVEL_TO_PARK);
                }
                break;
            case CASE2B_TRAVEL_TO_PARK:
                if (!follower.isBusy()) setState(State.DONE);
                break;

            case DONE:
                requestOpModeStop();
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
        vision.update(follower.pose().heading());
        follower.update();
        robot.flywheel.update(targetRPM, robot.getExpansionHubVoltage()); // re-run PIDF every loop so RPM actually converges on target also dih
        robot.flower.update();
        robot.update(); // advances the shooting state machine and intake power-save
        autoUpdate();
        robot.scheduler.update(loopTimer.nanoseconds()); // write hardware first: skipping this budget starves every device
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
        robot.scheduler.setAutoLoopLength(); // make sure we use the longer auto loops
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
            robot.flywheel.update(0, robot.getExpansionHubVoltage());
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
    }
}

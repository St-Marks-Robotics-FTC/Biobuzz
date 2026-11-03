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
 * hive TIPPED  -> CASE1: tippedShoot, flower 1, tippedShoot, flower 2, checkpoint (every "shoot" fires the flywheel)
 *     made all 8      -> CASE1A: original shoot pose, shoot, park
 *     missed some     -> CASE1B: tippedShoot, shoot, park
 *
 * hive NOT tipped -> CASE2: shoot again, audience flower 1, checkpoint
 *     tipped now      -> CASE2A: tippedShoot, shoot, flower 2, tippedShoot, shoot, park
 *     still not       -> CASE2B: original shoot pose, shoot, audience flower 2, park
 * </pre>
 *
 * The two "judge the hive" decisions at the checkpoints ({@link #madeAllEight()} and
 * {@link #hiveTippedAtCheckpoint()}) are hard-coded to {@code true} (CASE1A / CASE2A); replace with Limelight checks.
 * The new poses (checkpoints, audience flowers) are all (0, 0) in BlueAuto/RedAuto: tune them before running.
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
     * "_2"/"_3"/"_4" count the shots (1 = preloaded, 2 = after refuel, ...), "FLOWER_2" is the second flower.
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

        // flower 1: line to the intermediate pose, slow line into the flower, collect, back out, line to the tipped shoot pose, shoot
        CASE1_TRAVEL_TO_FLOWER_INTERMEDIATE,        // full-speed line from the tipped shoot pose to flower 1's intermediate pose
        CASE1_TRAVEL_TO_FLOWER,                     // reduced-speed line from the intermediate pose into flower 1
        CASE1_COLLECT_FLOWER,                       // sitting on flower 1 with intake running
        CASE1_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE,   // backing out of flower 1 to its intermediate pose
        CASE1_TRAVEL_TO_SHOOT_3,                    // line from the intermediate pose back to the tipped shoot pose
        CASE1_SPEED_UP_SHOOT_3,
        CASE1_FEED_SHOOT_3,

        // flower 2: curve to the intermediate pose, otherwise the same shape as flower 1; then on to the checkpoint
        CASE1_TRAVEL_TO_FLOWER_2_INTERMEDIATE,      // full-speed curve (1 control point) from the tipped shoot pose
        CASE1_TRAVEL_TO_FLOWER_2,                   // reduced-speed line into flower 2
        CASE1_COLLECT_FLOWER_2,
        CASE1_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE, // backing out of flower 2
        CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT, // line to a spot where the Limelight can judge the hive; decides 1A vs 1B

        // ---- CASE1A: we made all 8 and the hive flipped a second time -> shoot from the ORIGINAL shoot pose and park
        CASE1A_TRAVEL_TO_SHOOT_4,  // line from the checkpoint to the original shoot pose
        CASE1A_SPEED_UP_SHOOT_4,
        CASE1A_FEED_SHOOT_4,
        CASE1A_TRAVEL_TO_PARK,     // everything off; curve (1 control point) to the park pose

        // ---- CASE1B: we did not make all 8 -> go back to the SCORING side and shoot again
        CASE1B_TRAVEL_TO_SHOOT_4,  // line from the checkpoint to the tipped shoot pose
        CASE1B_SPEED_UP_SHOOT_4,
        CASE1B_FEED_SHOOT_4,
        CASE1B_TRAVEL_TO_PARK,     // everything off; curve (1 control point) from the tipped shoot pose to the park pose

        // ================================================================ CASE2: HIVE DID NOT TIP FIRST TRY
        // we are still at the original shoot pose, so there is no travel before the second shot
        CASE2_SPEED_UP_SHOOT_2,
        CASE2_FEED_SHOOT_2,

        // audience-side flower 1, then the checkpoint
        CASE2_TRAVEL_TO_FLOWER_INTERMEDIATE,        // full-speed line from the shoot pose to the audience flower's intermediate pose
        CASE2_TRAVEL_TO_FLOWER,
        CASE2_COLLECT_FLOWER,
        CASE2_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE,
        CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT, // line to the audience-side checkpoint; decides 2A vs 2B

        // ---- CASE2A: hive finally tipped -> shoot on the SCORING side, collect flower 2 there, shoot, park
        CASE2A_TRAVEL_TO_SHOOT_3,   // line from the checkpoint to the tipped shoot pose
        CASE2A_SPEED_UP_SHOOT_3,
        CASE2A_FEED_SHOOT_3,
        CASE2A_TRAVEL_TO_FLOWER_2_INTERMEDIATE,     // full-speed curve from the tipped shoot pose
        CASE2A_TRAVEL_TO_FLOWER_2,
        CASE2A_COLLECT_FLOWER_2,
        CASE2A_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE,
        CASE2A_TRAVEL_TO_SHOOT_4,   // line from flower 2's intermediate pose to the tipped shoot pose
        CASE2A_SPEED_UP_SHOOT_4,
        CASE2A_FEED_SHOOT_4,
        CASE2A_TRAVEL_TO_PARK,      // everything off; curve from the tipped shoot pose to the park pose

        // ---- CASE2B: hive still not tipped -> shoot AUDIENCE side, collect audience flower 2, park
        CASE2B_TRAVEL_TO_SHOOT_3,   // line from the checkpoint to the original shoot pose
        CASE2B_SPEED_UP_SHOOT_3,
        CASE2B_FEED_SHOOT_3,
        CASE2B_TRAVEL_TO_FLOWER_2_INTERMEDIATE,     // full-speed curve from the shoot pose
        CASE2B_TRAVEL_TO_FLOWER_2,
        CASE2B_COLLECT_FLOWER_2,
        CASE2B_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE,
        CASE2B_TRAVEL_TO_PARK,      // everything off; curve from the intermediate pose to the park pose

        DONE,                  // parked; waiting for the OpMode to be stopped
    }

    private State state = State.TRAVEL_TO_FIRST_SHOOT;

    // ======================================================================== PATHS
    // Naming: <from>To<to>Path. "Intermediate" is the waypoint in front of a flower; "tippedShoot" is the scoring-side
    // shoot pose; "shoot" is the original (audience-side) shoot pose. All are built once in buildPaths().

    // ---- SHARED: startPose -> shoot -> intermediate -> refuel -> shoot
    private Path startToShootPath, shootToIntermediatePath, intermediateToRefuelPath, refuelToShootPath;

    // ---- CASE1: hive tipped. shoot -> tippedShoot, flower 1 round trip, flower 2 round trip, then to the checkpoint
    private Path shootToTippedShootPath;
    private Path tippedShootToFlowerIntermediatePath, flowerIntermediateToFlowerPath,
            flowerToFlowerIntermediatePath, flowerIntermediateToTippedShootPath;
    private Path tippedShootToFlower2IntermediatePath, flower2IntermediateToFlower2Path,
            flower2ToFlower2IntermediatePath, flower2IntermediateToCheckpointPath;
    // CASE1A: made all 8 -> original shoot pose -> park
    private Path checkpointToShootPath, shootToParkPath;
    // CASE1B: missed some -> tippedShoot -> park (also used by CASE2A)
    private Path checkpointToTippedShootPath, tippedShootToParkPath;

    // ---- CASE2: hive not tipped. Shoot again from where we are, audience flower 1 round trip, then to the checkpoint
    private Path shootToAudienceFlowerIntermediatePath, audienceFlowerIntermediateToAudienceFlowerPath,
            audienceFlowerToAudienceFlowerIntermediatePath, audienceFlowerIntermediateToAudienceCheckpointPath;
    // CASE2A: tipped now -> tippedShoot -> flower 2 round trip (CASE1's flower 2 paths) -> tippedShoot -> park
    private Path audienceCheckpointToTippedShootPath, flower2IntermediateToTippedShootPath;
    // CASE2B: still not tipped -> original shoot pose -> audience flower 2 round trip -> park
    private Path audienceCheckpointToShootPath,
            shootToAudienceFlower2IntermediatePath, audienceFlower2IntermediateToAudienceFlower2Path,
            audienceFlower2ToAudienceFlower2IntermediatePath, audienceFlower2IntermediateToParkPath;

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
        // flower 1: full-speed line out, creep in, line back out, line to tippedShoot
        tippedShootToFlowerIntermediatePath = plainLine(config.tippedShootPose, config.flowerIntermediatePose);
        flowerIntermediateToFlowerPath = creepLine(config.flowerIntermediatePose, config.flowerPose);
        flowerToFlowerIntermediatePath = plainLine(config.flowerPose, config.flowerIntermediatePose);
        flowerIntermediateToTippedShootPath = plainLine(config.flowerIntermediatePose, config.tippedShootPose);
        // flower 2: like flower 1 but the full-speed approach is a curve (1 control point); then on to the checkpoint
        tippedShootToFlower2IntermediatePath = curve(config.tippedShootPose, config.flower2ControlPose, config.flower2IntermediatePose)
                .linear(config.tippedShootPose, config.flower2IntermediatePose);
        flower2IntermediateToFlower2Path = creepLine(config.flower2IntermediatePose, config.flower2Pose);
        flower2ToFlower2IntermediatePath = plainLine(config.flower2Pose, config.flower2IntermediatePose);
        flower2IntermediateToCheckpointPath = plainLine(config.flower2IntermediatePose, config.checkpointPose);
        // CASE1A: checkpoint -> ORIGINAL shoot pose -> park (curve, 1 control point)
        checkpointToShootPath = plainLine(config.checkpointPose, shootPose);
        shootToParkPath = curve(shootPose, config.parkControlPose, config.parkPose)
                .linear(shootPose, config.parkPose);
        // CASE1B: checkpoint -> tippedShoot -> park (curve, 1 control point)
        checkpointToTippedShootPath = plainLine(config.checkpointPose, config.tippedShootPose);
        tippedShootToParkPath = curve(config.tippedShootPose, config.parkControlPose, config.parkPose)
                .linear(config.tippedShootPose, config.parkPose);

        // ---------------------------------------------------------------- CASE2: hive not tipped
        // audience flower 1: full-speed line out, creep in, line back out, line to the checkpoint
        shootToAudienceFlowerIntermediatePath = plainLine(shootPose, config.audienceFlowerIntermediatePose);
        audienceFlowerIntermediateToAudienceFlowerPath = creepLine(config.audienceFlowerIntermediatePose, config.audienceFlowerPose);
        audienceFlowerToAudienceFlowerIntermediatePath = plainLine(config.audienceFlowerPose, config.audienceFlowerIntermediatePose);
        audienceFlowerIntermediateToAudienceCheckpointPath = plainLine(config.audienceFlowerIntermediatePose, config.audienceCheckpointPose);
        // CASE2A: checkpoint -> tippedShoot; later flower 2's intermediate pose -> tippedShoot
        // (flower 2 itself reuses the CASE1 scoring-side flower 2 paths above)
        audienceCheckpointToTippedShootPath = plainLine(config.audienceCheckpointPose, config.tippedShootPose);
        flower2IntermediateToTippedShootPath = plainLine(config.flower2IntermediatePose, config.tippedShootPose);
        // CASE2B: checkpoint -> ORIGINAL shoot pose -> audience flower 2 round trip -> park
        audienceCheckpointToShootPath = plainLine(config.audienceCheckpointPose, shootPose);
        shootToAudienceFlower2IntermediatePath = curve(shootPose, config.audienceFlower2ControlPose, config.audienceFlower2IntermediatePose)
                .linear(shootPose, config.audienceFlower2IntermediatePose);
        audienceFlower2IntermediateToAudienceFlower2Path = creepLine(config.audienceFlower2IntermediatePose, config.audienceFlower2Pose);
        audienceFlower2ToAudienceFlower2IntermediatePath = plainLine(config.audienceFlower2Pose, config.audienceFlower2IntermediatePose);
        audienceFlower2IntermediateToParkPath = curve(config.audienceFlower2IntermediatePose, config.parkControlPose, config.parkPose)
                .linear(config.audienceFlower2IntermediatePose, config.parkPose);
    }

    /**
     * Decision at the CASE1 checkpoint.
     * @return true if we made all 8 balls and the hive flipped a second time (-> CASE1A); false if we missed some
     *         (-> CASE1B). Currently hard-coded; replace with a real Limelight check.
     */
    private boolean madeAllEight() {
        return true;
    }

    /**
     * Decision at the CASE2 checkpoint.
     * @return true if the hive is now tipped (-> CASE2A); false if it still is not (-> CASE2B). Currently
     *         hard-coded; replace with a real Limelight check.
     */
    private boolean hiveTippedAtCheckpoint() {
        return true;
    }

    /** @return true once a path has finished, or {@link Tunables#maxTravelToRefuelTime} passed (stuck on a ball). **/
    private boolean finishedCreepingIntoFlower() {
        return !follower.isBusy() || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime;
    }

    /** @return true once we have sat on a flower with the intake running for {@link Tunables#refuelDurationMillis}. **/
    private boolean finishedCollecting() {
        return stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis;
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
                if (finishedCollecting()) {
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
                    robot.intake.forward(); // flywheel keeps spinning: we shoot again after the flower
                    driveThen(tippedShootToFlowerIntermediatePath, State.CASE1_TRAVEL_TO_FLOWER_INTERMEDIATE);
                }
                break;

            // flower 1, then shoot again from the same tipped shoot pose
            case CASE1_TRAVEL_TO_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(flowerIntermediateToFlowerPath, State.CASE1_TRAVEL_TO_FLOWER);
                }
                break;
            case CASE1_TRAVEL_TO_FLOWER:
                if (finishedCreepingIntoFlower()) setState(State.CASE1_COLLECT_FLOWER);
                break;
            case CASE1_COLLECT_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goUp(); // raise as soon as we leave the flower
                    driveThen(flowerToFlowerIntermediatePath, State.CASE1_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE1_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) driveThen(flowerIntermediateToTippedShootPath, State.CASE1_TRAVEL_TO_SHOOT_3);
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
                    driveThen(tippedShootToFlower2IntermediatePath, State.CASE1_TRAVEL_TO_FLOWER_2_INTERMEDIATE);
                }
                break;

            // flower 2, then the checkpoint
            case CASE1_TRAVEL_TO_FLOWER_2_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(flower2IntermediateToFlower2Path, State.CASE1_TRAVEL_TO_FLOWER_2);
                }
                break;
            case CASE1_TRAVEL_TO_FLOWER_2:
                if (finishedCreepingIntoFlower()) setState(State.CASE1_COLLECT_FLOWER_2);
                break;
            case CASE1_COLLECT_FLOWER_2:
                if (finishedCollecting()) {
                    robot.flower.goUp();
                    driveThen(flower2ToFlower2IntermediatePath, State.CASE1_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE);
                }
                break;
            case CASE1_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE:
                if (!follower.isBusy()) driveThen(flower2IntermediateToCheckpointPath, State.CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT);
                break;
            case CASE1_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT:
                if (!follower.isBusy()) {
                    if (madeAllEight()) driveThen(checkpointToShootPath, State.CASE1A_TRAVEL_TO_SHOOT_4);
                    else driveThen(checkpointToTippedShootPath, State.CASE1B_TRAVEL_TO_SHOOT_4);
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
                    driveThen(shootToAudienceFlowerIntermediatePath, State.CASE2_TRAVEL_TO_FLOWER_INTERMEDIATE);
                }
                break;

            // audience-side flower 1, then the checkpoint
            case CASE2_TRAVEL_TO_FLOWER_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(audienceFlowerIntermediateToAudienceFlowerPath, State.CASE2_TRAVEL_TO_FLOWER);
                }
                break;
            case CASE2_TRAVEL_TO_FLOWER:
                if (finishedCreepingIntoFlower()) setState(State.CASE2_COLLECT_FLOWER);
                break;
            case CASE2_COLLECT_FLOWER:
                if (finishedCollecting()) {
                    robot.flower.goUp();
                    driveThen(audienceFlowerToAudienceFlowerIntermediatePath, State.CASE2_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE);
                }
                break;
            case CASE2_TRAVEL_FROM_FLOWER_TO_INTERMEDIATE:
                if (!follower.isBusy()) driveThen(audienceFlowerIntermediateToAudienceCheckpointPath, State.CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT);
                break;
            case CASE2_TRAVEL_FROM_INTERMEDIATE_TO_CHECKPOINT:
                if (!follower.isBusy()) {
                    if (hiveTippedAtCheckpoint()) driveThen(audienceCheckpointToTippedShootPath, State.CASE2A_TRAVEL_TO_SHOOT_3);
                    else driveThen(audienceCheckpointToShootPath, State.CASE2B_TRAVEL_TO_SHOOT_3);
                }
                break;

            // ---- CASE2A: hive finally tipped -> scoring side: shoot, flower 2, shoot, park
            case CASE2A_TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) setState(State.CASE2A_SPEED_UP_SHOOT_3);
                break;
            case CASE2A_SPEED_UP_SHOOT_3:
                launchWhenAtSpeed(State.CASE2A_FEED_SHOOT_3);
                break;
            case CASE2A_FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(tippedShootToFlower2IntermediatePath, State.CASE2A_TRAVEL_TO_FLOWER_2_INTERMEDIATE);
                }
                break;
            case CASE2A_TRAVEL_TO_FLOWER_2_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(flower2IntermediateToFlower2Path, State.CASE2A_TRAVEL_TO_FLOWER_2);
                }
                break;
            case CASE2A_TRAVEL_TO_FLOWER_2:
                if (finishedCreepingIntoFlower()) setState(State.CASE2A_COLLECT_FLOWER_2);
                break;
            case CASE2A_COLLECT_FLOWER_2:
                if (finishedCollecting()) {
                    robot.flower.goUp();
                    driveThen(flower2ToFlower2IntermediatePath, State.CASE2A_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE);
                }
                break;
            case CASE2A_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE:
                if (!follower.isBusy()) driveThen(flower2IntermediateToTippedShootPath, State.CASE2A_TRAVEL_TO_SHOOT_4);
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

            // ---- CASE2B: hive still not tipped -> original shoot pose: shoot, audience flower 2, park
            case CASE2B_TRAVEL_TO_SHOOT_3:
                if (!follower.isBusy()) setState(State.CASE2B_SPEED_UP_SHOOT_3);
                break;
            case CASE2B_SPEED_UP_SHOOT_3:
                launchWhenAtSpeed(State.CASE2B_FEED_SHOOT_3);
                break;
            case CASE2B_FEED_SHOOT_3:
                if (!robot.isLaunching()) {
                    robot.intake.forward();
                    driveThen(shootToAudienceFlower2IntermediatePath, State.CASE2B_TRAVEL_TO_FLOWER_2_INTERMEDIATE);
                }
                break;
            case CASE2B_TRAVEL_TO_FLOWER_2_INTERMEDIATE:
                if (!follower.isBusy()) {
                    robot.flower.goDown();
                    driveThen(audienceFlower2IntermediateToAudienceFlower2Path, State.CASE2B_TRAVEL_TO_FLOWER_2);
                }
                break;
            case CASE2B_TRAVEL_TO_FLOWER_2:
                if (finishedCreepingIntoFlower()) setState(State.CASE2B_COLLECT_FLOWER_2);
                break;
            case CASE2B_COLLECT_FLOWER_2:
                if (finishedCollecting()) {
                    robot.flower.goUp();
                    driveThen(audienceFlower2ToAudienceFlower2IntermediatePath, State.CASE2B_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE);
                }
                break;
            case CASE2B_TRAVEL_FROM_FLOWER_2_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    shutOff(); // last stop before park
                    driveThen(audienceFlower2IntermediateToParkPath, State.CASE2B_TRAVEL_TO_PARK);
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

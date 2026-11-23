package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.branch;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.sequence;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.auto.steps.AutoContext;
import org.firstinspires.ftc.teamcode.auto.steps.RobotSteps;

/**
 * The solo hive autonomous; Blue and Red only supply poses and the alliance. The route is {@link #buildRoute}.
 *
 * <pre>
 * SHARED   start -> shoot -> intermediate -> refuel -> shoot -> (look at the hive)
 *
 * tags NOT visible (hive TIPPED) -> CASE1: tippedShoot, scoring flower, tippedShoot, audience flower, checkpoint
 *     tags visible     -> CASE1A: original shoot pose, shoot, park
 *     tags not visible -> CASE1B: tippedShoot, shoot, park
 *
 * tags visible (hive NOT tipped) -> CASE2: shoot again, audience flower, checkpoint
 *     tags not visible -> CASE2A: tippedShoot, shoot, scoring flower, tippedShoot, shoot, park
 *     tags visible     -> CASE2B: original shoot pose, shoot, scoring flower, park
 * </pre>
 */
@Configurable
public abstract class BozoAuto extends AutoOpMode {
    /** configurables **/

    public static boolean IS_DEBUGGING = false;
    public static double FLOWER_DISTANCE = 13.875; // how far from the edge of the field where the flower is should we refuel
    public static double TAG_THRESHOLD = 2; // amount of tags we need at least
    public static double SHOOT_RPM = 3100; // target flywheel RPM while lined up to shoot
    public static double FLYWHEEL_RPM_MARGIN = 75; // how close to shootRPM we must be before feeding balls
    public static long FLOWER_REFUEL_DURATION_MILLIS = 450; // how long to intake while driving through the refuel line
    public static long REFUEL_DURATION_MILLIS = 150; // how long to intake while driving through the refuel line
    public static double FLOWER_INTAKE_SPEED = 0.3; // 0-1 speed to intake flower (lower=slower)
    public static double MAX_TRAVEL_TO_REFUEL_TIME = 750; // timeout for travel to picking up balls (if it gets stuck on a ball)
    public static double CROSS_FIELD_HOLD_FROM = 0.05; // fraction of cross field path spent holding initial heading before turning to cross field heading.
    public static double CROSS_FIELD_HOLD_UNTIL = 0.8; // fraction (0-1] of a cross-field path spent holding 90/270 heading before turning to the final heading; read when paths are built at init
    public static double FLOWER_APPROACH_SPEED = 0.5;

    /** end configurables **/
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();

    @Override
    public void init() {
        config = buildConfig(); // before super.init(): startPose() and buildRoute() read it
        super.init();
    }

    @Override
    protected Pose startPose() {
        return config.startPose;
    }

    @Override
    protected Step buildRoute(AutoContext ctx) {
        RobotSteps s = new RobotSteps(ctx);
        AutoPaths p = new AutoPaths(config);

        // shared by more than one case
        Step scoringFlowerLap = sequence("scoring flower lap",
                s.collectFlower(p.scoringFlower, p.tippedShootToScoringFlowerIntermediate),
                s.travelShoot("back to tipped shoot", p.scoringFlowerIntermediateToTippedShoot));
        Step toCheckpoint = s.drive("to checkpoint", p.audienceFlowerIntermediateToCheckpoint);

        // ---- CASE1: hive tipped
        Step case1A = sequence("CASE1A",
                s.travelShoot("to shoot", p.checkpointToShoot),
                s.park(p.shootToPark));
        Step case1B = sequence("CASE1B",
                s.travelShoot("to tipped shoot", p.checkpointToTippedShoot),
                s.park(p.tippedShootToPark));
        Step case1 = sequence("CASE1",
                s.travelShoot("to tipped shoot", p.shootToTippedShoot),
                scoringFlowerLap,
                s.collectFlower(p.audienceFlower, p.tippedShootToAudienceFlowerIntermediate),
                toCheckpoint,
                branch("tags visible", s::hiveTagsVisible, case1A, case1B));

        // ---- CASE2: hive not tipped; still on the shoot pose
        Step case2A = sequence("CASE2A",
                s.travelShoot("to tipped shoot", p.checkpointToTippedShoot),
                scoringFlowerLap,
                s.park(p.tippedShootToPark));
        Step case2B = sequence("CASE2B",
                s.travelShoot("to shoot", p.checkpointToShoot),
                s.collectFlower(p.scoringFlower, p.shootToScoringFlowerIntermediate),
                s.park(p.scoringFlowerIntermediateToPark));
        Step case2 = sequence("CASE2",
                s.shoot(),
                s.collectFlower(p.audienceFlower, p.shootToAudienceFlowerIntermediate),
                toCheckpoint,
                branch("tags visible", s::hiveTagsVisible, case2B, case2A));

        return sequence("hive auto",
                s.spinUpFlywheel(), // spin up while driving to the first shot
                s.travelShoot("first shoot", p.startToShoot),
                s.drive("to intermediate", p.shootToIntermediate),
                s.refuel(p.intermediateToRefuel),
                s.drive("refuel to shoot", p.refuelToShoot),
                s.flowerDown(),
                branch("tags visible", s::hiveTagsVisible, case2, case1));
    }
}

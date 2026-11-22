package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.auto.routes.HiveRoute;
import org.firstinspires.ftc.teamcode.auto.steps.AutoContext;
import org.firstinspires.ftc.teamcode.auto.steps.HiveView;
import org.firstinspires.ftc.teamcode.auto.steps.RobotSteps;

/**
 * The solo hive autonomous. Poses come from {@link AutoConfig}, paths from {@link AutoPaths}, the sequence of actions
 * from {@link HiveRoute}; the lifecycle is {@link AutoOpMode}. Blue and Red only supply poses and the alliance.
 */
@Configurable
public abstract class BozoAuto extends AutoOpMode {
    /** configurables **/

    public static boolean IS_DEBUGGING = false;
    public static double FLOWER_DISTANCE = 13.500; // how far from the edge of the field where the flower is should we refuel
    public static double TAG_THRESHOLD = 2; // amount of tags we need at least
    public static double SHOOT_RPM = 3100; // target flywheel RPM while lined up to shoot
    public static double FLYWHEEL_RPM_MARGIN = 75; // how close to shootRPM we must be before feeding balls
    public static long FLOWER_REFUEL_DURATION_MILLIS = 625; // how long to intake while driving through the refuel line
    public static long REFUEL_DURATION_MILLIS = 150; // how long to intake while driving through the refuel line
    public static double FLOWER_INTAKE_SPEED = 0.3; // 0-1 speed to intake flower (lower=slower)
    public static double MAX_TRAVEL_TO_REFUEL_TIME = 1500; // timeout for travel to picking up balls (if it gets stuck on a ball)
    public static double CROSS_FIELD_HOLD_UNTIL = 0.8; // fraction (0-1] of a cross-field path spent holding 90/270 heading before turning to the final heading; read when paths are built at init
    public static double FLOWER_APPROACH_SPEED = .25;

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
        return HiveRoute.build(new RobotSteps(ctx), new AutoPaths(config), new HiveView(ctx.vision));
    }
}

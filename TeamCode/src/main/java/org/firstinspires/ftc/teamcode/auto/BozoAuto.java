package org.firstinspires.ftc.teamcode.auto;

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
public abstract class BozoAuto extends AutoOpMode {
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

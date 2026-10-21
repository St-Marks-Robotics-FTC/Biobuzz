package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose startPose = new Pose(60, 15, Math.toRadians(90)); // where the robot is placed at init; drives to shootPose first
    public static Pose shootPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210)); // approach heading toward the wall (mirror of Blue's 330)
    public static Pose refuelPose = new Pose(9, 6.754, Math.toRadians(180));
    public static Pose parkControlPose = new Pose(36, 24, Math.toRadians(0)); // bezier control point: pulls the park path into a curve; x/y only
    public static Pose parkPose = new Pose(18, 102, Math.toRadians(0));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                parkControlPose,
                parkPose
        );
    }
}
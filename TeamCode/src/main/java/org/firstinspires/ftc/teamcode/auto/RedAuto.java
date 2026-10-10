package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose shootPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210)); // approach heading toward the wall (mirror of Blue's 330)
    public static Pose refuelPose = new Pose(9, 6.754, Math.toRadians(180));
    public static Pose parkControlPose = new Pose(48, 24, Math.toRadians(0)); // bezier control point: pulls the park path into a curve; x/y only
    public static Pose parkPose = new Pose(36, 12, Math.toRadians(0)); // mirror of Blue's park pose (x -> 144 - x, heading 180 -> 0)

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                shootPose,
                intermediatePose,
                refuelPose,
                parkControlPose,
                parkPose
        );
    }
}
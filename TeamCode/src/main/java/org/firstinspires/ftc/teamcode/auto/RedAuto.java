package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose shootPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210)); // approach heading toward the wall (mirror of Blue's 330)
    public static Pose refuelPose = new Pose(9, 6.504, Math.toRadians(180));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                shootPose,
                intermediatePose,
                refuelPose
        );
    }
}
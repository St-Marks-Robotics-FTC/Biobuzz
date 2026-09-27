package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose startPose = new Pose(60, 12, Math.toRadians(270));
    public static Pose shootRightPose = new Pose(60, 24, Math.toRadians(225));
    public static Pose refuelPose = new Pose(10, 10, Math.toRadians(225));
    public static Pose endPose = new Pose(8, 108, Math.toRadians(270));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootRightPose,
                refuelPose,
                endPose
        );
    }
}
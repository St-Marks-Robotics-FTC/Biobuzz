package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 132, Math.toRadians(90));
    public static Pose shootRightPose = new Pose(84, 120, Math.toRadians(45));
    public static Pose refuelPose = new Pose(134, 134, Math.toRadians(45));
    public static Pose endPose = new Pose(136, 36, Math.toRadians(90));

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
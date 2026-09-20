package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(144 - 12, 144 - 72, Math.toRadians(270));
    public static Pose shootLeftPose = new Pose(144 - 60, 144 - 108, Math.toRadians(270));
    public static Pose shootRightPose = new Pose(144 - 60, 144 - 36, Math.toRadians(0));
    public static Pose flowerLeftPose = new Pose(144 - 48, 144 - 132, Math.toRadians(0));
    public static Pose flowerRightPose = new Pose(144 - 12, 144 - 48, Math.toRadians(180));
    public static Pose endPose = new Pose(144 - 12, 144 - 72, Math.toRadians(0));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootLeftPose,
                shootRightPose,
                flowerLeftPose,
                flowerRightPose,
                endPose
        );
    }
}

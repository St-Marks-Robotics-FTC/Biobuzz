package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(12, 72, Math.toRadians(90));
    public static Pose shootLeftPose = new Pose(60, 108, Math.toRadians(90));
    public static Pose shootRightPose = new Pose(60, 36, Math.toRadians(90));
    public static Pose flowerLeftPose = new Pose(0, 0, Math.toRadians(90));
    public static Pose flowerRightPose = new Pose(0, 0, Math.toRadians(90));
    public static Pose endPose = new Pose(0, 0, Math.toRadians(90));

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
}

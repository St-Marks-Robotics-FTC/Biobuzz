package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static int startX = 0;
    public static int startY = 0;
    public static int startAngle = 0;
    public static Pose startPose = new Pose(0, 0, Math.toRadians(90));
    public static Pose shootLeftPose = new Pose(24, 24, Math.toRadians(45));
    public static Pose shootRightPose = new Pose(48, 24, Math.toRadians(135));
    public static Pose flowerLeftPose = new Pose(12, 48, Math.toRadians(0));
    public static Pose flowerRightPose = new Pose(60, 48, Math.toRadians(180));
    public static Pose endPose = new Pose(36, 36, Math.toRadians(90));

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
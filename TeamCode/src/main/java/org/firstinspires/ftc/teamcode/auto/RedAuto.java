package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static int startX = 0;
    public static int startY = 0;
    public static int startAngle = 0;
    public static Pose startPose = new Pose(144, 144, Math.toRadians(270));
    public static Pose shootLeftPose = new Pose(120, 120, Math.toRadians(225));
    public static Pose shootRightPose = new Pose(96, 120, Math.toRadians(315));
    public static Pose flowerLeftPose = new Pose(132, 96, Math.toRadians(180));
    public static Pose flowerRightPose = new Pose(84, 96, Math.toRadians(0));
    public static Pose endPose = new Pose(108, 108, Math.toRadians(270));

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
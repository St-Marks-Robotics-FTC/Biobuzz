package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static int startX = 0;
    public static int startY = 0;
    public static int startAngle = 0;
    public static Pose startPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));
    public static Pose shootLeftPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));
    public static Pose shootRightPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));
    public static Pose flowerLeftPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));
    public static Pose flowerRightPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));
    public static Pose endPose = new Pose(startX, startY, Math.toRadians(startAngle) + Math.toRadians(90));

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
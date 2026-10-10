package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose shootPose = new Pose(84, 6, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(104, 6, Math.toRadians(330)); // approach heading toward the wall
    public static Pose refuelPose = new Pose(138, 6, Math.toRadians(0));
    public static Pose parkControlPose = new Pose(96, 24, Math.toRadians(180)); // bezier control point: pulls the park path into a curve; x/y only
    public static Pose parkPose = new Pose(108, 12, Math.toRadians(180));


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
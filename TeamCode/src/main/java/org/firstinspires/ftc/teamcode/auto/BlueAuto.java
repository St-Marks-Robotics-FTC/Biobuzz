package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270)); // where the robot is placed at init; drives to shootPose first
    public static Pose shootPose = new Pose(84, 111.246, Math.toRadians(270));
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30)); // approach heading toward the wall (mirror of Red's 210)
    public static Pose refuelPose = new Pose(133, 135.246, Math.toRadians(0));
    public static Pose parkControlPose = new Pose(108, 120, Math.toRadians(180)); // bezier control point: pulls the park path into a curve; x/y only
    public static Pose parkPose = new Pose(132, 42, Math.toRadians(180));


    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                parkControlPose,
                parkPose
        );
    }
}
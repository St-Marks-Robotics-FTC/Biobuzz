package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270)); // where the robot is placed at init; drives to shootPose first
    public static Pose shootPose = new Pose(84, 134, Math.toRadians(270)); // y 111.246
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30)); // approach heading toward the wall (mirror of Red's 210)
    public static Pose refuelPose = new Pose(133, 135.246, Math.toRadians(0));
    public static Pose parkControlPose = new Pose(108, 120, Math.toRadians(180)); // bezier control point: pulls the park path into a curve; x/y only
    public static Pose parkPose = new Pose(132, 42, Math.toRadians(180));
    // CASE 2 (hive tipped)
    public static Pose tippedControlPose = new Pose(72, 72, Math.toRadians(270)); // bezier control point toward scoring side; x/y only
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(90)); // scoring side: shootPose mirrored over y = 72 (y -> 144 - y)
    public static Pose tippedParkControlPose = new Pose(96, 60, Math.toRadians(180)); // bezier control point tippedShoot -> park; x/y only


    @Override
    protected boolean isBlueTeam() { return true; }

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                parkControlPose,
                parkPose,
                tippedControlPose,
                tippedShootPose,
                tippedParkControlPose
        );
    }
}
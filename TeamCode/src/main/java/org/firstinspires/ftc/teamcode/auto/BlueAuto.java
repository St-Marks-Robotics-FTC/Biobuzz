package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270));
    public static Pose shootPose = new Pose(84, 134, Math.toRadians(270));

    // refuel run before the hive tips
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30));
    public static Pose refuelPose = new Pose(134, 137.246, Math.toRadians(0));

    // hive tipped: scoring side (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(132, 132, Math.toRadians(270));
    public static Pose tippedControlPose2 = new Pose(126, 20, Math.toRadians(270));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(90));

    // flowers, collected after the tipped shoot
    public static Pose flowerIntermediatePose = new Pose(96, 24, Math.toRadians(270));
    public static Pose flowerPose = new Pose(96, 14, Math.toRadians(270));
    public static Pose flower2ControlPose = new Pose(120, 24, Math.toRadians(0));
    public static Pose flower2IntermediatePose = new Pose(120, 96, Math.toRadians(0));
    public static Pose flower2Pose = new Pose(130, 96, Math.toRadians(0));

    public static Pose parkControlPose = new Pose(120, 120, Math.toRadians(180));
    public static Pose parkPose = new Pose(132, 42, Math.toRadians(180));

    @Override
    protected boolean isBlueTeam() { return true; }

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                tippedControlPose1,
                tippedControlPose2,
                tippedShootPose,
                flowerIntermediatePose,
                flowerPose,
                flower2ControlPose,
                flower2IntermediatePose,
                flower2Pose,
                parkControlPose,
                parkPose
        );
    }
}

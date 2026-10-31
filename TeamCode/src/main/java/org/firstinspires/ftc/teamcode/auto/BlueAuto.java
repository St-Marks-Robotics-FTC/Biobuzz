package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270));
    public static Pose shootPose = new Pose(84, 134, Math.toRadians(270));

    // refuel run before the hive tips
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30));
    public static Pose refuelPose = new Pose(133, 135.246, Math.toRadians(0));

    // hive tipped: scoring side (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(72, 72, Math.toRadians(270));
    public static Pose tippedControlPose2 = new Pose(96, 72, Math.toRadians(270)); // DUMMY coords, replace with tuned values
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(90));

    // flowers, collected after the tipped shoot
    public static Pose flowerControlPose = new Pose(96, 84, Math.toRadians(0));
    public static Pose flowerIntermediatePose = new Pose(108, 96, Math.toRadians(0));
    public static Pose flowerPose = new Pose(120, 96, Math.toRadians(0));
    public static Pose flower2ControlPose = new Pose(96, 108, Math.toRadians(0));
    public static Pose flower2IntermediatePose = new Pose(108, 120, Math.toRadians(0));
    public static Pose flower2Pose = new Pose(120, 120, Math.toRadians(0));

    public static Pose parkControlPose = new Pose(108, 120, Math.toRadians(180));
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
                flowerControlPose,
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

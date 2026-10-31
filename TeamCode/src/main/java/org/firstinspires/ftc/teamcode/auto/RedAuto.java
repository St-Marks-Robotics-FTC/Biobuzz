package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose startPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose shootPose = new Pose(60, 10, Math.toRadians(90));

    // refuel run before the hive tips
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210));
    public static Pose refuelPose = new Pose(10, 6.754, Math.toRadians(180));

    // hive tipped: scoring side (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(72, 72, Math.toRadians(90));
    public static Pose tippedControlPose2 = new Pose(48, 72, Math.toRadians(90));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(270));

    // flowers, collected after the tipped shoot
    public static Pose flowerIntermediatePose = new Pose(36, 96, Math.toRadians(180));
    public static Pose flowerPose = new Pose(24, 96, Math.toRadians(180));
    public static Pose flower2ControlPose = new Pose(48, 108, Math.toRadians(180));
    public static Pose flower2IntermediatePose = new Pose(36, 120, Math.toRadians(180));
    public static Pose flower2Pose = new Pose(24, 120, Math.toRadians(180));

    public static Pose parkControlPose = new Pose(36, 24, Math.toRadians(0));
    public static Pose parkPose = new Pose(12, 102, Math.toRadians(0));

    @Override
    protected boolean isBlueTeam() { return false; }

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

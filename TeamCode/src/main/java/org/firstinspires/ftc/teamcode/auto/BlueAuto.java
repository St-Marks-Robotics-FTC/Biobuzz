package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

/** Blue-alliance poses. See {@link AutoConfig} for what each pose means and the route that uses it. **/
@Configurable
public abstract class BlueAuto extends BozoAuto {
    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270));
    public static Pose shootPose = new Pose(84, 134, Math.toRadians(270));
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30));
    public static Pose refuelPose = new Pose(134, 137.246, Math.toRadians(0));

    // ---------------------------------------------------------------- SCORING SIDE (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(132, 132, Math.toRadians(270));
    public static Pose tippedControlPose2 = new Pose(126, 20, Math.toRadians(270));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(90));

    // ---------------------------------------------------------------- CASE1 / CASE2A: scoring-side flowers
    public static Pose flowerIntermediatePose = new Pose(96, 24, Math.toRadians(270));
    public static Pose flowerPose = new Pose(96, 14, Math.toRadians(270));
    public static Pose flower2ControlPose = new Pose(120, 24, Math.toRadians(0));
    public static Pose flower2IntermediatePose = new Pose(120, 96, Math.toRadians(0));
    public static Pose flower2Pose = new Pose(130, 96, Math.toRadians(0));

    // ---------------------------------------------------------------- CASE1: Limelight checkpoint  (UNTUNED: 0,0)
    public static Pose checkpointPose = new Pose(0, 0, Math.toRadians(0));

    // ---------------------------------------------------------------- CASE2: audience-side flowers  (UNTUNED: 0,0)
    public static Pose audienceFlowerIntermediatePose = new Pose(0, 0, Math.toRadians(0));
    public static Pose audienceFlowerPose = new Pose(0, 0, Math.toRadians(0));
    public static Pose audienceCheckpointPose = new Pose(0, 0, Math.toRadians(0));
    public static Pose audienceFlower2ControlPose = new Pose(0, 0, Math.toRadians(0));
    public static Pose audienceFlower2IntermediatePose = new Pose(0, 0, Math.toRadians(0));
    public static Pose audienceFlower2Pose = new Pose(0, 0, Math.toRadians(0));

    // ---------------------------------------------------------------- PARK
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
                checkpointPose,
                audienceFlowerIntermediatePose,
                audienceFlowerPose,
                audienceCheckpointPose,
                audienceFlower2ControlPose,
                audienceFlower2IntermediatePose,
                audienceFlower2Pose,
                parkControlPose,
                parkPose
        );
    }
}

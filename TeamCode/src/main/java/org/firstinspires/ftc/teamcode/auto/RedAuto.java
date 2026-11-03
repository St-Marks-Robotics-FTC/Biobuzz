package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

/** Red-alliance poses. See {@link AutoConfig} for what each pose means and the route that uses it. **/
@Configurable
public abstract class RedAuto extends BozoAuto {
    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    public static Pose startPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose shootPose = new Pose(60, 10, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210));
    public static Pose refuelPose = new Pose(10, 6.754, Math.toRadians(180));

    // ---------------------------------------------------------------- SCORING SIDE (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(12, 12, Math.toRadians(90));
    public static Pose tippedControlPose2 = new Pose(18, 90, Math.toRadians(90));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(270));

    // ---------------------------------------------------------------- CASE1 / CASE2A: scoring-side flowers
    public static Pose flowerIntermediatePose = new Pose(48, 120, Math.toRadians(90));
    public static Pose flowerPose = new Pose(48, 136, Math.toRadians(90));
    public static Pose flower2ControlPose = new Pose(18, 90, Math.toRadians(180));
    public static Pose flower2IntermediatePose = new Pose(24, 48, Math.toRadians(180));
    public static Pose flower2Pose = new Pose(8, 48, Math.toRadians(180));

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
    public static Pose parkControlPose = new Pose(24, 24, Math.toRadians(0));
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

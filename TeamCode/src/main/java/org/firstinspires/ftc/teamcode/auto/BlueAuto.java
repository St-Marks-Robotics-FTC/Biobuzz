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

    // ---------------------------------------------------------------- SCORING SIDE: the scoring flower + CASE1 checkpoint
    public static Pose scoringFlowerIntermediatePose = new Pose(96, 24, Math.toRadians(270));
    public static Pose scoringFlowerPose = new Pose(96, 14, Math.toRadians(270));
    public static Pose checkpointPose = new Pose(0, 0, Math.toRadians(0));                  // UNTUNED

    // ---------------------------------------------------------------- AUDIENCE SIDE: the audience flower + CASE2 checkpoint
    public static Pose tippedShootToAudienceFlowerControlPose = new Pose(120, 24, Math.toRadians(0));
    public static Pose audienceFlowerIntermediatePose = new Pose(120, 96, Math.toRadians(0));
    public static Pose audienceFlowerPose = new Pose(130, 96, Math.toRadians(0));
    public static Pose audienceCheckpointPose = new Pose(0, 0, Math.toRadians(0));          // UNTUNED
    public static Pose shootToScoringFlowerControlPose = new Pose(0, 0, Math.toRadians(0)); // UNTUNED (CASE2B)

    // ---------------------------------------------------------------- PARK
    public static Pose parkControlPoseFromShoot = new Pose(0, 0, Math.toRadians(180));              // UNTUNED
    public static Pose parkControlPoseFromTippedShoot = new Pose(120, 120, Math.toRadians(180));
    public static Pose parkControlPoseFromScoringFlower = new Pose(0, 0, Math.toRadians(180));      // UNTUNED
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
                scoringFlowerIntermediatePose,
                scoringFlowerPose,
                checkpointPose,
                tippedShootToAudienceFlowerControlPose,
                audienceFlowerIntermediatePose,
                audienceFlowerPose,
                audienceCheckpointPose,
                shootToScoringFlowerControlPose,
                parkControlPoseFromShoot,
                parkControlPoseFromTippedShoot,
                parkControlPoseFromScoringFlower,
                parkPose
        );
    }
}

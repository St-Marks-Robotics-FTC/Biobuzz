package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

/** Blue-alliance poses. See {@link AutoConfig} for what each pose means and the route that uses it. **/
@Configurable
public abstract class BlueAuto extends BozoAuto {
    // LAST MIRRORED: 11/2/26 (OUT-OF-DATE)

    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    public static Pose startPose = new Pose(84, 137.246, Math.toRadians(270));
    public static Pose shootPose = new Pose(84, 134, Math.toRadians(270));
    public static Pose intermediatePose = new Pose(122, 132.25, Math.toRadians(30));
    public static Pose refuelPose = new Pose(134, 137.246, Math.toRadians(0));

    // ---------------------------------------------------------------- SCORING SIDE (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(132, 132, Math.toRadians(270));
    public static Pose tippedControlPose2 = new Pose(126, 54, Math.toRadians(270));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(90));

    // ---------------------------------------------------------------- SCORING SIDE: the scoring flower
    public static Pose scoringFlowerIntermediatePose = new Pose(96, 24, Math.toRadians(270));
    public static Pose scoringFlowerPose = new Pose(96, 8, Math.toRadians(270));

    // ---------------------------------------------------------------- AUDIENCE SIDE: the audience flower + checkpoint
    public static Pose tippedShootToAudienceFlowerControlPose = new Pose(126, 54, Math.toRadians(0));
    public static Pose audienceFlowerIntermediatePose = new Pose(120, 96, Math.toRadians(0));
    public static Pose audienceFlowerPose = new Pose(136, 96, Math.toRadians(0));
    public static Pose audienceCheckpointPose = new Pose(144, 144, Math.toRadians(180)); // UNTUNED (mirror of Red's 0,0 placeholder)
    public static Pose audienceCheckpointToTippedShootControlPose1 = new Pose(132, 132, Math.toRadians(270)); // UNTUNED
    public static Pose audienceCheckpointToTippedShootControlPose2 = new Pose(126, 54, Math.toRadians(270)); // UNTUNED
    public static Pose shootToScoringFlowerControlPose1 = new Pose(132, 132, Math.toRadians(180));
    public static Pose shootToScoringFlowerControlPose2 = new Pose(132, 54, Math.toRadians(180));

    // ---------------------------------------------------------------- PARK
    public static Pose parkControlPoseFromShoot = new Pose(132, 132, Math.toRadians(180));
    public static Pose parkControlPoseFromTippedShoot = new Pose(120, 120, Math.toRadians(180));
    public static Pose parkControlPoseFromScoringFlower = new Pose(96, 24, Math.toRadians(180));
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
                tippedShootToAudienceFlowerControlPose,
                audienceFlowerIntermediatePose,
                audienceFlowerPose,
                audienceCheckpointPose,
                audienceCheckpointToTippedShootControlPose1,
                audienceCheckpointToTippedShootControlPose2,
                shootToScoringFlowerControlPose1,
                shootToScoringFlowerControlPose2,
                parkControlPoseFromShoot,
                parkControlPoseFromTippedShoot,
                parkControlPoseFromScoringFlower,
                parkPose
        );
    }
}

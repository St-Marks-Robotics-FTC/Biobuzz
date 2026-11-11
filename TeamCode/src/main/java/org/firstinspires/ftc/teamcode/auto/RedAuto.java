package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Red-alliance poses. See {@link AutoConfig} for what each pose means and the route that uses it. **/
@Configurable
@Autonomous(name = "RedAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedAuto extends BozoAuto {
    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    public static Pose startPose = new Pose(60, 6.754, Math.toRadians(90));
    public static Pose shootPose = new Pose(60, 10, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(22, 11.75, Math.toRadians(210));
    public static Pose refuelPose = new Pose(10, 6.754, Math.toRadians(180));

    // ---------------------------------------------------------------- SCORING SIDE (shootPose mirrored over y = 72)
    public static Pose tippedControlPose1 = new Pose(12, 12, Math.toRadians(90));
    public static Pose tippedControlPose2 = new Pose(12, 70, Math.toRadians(90));
    public static Pose tippedShootPose = new Pose(shootPose.x(), 144 - shootPose.y(), Math.toRadians(270));

    // ---------------------------------------------------------------- SCORING SIDE: the scoring flower
    public static Pose scoringFlowerIntermediatePose = new Pose(48, 120, Math.toRadians(90));
    public static Pose scoringFlowerPose = new Pose(48, 136, Math.toRadians(90));

    // ---------------------------------------------------------------- AUDIENCE SIDE: the audience flower + checkpoint
    public static Pose tippedShootToAudienceFlowerControlPose = new Pose(18, 80, Math.toRadians(180));
    public static Pose audienceFlowerIntermediatePose = new Pose(24, 48, Math.toRadians(180));
    public static Pose audienceFlowerPose = new Pose(8, 48, Math.toRadians(180));
    public static Pose audienceCheckpointPose = new Pose(46, 36, Math.toRadians(75));
    public static Pose audienceCheckpointToTippedShootControlPose1 = new Pose(12, 12, Math.toRadians(90));
    public static Pose audienceCheckpointToTippedShootControlPose2 = new Pose(12, 70, Math.toRadians(90));
    public static Pose shootToScoringFlowerControlPose1 = new Pose(12, 12, Math.toRadians(0));
    public static Pose shootToScoringFlowerControlPose2 = new Pose(12, 80, Math.toRadians(0));

    // ---------------------------------------------------------------- PARK
    public static Pose parkControlPoseFromShoot = new Pose(12, 12, Math.toRadians(0));
    public static Pose parkControlPoseFromTippedShoot = new Pose(24, 120, Math.toRadians(0));
    public static Pose parkControlPoseFromScoringFlower = new Pose(48, 120, Math.toRadians(0));
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

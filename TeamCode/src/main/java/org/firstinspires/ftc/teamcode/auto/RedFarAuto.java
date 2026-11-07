package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends BozoAuto {
    // BlueFarAuto rotated 180° about field center
    public static Pose startPose = new Pose(18.1208, 135.5646, Math.toRadians(270));
    public static Pose shootPose = new Pose(61.6592, 12, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(27.3, 6, Math.toRadians(192));
    public static Pose refuelPose = new Pose(8.6385, 6.93, Math.toRadians(192));
    public static Pose parkControlPoseFromShoot = new Pose(11.7462, 42.5577, Math.toRadians(0));
    public static Pose parkPose = new Pose(13.6038, 109.5377, Math.toRadians(0));

    @Override
    protected boolean isBlueTeam() { return false; }

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                RedAuto.tippedControlPose1,
                RedAuto.tippedControlPose2,
                RedAuto.tippedShootPose,
                RedAuto.scoringFlowerIntermediatePose,
                RedAuto.scoringFlowerPose,
                RedAuto.tippedShootToAudienceFlowerControlPose,
                RedAuto.audienceFlowerIntermediatePose,
                RedAuto.audienceFlowerPose,
                RedAuto.audienceCheckpointPose,
                RedAuto.audienceCheckpointToTippedShootControlPose1,
                RedAuto.audienceCheckpointToTippedShootControlPose2,
                RedAuto.shootToScoringFlowerControlPose1,
                RedAuto.shootToScoringFlowerControlPose2,
                parkControlPoseFromShoot,
                RedAuto.parkControlPoseFromTippedShoot,
                RedAuto.parkControlPoseFromScoringFlower,
                parkPose
        );
    }
}

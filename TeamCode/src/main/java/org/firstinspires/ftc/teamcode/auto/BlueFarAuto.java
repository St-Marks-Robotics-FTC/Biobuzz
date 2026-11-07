package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends BozoAuto {
    public static Pose startPose = new Pose(125.8792, 8.4354, Math.toRadians(90));
    public static Pose shootPose = new Pose(82.3408, 132, Math.toRadians(270));
    public static Pose intermediatePose = new Pose(116.7, 138, Math.toRadians(12));
    public static Pose refuelPose = new Pose(135.3615, 137.07, Math.toRadians(12));
    public static Pose parkControlPoseFromShoot = new Pose(132.2538, 101.4423, Math.toRadians(180));
    public static Pose parkPose = new Pose(130.3962, 34.4623, Math.toRadians(180));

    @Override
    protected boolean isBlueTeam() { return true; }

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPose,
                intermediatePose,
                refuelPose,
                BlueAuto.tippedControlPose1,
                BlueAuto.tippedControlPose2,
                BlueAuto.tippedShootPose,
                BlueAuto.scoringFlowerIntermediatePose,
                BlueAuto.scoringFlowerPose,
                BlueAuto.tippedShootToAudienceFlowerControlPose,
                BlueAuto.audienceFlowerIntermediatePose,
                BlueAuto.audienceFlowerPose,
                BlueAuto.audienceCheckpointPose,
                BlueAuto.audienceCheckpointToTippedShootControlPose1,
                BlueAuto.audienceCheckpointToTippedShootControlPose2,
                BlueAuto.shootToScoringFlowerControlPose1,
                BlueAuto.shootToScoringFlowerControlPose2,
                parkControlPoseFromShoot,
                BlueAuto.parkControlPoseFromTippedShoot,
                BlueAuto.parkControlPoseFromScoringFlower,
                parkPose
        );
    }
}

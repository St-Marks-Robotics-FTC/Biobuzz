package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    /** where the robot is placed at init **/
    public final Pose startPose;
    public final Pose shootPose;
    /** waypoint between {@link #shootPose} and {@link #refuelPose} **/
    public final Pose intermediatePose;
    public final Pose refuelPose;

    // ---------------------------------------------------------------- SCORING SIDE
    public final Pose tippedControlPose1;
    public final Pose tippedControlPose2;
    /** scoring-side shoot pose **/
    public final Pose tippedShootPose;
    public final Pose scoringFlowerIntermediatePose;
    public final Pose scoringFlowerPose;

    // ---------------------------------------------------------------- AUDIENCE SIDE
    public final Pose tippedShootToAudienceFlowerControlPose;
    /** waypoint in front of the audience flower **/
    public final Pose audienceFlowerIntermediatePose;
    public final Pose audienceFlowerPose;
    public final Pose audienceCheckpointPose;
    public final Pose audienceCheckpointToTippedShootControlPose1;
    public final Pose audienceCheckpointToTippedShootControlPose2;
    public final Pose shootToScoringFlowerControlPose1;
    public final Pose shootToScoringFlowerControlPose2;

    // ---------------------------------------------------------------- PARK
    public final Pose parkControlPoseFromShoot;
    public final Pose parkControlPoseFromTippedShoot;
    public final Pose parkControlPoseFromScoringFlower;
    public final Pose parkPose;

    public AutoConfig(
            Pose startPose,
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose,
            Pose tippedControlPose1,
            Pose tippedControlPose2,
            Pose tippedShootPose,
            Pose scoringFlowerIntermediatePose,
            Pose scoringFlowerPose,
            Pose tippedShootToAudienceFlowerControlPose,
            Pose audienceFlowerIntermediatePose,
            Pose audienceFlowerPose,
            Pose audienceCheckpointPose,
            Pose audienceCheckpointToTippedShootControlPose1,
            Pose audienceCheckpointToTippedShootControlPose2,
            Pose shootToScoringFlowerControlPose1,
            Pose shootToScoringFlowerControlPose2,
            Pose parkControlPoseFromShoot,
            Pose parkControlPoseFromTippedShoot,
            Pose parkControlPoseFromScoringFlower,
            Pose parkPose
    ) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
        this.tippedControlPose1 = tippedControlPose1;
        this.tippedControlPose2 = tippedControlPose2;
        this.tippedShootPose = tippedShootPose;
        this.scoringFlowerIntermediatePose = scoringFlowerIntermediatePose;
        this.scoringFlowerPose = scoringFlowerPose;
        this.tippedShootToAudienceFlowerControlPose = tippedShootToAudienceFlowerControlPose;
        this.audienceFlowerIntermediatePose = audienceFlowerIntermediatePose;
        this.audienceFlowerPose = audienceFlowerPose;
        this.audienceCheckpointPose = audienceCheckpointPose;
        this.audienceCheckpointToTippedShootControlPose1 = audienceCheckpointToTippedShootControlPose1;
        this.audienceCheckpointToTippedShootControlPose2 = audienceCheckpointToTippedShootControlPose2;
        this.shootToScoringFlowerControlPose1 = shootToScoringFlowerControlPose1;
        this.shootToScoringFlowerControlPose2 = shootToScoringFlowerControlPose2;
        this.parkControlPoseFromShoot = parkControlPoseFromShoot;
        this.parkControlPoseFromTippedShoot = parkControlPoseFromTippedShoot;
        this.parkControlPoseFromScoringFlower = parkControlPoseFromScoringFlower;
        this.parkPose = parkPose;
    }
}

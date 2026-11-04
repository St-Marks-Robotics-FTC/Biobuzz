package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * Every pose the autonomous drives through, grouped by where in the auto it is used. Headings of control poses are
 * ignored (only x/y shape the curve).
 *
 * <h3>Vocabulary</h3>
 * <ul>
 *   <li><b>audience side</b>: where {@link #shootPose} is; the side we start on.</li>
 *   <li><b>scoring side</b>: the far side, at {@link #tippedShootPose} (audience shoot pose mirrored over y = 72).</li>
 *   <li><b>scoring flower</b>: the one flower on the scoring side ({@link #scoringFlowerPose}).</li>
 *   <li><b>audience flower</b>: the one flower on the audience side ({@link #audienceFlowerPose}).</li>
 *   <li><b>intermediate pose</b>: a waypoint in front of a flower; we drive to it at full speed, then creep into the
 *       flower at reduced speed, and back out to it again.</li>
 *   <li><b>checkpoint pose</b>: a spot on the audience side with a good view for the Limelight to judge the hive after
 *       the audience flower; the result picks the sub-case (1A/1B, 2A/2B).</li>
 * </ul>
 * There are only two physical flowers. "first" and "second" flower in the route below only describe the order within
 * one case; they are not names for a particular flower.
 *
 * <h3>Route</h3>
 * <pre>
 * SHARED   start -> shoot -> intermediate -> refuel -> shoot, then look at the hive:
 *
 * CASE1  (hive tipped)       shoot -> tippedShoot, then
 *                            scoringFlower -> tippedShoot -> audienceFlower -> audienceCheckpoint
 *    CASE1A (made all 8)     audienceCheckpoint -> shoot -> park
 *    CASE1B (missed some)    audienceCheckpoint -> tippedShoot -> park
 *
 * CASE2  (hive not tipped)   shoot again, then
 *                            audienceFlower -> audienceCheckpoint
 *    CASE2A (tipped now)     audienceCheckpoint -> tippedShoot -> scoringFlower -> tippedShoot -> park
 *    CASE2B (still not)      audienceCheckpoint -> shoot -> scoringFlower -> park
 * </pre>
 */
public class AutoConfig {
    // ---------------------------------------------------------------- SHARED (before the hive is judged)
    /** where the robot is placed at init **/
    public final Pose startPose;
    /** audience-side shoot pose; also the "original" shoot pose that CASE1A/CASE2B return to **/
    public final Pose shootPose;
    /** waypoint between {@link #shootPose} and {@link #refuelPose} **/
    public final Pose intermediatePose;
    /** where we pick up balls before the hive is judged **/
    public final Pose refuelPose;

    // ---------------------------------------------------------------- SCORING SIDE
    /** first control point of the cubic curve {@link #shootPose} -> {@link #tippedShootPose} **/
    public final Pose tippedControlPose1;
    /** second control point of the cubic curve {@link #shootPose} -> {@link #tippedShootPose} **/
    public final Pose tippedControlPose2;
    /** scoring-side shoot pose **/
    public final Pose tippedShootPose;
    /** waypoint in front of the scoring flower (reached by a line from {@link #tippedShootPose}) **/
    public final Pose scoringFlowerIntermediatePose;
    public final Pose scoringFlowerPose;

    // ---------------------------------------------------------------- AUDIENCE SIDE
    /** control point of the curve {@link #tippedShootPose} -> {@link #audienceFlowerIntermediatePose} (CASE1) **/
    public final Pose tippedShootToAudienceFlowerControlPose;
    /** waypoint in front of the audience flower **/
    public final Pose audienceFlowerIntermediatePose;
    public final Pose audienceFlowerPose;
    /** where CASE1 and CASE2 stop to let the Limelight judge the hive after the audience flower (decides 1A vs 1B, 2A vs 2B) **/
    public final Pose audienceCheckpointPose;
    /** first control point of the cubic curve {@link #audienceCheckpointPose} -> {@link #tippedShootPose} (CASE1B and CASE2A) **/
    public final Pose audienceCheckpointToTippedShootControlPose1;
    /** second control point of the cubic curve {@link #audienceCheckpointPose} -> {@link #tippedShootPose} (CASE1B and CASE2A) **/
    public final Pose audienceCheckpointToTippedShootControlPose2;
    /** first control point of the cubic curve {@link #shootPose} -> {@link #scoringFlowerIntermediatePose} (CASE2B) **/
    public final Pose shootToScoringFlowerControlPose1;
    /** second control point of the cubic curve {@link #shootPose} -> {@link #scoringFlowerIntermediatePose} (CASE2B) **/
    public final Pose shootToScoringFlowerControlPose2;

    // ---------------------------------------------------------------- PARK
    /** control point of the curve {@link #shootPose} -> {@link #parkPose} (CASE1A) **/
    public final Pose parkControlPoseFromShoot;
    /** control point of the curve {@link #tippedShootPose} -> {@link #parkPose} (CASE1B and CASE2A) **/
    public final Pose parkControlPoseFromTippedShoot;
    /** control point of the curve {@link #scoringFlowerIntermediatePose} -> {@link #parkPose} (CASE2B) **/
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

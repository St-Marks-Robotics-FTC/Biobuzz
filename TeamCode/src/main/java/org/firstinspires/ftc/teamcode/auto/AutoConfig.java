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
 *   <li><b>intermediate pose</b>: a waypoint in front of a flower; we drive to it at full speed, then creep into the
 *       flower at reduced speed, and back out to it again.</li>
 *   <li><b>checkpoint pose</b>: a spot with a good view for the Limelight to judge the hive; the result picks the
 *       sub-case (1A/1B, 2A/2B).</li>
 * </ul>
 *
 * <h3>Route</h3>
 * <pre>
 * SHARED   start -> shoot -> intermediate -> refuel -> shoot, then look at the hive:
 *
 * CASE1  (hive tipped)       shoot -> tippedShoot, then
 *                            flower1 -> tippedShoot -> flower2 -> checkpoint
 *    CASE1A (made all 8)     checkpoint -> shoot -> park
 *    CASE1B (missed some)    checkpoint -> tippedShoot
 *
 * CASE2  (hive not tipped)   shoot again, then
 *                            audienceFlower1 -> audienceCheckpoint
 *    CASE2A (tipped now)     audienceCheckpoint -> tippedShoot -> flower2 -> tippedShoot -> park
 *    CASE2B (still not)      audienceCheckpoint -> shoot -> audienceFlower2 -> park
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

    // ---------------------------------------------------------------- CASE1 / CASE2A: scoring-side flowers
    /** waypoint in front of flower 1 (reached by a line from {@link #tippedShootPose}) **/
    public final Pose flowerIntermediatePose;
    public final Pose flowerPose;
    /** control point of the curve {@link #tippedShootPose} -> {@link #flower2IntermediatePose} **/
    public final Pose flower2ControlPose;
    /** waypoint in front of flower 2 **/
    public final Pose flower2IntermediatePose;
    public final Pose flower2Pose;

    // ---------------------------------------------------------------- CASE1: Limelight checkpoint
    /** where CASE1 stops to let the Limelight judge the hive after flower 2 (decides 1A vs 1B) **/
    public final Pose checkpointPose;

    // ---------------------------------------------------------------- CASE2: audience-side flowers
    /** waypoint in front of the audience-side flower 1 (reached by a line from {@link #shootPose}) **/
    public final Pose audienceFlowerIntermediatePose;
    public final Pose audienceFlowerPose;
    /** where CASE2 stops to let the Limelight judge the hive after flower 1 (decides 2A vs 2B) **/
    public final Pose audienceCheckpointPose;
    /** control point of the curve {@link #shootPose} -> {@link #audienceFlower2IntermediatePose} (CASE2B) **/
    public final Pose audienceFlower2ControlPose;
    /** waypoint in front of the audience-side flower 2 (CASE2B) **/
    public final Pose audienceFlower2IntermediatePose;
    public final Pose audienceFlower2Pose;

    // ---------------------------------------------------------------- PARK
    /** control point of the curve to {@link #parkPose} **/
    public final Pose parkControlPose;
    public final Pose parkPose;

    public AutoConfig(
            Pose startPose,
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose,
            Pose tippedControlPose1,
            Pose tippedControlPose2,
            Pose tippedShootPose,
            Pose flowerIntermediatePose,
            Pose flowerPose,
            Pose flower2ControlPose,
            Pose flower2IntermediatePose,
            Pose flower2Pose,
            Pose checkpointPose,
            Pose audienceFlowerIntermediatePose,
            Pose audienceFlowerPose,
            Pose audienceCheckpointPose,
            Pose audienceFlower2ControlPose,
            Pose audienceFlower2IntermediatePose,
            Pose audienceFlower2Pose,
            Pose parkControlPose,
            Pose parkPose
    ) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
        this.tippedControlPose1 = tippedControlPose1;
        this.tippedControlPose2 = tippedControlPose2;
        this.tippedShootPose = tippedShootPose;
        this.flowerIntermediatePose = flowerIntermediatePose;
        this.flowerPose = flowerPose;
        this.flower2ControlPose = flower2ControlPose;
        this.flower2IntermediatePose = flower2IntermediatePose;
        this.flower2Pose = flower2Pose;
        this.checkpointPose = checkpointPose;
        this.audienceFlowerIntermediatePose = audienceFlowerIntermediatePose;
        this.audienceFlowerPose = audienceFlowerPose;
        this.audienceCheckpointPose = audienceCheckpointPose;
        this.audienceFlower2ControlPose = audienceFlower2ControlPose;
        this.audienceFlower2IntermediatePose = audienceFlower2IntermediatePose;
        this.audienceFlower2Pose = audienceFlower2Pose;
        this.parkControlPose = parkControlPose;
        this.parkPose = parkPose;
    }
}

package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.auto.paths.PathHelpers.crossFieldHeading;
import static org.firstinspires.ftc.teamcode.auto.paths.PathHelpers.limitSpeed;
import static org.firstinspires.ftc.teamcode.auto.paths.PathHelpers.plainLine;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.auto.AutoConfig;

/**
 * Every path of the hive autonomous, built once from an {@link AutoConfig}. Built up front so a bad pose fails at init,
 * not mid-match.
 *
 * <p>Naming: {@code <from>To<to>}. "Intermediate" is the waypoint in front of a flower; "tippedShoot" is the
 * scoring-side shoot pose; "shoot" is the original (audience-side) shoot pose; there are only two physical flowers,
 * {@link #scoringFlower} and {@link #audienceFlower}. For {@code curve(start, control..., end)} the control points
 * are not driven through, they only pull the path toward them.
 *
 * <p>A path is reused wherever the same move appears in several cases (e.g. {@link #checkpointToTippedShoot} serves
 * both CASE1B and CASE2A); only one case runs per match, so no path is ever driven twice.
 */
public final class AutoPaths {
    /** heading holds the intermediate pose's heading until this fraction of the refuel path, then rotates to the refuel pose **/
    private static final double REFUEL_ROTATE_START = 0.7;

    public final FlowerSite scoringFlower;
    public final FlowerSite audienceFlower;

    // ---- SHARED: start -> shoot -> intermediate -> refuel -> shoot
    public final Path startToShoot, shootToIntermediate, intermediateToRefuel, refuelToShoot;

    // ---- toward the scoring side
    public final Path shootToTippedShoot;
    public final Path tippedShootToScoringFlowerIntermediate, scoringFlowerIntermediateToTippedShoot;
    public final Path shootToScoringFlowerIntermediate;

    // ---- toward the audience side
    public final Path tippedShootToAudienceFlowerIntermediate, shootToAudienceFlowerIntermediate;
    public final Path audienceFlowerIntermediateToCheckpoint;
    public final Path checkpointToShoot, checkpointToTippedShoot;

    // ---- park
    public final Path shootToPark, tippedShootToPark, scoringFlowerIntermediateToPark;

    public AutoPaths(AutoConfig c) {
        Pose shoot = c.shootPose;
        scoringFlower = new FlowerSite("scoring flower", c.scoringFlowerIntermediatePose, c.scoringFlowerPose);
        audienceFlower = new FlowerSite("audience flower", c.audienceFlowerIntermediatePose, c.audienceFlowerPose);

        // ---------------------------------------------------------------- SHARED
        startToShoot = plainLine(c.startPose, shoot);
        shootToIntermediate = plainLine(shoot, c.intermediatePose);
        intermediateToRefuel = limitSpeed(
                line(c.intermediatePose, c.refuelPose).heading(Interpolator.piecewise()
                        .until(REFUEL_ROTATE_START, Interpolator.constant(c.intermediatePose))
                        .until(1.0, Interpolator.linear(c.intermediatePose, c.refuelPose))),
                Tunables.flowerIntakeSpeed);
        refuelToShoot = plainLine(c.refuelPose, shoot);

        // ---------------------------------------------------------------- toward the scoring side
        // shoot -> scoring-side shoot pose: cubic curve bent by the two tipped control poses
        shootToTippedShoot = crossFieldHeading(
                curve(shoot, c.tippedControlPose1, c.tippedControlPose2, c.tippedShootPose),
                shoot, c.tippedShootPose);
        tippedShootToScoringFlowerIntermediate = plainLine(c.tippedShootPose, c.scoringFlowerIntermediatePose);
        scoringFlowerIntermediateToTippedShoot = plainLine(c.scoringFlowerIntermediatePose, c.tippedShootPose);
        // shoot -> scoring flower directly: cubic curve (2 control points)
        shootToScoringFlowerIntermediate = crossFieldHeading(
                curve(shoot, c.shootToScoringFlowerControlPose1, c.shootToScoringFlowerControlPose2, c.scoringFlowerIntermediatePose),
                shoot, c.scoringFlowerIntermediatePose);

        // ---------------------------------------------------------------- toward the audience side
        // tippedShoot -> audience flower: curve (1 control point)
        tippedShootToAudienceFlowerIntermediate = crossFieldHeading(
                curve(c.tippedShootPose, c.tippedShootToAudienceFlowerControlPose, c.audienceFlowerIntermediatePose),
                c.tippedShootPose, c.audienceFlowerIntermediatePose);
        shootToAudienceFlowerIntermediate = plainLine(shoot, c.audienceFlowerIntermediatePose);
        audienceFlowerIntermediateToCheckpoint = plainLine(c.audienceFlowerIntermediatePose, c.audienceCheckpointPose);
        checkpointToShoot = plainLine(c.audienceCheckpointPose, shoot);
        // checkpoint -> tippedShoot: cubic curve (2 control points)
        checkpointToTippedShoot = crossFieldHeading(
                curve(c.audienceCheckpointPose, c.audienceCheckpointToTippedShootControlPose1, c.audienceCheckpointToTippedShootControlPose2, c.tippedShootPose),
                c.audienceCheckpointPose, c.tippedShootPose);

        // ---------------------------------------------------------------- park (each a curve with 1 control point)
        shootToPark = crossFieldHeading(
                curve(shoot, c.parkControlPoseFromShoot, c.parkPose),
                shoot, c.parkPose);
        tippedShootToPark = curve(c.tippedShootPose, c.parkControlPoseFromTippedShoot, c.parkPose).tangent();
        scoringFlowerIntermediateToPark = curve(c.scoringFlowerIntermediatePose, c.parkControlPoseFromScoringFlower, c.parkPose)
                .linear(c.scoringFlowerIntermediatePose, c.parkPose);
    }
}

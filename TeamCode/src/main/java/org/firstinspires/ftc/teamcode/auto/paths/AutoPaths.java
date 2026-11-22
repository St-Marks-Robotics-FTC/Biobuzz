package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.BozoAuto;
import org.firstinspires.ftc.teamcode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

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

    /**
     * One physical flower: the waypoint in front of it, the flower itself, and the two short paths between them. We
     * reach the intermediate pose at full speed by a path that differs per call site, then {@link #creepIn} slowly and
     * {@link #backOut} again; those two never differ, so they are built here once.
     */
    public static final class FlowerSite {
        public final String name;
        public final Pose intermediatePose;
        public final Pose flowerPose;
        /** intermediate pose -> flower at {@link Tunables#flowerApproachSpeed} **/
        public final Path creepIn;
        /** flower -> intermediate pose at {@link Tunables#flowerIntakeSpeed} **/
        public final Path backOut;

        /** @param name used in telemetry and the route plan, e.g. "scoring flower" **/
        FlowerSite(String name, Pose intermediatePose, Pose flowerPose) {
            this.name = name;
            this.intermediatePose = intermediatePose;
            this.flowerPose = flowerPose;
            this.creepIn = limitSpeed(plainLine(intermediatePose, flowerPose), Tunables.flowerApproachSpeed);
            this.backOut = limitSpeed(plainLine(flowerPose, intermediatePose), Tunables.flowerIntakeSpeed);
        }
    }

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
                BozoAuto.FLOWER_INTAKE_SPEED);
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

    // ======================================================================== helpers

    /** a straight line {@code from} -> {@code to}, heading turning linearly between them **/
    private static Path plainLine(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    /** {@code path} capped to {@code speed} (fraction 0-1 of max path speed) for this path only **/
    private static Path limitSpeed(Path path, double speed) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(speed));
    }

    /**
     * Heading override for a path that crosses the field midline: face 90 deg (+y) when driving toward +y, 270 deg
     * (-y) when driving toward -y, hold it until {@link Tunables#crossFieldHoldUntil}, then turn to {@code to}'s heading.
     * Red audience->scoring is +y so 90; Blue is the point reflection so its audience->scoring is -y so 270, which
     * matches the mirror rule with no per-alliance code.
     * Precondition: {@code from.y() != to.y()}.
     */
    private static Path crossFieldHeading(Path path, Pose from, Pose to) {
        double hold = Math.toRadians(to.y() > from.y() ? 90 : 270);
        Pose holdPose = new Pose(from.x(), from.y(), hold);
        return path.heading(Interpolator.piecewise()
                .until(Tunables.crossFieldHoldUntil, Interpolator.constant(holdPose))
                .until(1.0, Interpolator.linear(holdPose, to)));
    }
}

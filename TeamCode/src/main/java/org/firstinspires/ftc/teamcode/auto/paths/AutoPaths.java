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
 * Every path of the hive auto, built once from an {@link AutoConfig} so a bad pose fails at init.
 * Naming is {@code <from>To<to>}; "intermediate" is the waypoint in front of a flower, "tippedShoot" the scoring-side
 * shoot pose. Paths are shared between cases where the move is identical.
 */
public final class AutoPaths {
    /** fraction of the refuel path spent holding the intermediate heading before turning **/
    private static final double REFUEL_ROTATE_START = 0.7;

    /** One physical flower: waypoint in front of it, the flower, and the creep-in / back-out paths between them. **/
    public static final class FlowerSite {
        public final String name;
        public final Pose intermediatePose;
        public final Pose flowerPose;
        /** intermediate pose -> flower at {@link BozoAuto#FLOWER_APPROACH_SPEED} **/
        public final Path creepIn;
        /** flower -> intermediate pose at {@link BozoAuto#FLOWER_INTAKE_SPEED} **/
        public final Path backOut;

        FlowerSite(String name, Pose intermediatePose, Pose flowerPose) {
            this.name = name;
            this.intermediatePose = intermediatePose;
            this.flowerPose = flowerPose;
            this.creepIn = limitSpeed(plainLine(intermediatePose, flowerPose), BozoAuto.FLOWER_APPROACH_SPEED);
            this.backOut = limitSpeed(plainLine(flowerPose, intermediatePose), BozoAuto.FLOWER_INTAKE_SPEED);
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

    /** straight line with heading turning linearly **/
    private static Path plainLine(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    /** caps this path to {@code speed} (0-1 of max) **/
    private static Path limitSpeed(Path path, double speed) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(speed));
    }

    /**
     * Heading override for a cross-field path: keep {@code from}'s heading until {@link BozoAuto#CROSS_FIELD_HOLD_FROM},
     * turn to 90 deg (driving +y) or 270 deg (-y) by {@link BozoAuto#CROSS_FIELD_TURN_DONE}, hold it until
     * {@link BozoAuto#CROSS_FIELD_HOLD_UNTIL}, then turn to {@code to}'s heading by t=1.
     * Precondition: {@code from.y() != to.y()} and 0 < HoldFrom < TurnDone < HoldUntil < 1.
     * @throws IllegalStateException if the tunable fractions are not strictly increasing
     **/
    private static Path crossFieldHeading(Path path, Pose from, Pose to) {
        if (!(0 < BozoAuto.CROSS_FIELD_HOLD_FROM
                && BozoAuto.CROSS_FIELD_HOLD_FROM < BozoAuto.CROSS_FIELD_TURN_DONE
                && BozoAuto.CROSS_FIELD_TURN_DONE < BozoAuto.CROSS_FIELD_HOLD_UNTIL
                && BozoAuto.CROSS_FIELD_HOLD_UNTIL < 1)) {
            throw new IllegalStateException("cross-field tunables must satisfy 0 < HoldFrom < TurnDone < HoldUntil < 1, got "
                    + BozoAuto.CROSS_FIELD_HOLD_FROM + ", " + BozoAuto.CROSS_FIELD_TURN_DONE + ", " + BozoAuto.CROSS_FIELD_HOLD_UNTIL);
        }
        Pose startingHeading = new Pose(from.x(), from.y(), from.heading());

        double hold = Math.toRadians(to.y() > from.y() ? 90 : 270); // TEMP: we go backwards to avoid picking up nectar
        Pose holdPose = new Pose(from.x(), from.y(), hold);
        return path.heading(Interpolator.piecewise()
                .until(BozoAuto.CROSS_FIELD_HOLD_FROM, Interpolator.constant(startingHeading))
                .until(BozoAuto.CROSS_FIELD_TURN_DONE, Interpolator.linear(startingHeading, holdPose))
                .until(BozoAuto.CROSS_FIELD_HOLD_UNTIL, Interpolator.constant(holdPose))
                .until(1.0, Interpolator.linear(holdPose, to)));
    }
}

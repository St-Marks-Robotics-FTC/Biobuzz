package org.firstinspires.ftc.teamcode.auto.routes;

import static org.firstinspires.ftc.teamcode.auto.core.Steps.branch;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.sequence;

import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.auto.steps.HiveView;
import org.firstinspires.ftc.teamcode.auto.steps.RobotSteps;

/**
 * The hive autonomous as data: what to do, in order, with every decision spelled out as a {@code branch}. Poses are
 * documented in {@link org.firstinspires.ftc.teamcode.auto.AutoConfig}.
 *
 * <pre>
 * SHARED   start -> shoot -> intermediate -> refuel -> shoot -> (look at the hive)
 *
 * There are only two physical flowers: the scoring flower (scoring side) and the audience flower (audience side).
 *
 * tags NOT visible (hive TIPPED) -> CASE1: tippedShoot, scoring flower, tippedShoot, audience flower, checkpoint
 *     tags visible    -> CASE1A: original shoot pose, shoot, park
 *     tags not visible -> CASE1B: tippedShoot, shoot, park
 *
 * tags visible (hive NOT tipped) -> CASE2: shoot again, audience flower, checkpoint
 *     tags not visible -> CASE2A: tippedShoot, shoot, scoring flower, tippedShoot, shoot, park
 *     tags visible     -> CASE2B: original shoot pose, shoot, scoring flower, park
 * </pre>
 * Every "shoot" waits for the flywheel and fires. The checkpoint is where the Limelight judges the hive again.
 */
public final class HiveRoute {
    private HiveRoute() {}

    /** @return the whole autonomous, ready to hand to a {@link org.firstinspires.ftc.teamcode.auto.core.Sequencer} **/
    public static Step build(RobotSteps s, AutoPaths p, HiveView hive) {
        // ---- fragments used by more than one case; defined once so the cases cannot drift apart
        // at the tipped shoot pose: collect the scoring flower, come back, shoot (CASE1 and CASE2A)
        Step scoringFlowerLap = sequence("scoring flower lap",
                s.collectFlower(p.scoringFlower, p.tippedShootToScoringFlowerIntermediate),
                s.travelShoot("back to tipped shoot", p.scoringFlowerIntermediateToTippedShoot));
        Step toCheckpoint = s.drive("to checkpoint", p.audienceFlowerIntermediateToCheckpoint);

        // ---- CASE1: hive tipped first try
        Step case1A = sequence("CASE1A",
                s.travelShoot("to shoot", p.checkpointToShoot),
                s.park(p.shootToPark));
        Step case1B = sequence("CASE1B",
                s.travelShoot("to tipped shoot", p.checkpointToTippedShoot),
                s.park(p.tippedShootToPark));
        Step case1 = sequence("CASE1",
                s.travelShoot("to tipped shoot", p.shootToTippedShoot),
                scoringFlowerLap,
                s.collectFlower(p.audienceFlower, p.tippedShootToAudienceFlowerIntermediate),
                toCheckpoint,
                branch("tags visible", hive::tagsVisible, case1A, case1B));

        // ---- CASE2: hive did not tip first try; we are still on the shoot pose
        Step case2A = sequence("CASE2A",
                s.travelShoot("to tipped shoot", p.checkpointToTippedShoot),
                scoringFlowerLap,
                s.park(p.tippedShootToPark));
        Step case2B = sequence("CASE2B",
                s.travelShoot("to shoot", p.checkpointToShoot),
                s.collectFlower(p.scoringFlower, p.shootToScoringFlowerIntermediate),
                s.park(p.scoringFlowerIntermediateToPark));
        Step case2 = sequence("CASE2",
                s.shoot(),
                s.collectFlower(p.audienceFlower, p.shootToAudienceFlowerIntermediate),
                toCheckpoint,
                branch("tags visible", hive::tagsVisible, case2B, case2A));

        return sequence("hive auto",
                s.spinUpFlywheel(), // spin up while driving to the first shot
                s.travelShoot("first shoot", p.startToShoot),
                s.drive("to intermediate", p.shootToIntermediate),
                s.refuel(p.intermediateToRefuel),
                s.drive("refuel to shoot", p.refuelToShoot),
                s.flowerDown(),
                branch("tags visible", hive::tagsVisible, case2, case1));
    }
}

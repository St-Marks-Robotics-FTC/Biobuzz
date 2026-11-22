package org.firstinspires.ftc.teamcode.auto.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.branch;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.instant;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.parallel;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.timeout;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.waitMillis;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.waitUntil;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

public class StepsTest {
    private final List<String> log = new ArrayList<>();
    private final AtomicLong now = new AtomicLong();

    private Step log(String what) {
        return instant(what, () -> log.add(what));
    }

    /** a step that records start/update calls and finishes after {@code loops} updates **/
    private final class Counting implements Step {
        final String name; final int loops; int updates; int starts;
        Counting(String name, int loops) { this.name = name; this.loops = loops; }
        @Override public String label() { return name; }
        @Override public void start() { starts++; updates = 0; log.add("start " + name); }
        @Override public boolean update() { log.add("update " + name); return ++updates >= loops; }
    }

    private Sequencer run(Step root) {
        Sequencer s = new Sequencer(Collections.singletonList(root));
        s.start();
        return s;
    }

    // If this fails: a step is updated in the same loop that started it, so a drive step sees the PREVIOUS path's
    // isBusy()==false and finishes instantly
    @Test
    public void stepIsNeverUpdatedInTheLoopThatStartedIt() {
        Sequencer s = run(sequence(new Counting("a", 1), new Counting("b", 1)));
        s.update();
        assertEquals(Collections.singletonList("start a"), log);
        s.update();
        assertEquals("[start a, update a, start b]", log.toString());
    }

    // If this fails: every instant action costs a whole loop (20 ms) and a flower lap with 3 instants is 60 ms slower
    @Test
    public void instantsRunInTheSameLoopAsThePrecedingStepFinishing() {
        Sequencer s = run(sequence(new Counting("a", 1), log("x"), log("y"), new Counting("b", 1)));
        s.update();
        s.update();
        assertEquals("[start a, update a, x, y, start b]", log.toString());
    }

    // If this fails: a sequence that begins with instants (or a branch arm that does) waits a loop before acting
    @Test
    public void nestedSequenceRunsItsLeadingInstantsWhenStarted() {
        Sequencer s = run(sequence(sequence(log("x"), new Counting("a", 1))));
        s.update();
        assertEquals("[x, start a]", log.toString());
    }

    // If this fails: a sequencer ends early or skips a step
    @Test
    public void sequenceRunsEveryStepInOrderThenFinishes() {
        Sequencer s = run(sequence(log("1"), sequence(log("2"), log("3")), log("4")));
        assertEquals(Sequencer.Status.FINISHED, s.update());
        assertEquals("[1, 2, 3, 4]", log.toString());
    }

    // If this fails: an empty route reports RUNNING forever and the OpMode never stops
    @Test
    public void emptySequenceFinishesImmediately() {
        assertEquals(Sequencer.Status.FINISHED, run(sequence()).update());
    }

    // If this fails: the branch decides when the route is BUILT (frozen at init) instead of when it is reached
    @Test
    public void branchEvaluatesItsConditionWhenReachedNotWhenBuilt() {
        AtomicBoolean flag = new AtomicBoolean(true);
        Step root = sequence(new Counting("wait", 1), branch("flag", flag::get, log("yes"), log("no")));
        Sequencer s2 = run(root);
        s2.update();
        flag.set(false); // changes while "wait" is running; must be seen at branch time
        s2.update();
        assertEquals("[start wait, update wait, no]", log.toString());
        assertEquals(Sequencer.Status.FINISHED, s2.update());
    }

    // If this fails: a re-run of the same branch keeps the previous run's arm
    @Test
    public void branchReevaluatesOnEveryRun() {
        AtomicBoolean flag = new AtomicBoolean(true);
        Step b = branch("flag", flag::get, log("yes"), log("no"));
        Sequencer s = run(b);
        s.update();
        flag.set(false);
        s.start();
        s.update();
        assertEquals("[yes, no]", log.toString());
    }

    // If this fails: a shared step (scoring flower lap used by two cases) keeps state between runs
    @Test
    public void restartingResetsStepState() {
        Counting c = new Counting("a", 2);
        Sequencer s = run(sequence(c));
        s.update(); s.update(); s.update();
        assertEquals(Sequencer.Status.FINISHED, s.status());
        s.start();
        assertEquals(Sequencer.Status.RUNNING, s.update());
        assertEquals(2, c.starts);
        assertEquals(0, c.updates);
    }

    // If this fails: the 30 s cutoff cannot stop the routine and the robot keeps starting paths
    @Test
    public void abortStopsAllFurtherSteps() {
        Sequencer s = run(sequence(new Counting("a", 5), log("never")));
        s.update();
        s.abort();
        assertEquals(Sequencer.Status.ABORTED, s.update());
        assertFalse(log.contains("never"));
        assertEquals("", s.currentName());
    }

    // If this fails: forgetting start() silently does nothing instead of crashing
    @Test
    public void updateBeforeStartThrows() {
        try {
            new Sequencer(Collections.singletonList(log("x"))).update();
            fail("expected IllegalStateException");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("start"));
        }
    }

    // If this fails: parallel finishes when the first child does, or never finishes
    @Test
    public void parallelFinishesWhenTheLastChildFinishes() {
        Sequencer s = run(parallel("p", new Counting("a", 1), new Counting("b", 3)));
        s.update();
        assertEquals(Sequencer.Status.RUNNING, s.update());
        assertEquals(Sequencer.Status.RUNNING, s.update());
        assertEquals(Sequencer.Status.FINISHED, s.update());
    }

    // If this fails: a finished parallel child keeps being updated (would re-issue its hardware command)
    @Test
    public void parallelDoesNotUpdateAFinishedChild() {
        Counting a = new Counting("a", 1);
        Sequencer s = run(parallel("p", a, new Counting("b", 3)));
        s.update(); s.update(); s.update(); s.update();
        assertEquals(1, a.updates);
    }

    // If this fails: waits ignore the clock, or off-by-one at the exact deadline
    @Test
    public void waitMillisFinishesExactlyAtTheDeadline() {
        Sequencer s = run(waitMillis(now::get, () -> 100));
        s.update(); // starts
        now.set(99);
        assertEquals(Sequencer.Status.RUNNING, s.update());
        now.set(100);
        assertEquals(Sequencer.Status.FINISHED, s.update());
    }

    // If this fails: a duration tuned live in Panels after the route was built is ignored
    @Test
    public void waitMillisReadsDurationWhenItStarts() {
        double[] duration = {1000};
        Step w = waitMillis(now::get, () -> duration[0]);
        duration[0] = 10;
        Sequencer s = run(w);
        s.update();
        now.set(10);
        assertEquals(Sequencer.Status.FINISHED, s.update());
    }

    // If this fails: a stuck creep never gives up and eats the rest of the match
    @Test
    public void timeoutAbandonsAStepThatNeverFinishes() {
        Sequencer s = run(sequence(timeout(now::get, () -> 50, waitUntil("never", () -> false)), log("next")));
        s.update();
        now.set(49);
        s.update();
        assertFalse(log.contains("next"));
        now.set(50);
        assertEquals(Sequencer.Status.FINISHED, s.update());
        assertTrue(log.contains("next"));
    }

    // If this fails: a timeout wrapper delays a step that finishes early until the deadline
    @Test
    public void timeoutFinishesEarlyWhenTheInnerStepDoes() {
        Sequencer s = run(timeout(now::get, () -> 1000, new Counting("a", 1)));
        s.update();
        assertEquals(Sequencer.Status.FINISHED, s.update());
    }

    // If this fails: telemetry cannot tell which step of which case is running
    @Test
    public void activeNameShowsThePathToTheRunningStep() {
        Sequencer s = run(sequence("auto", sequence("CASE1", waitUntil("creep", () -> false))));
        s.update();
        assertEquals("auto>CASE1>creep", s.currentName());
    }

    // If this fails: the pre-match plan hides a branch arm so a wrong route cannot be spotted before the match
    @Test
    public void describeListsBothBranchArmsWithoutRunningAnything() {
        List<String> lines = Steps.describe(sequence("auto", branch("tags visible", () -> true, log("A"), log("B"))));
        assertEquals("[auto,   tags visible?,     yes:,       A,     no:,       B]", lines.toString());
        assertTrue(log.isEmpty());
    }
}

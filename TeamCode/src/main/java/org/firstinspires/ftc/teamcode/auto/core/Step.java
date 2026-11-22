package org.firstinspires.ftc.teamcode.auto.core;

import java.util.List;

/**
 * One unit of an autonomous routine: drive a path, wait for the flywheel, flip a servo, run a nested routine, ...
 *
 * <p>This package is plain Java (no FTC, Pedro, or Android types) so it can be unit tested on a laptop and reused by
 * any autonomous, including ones that partner with another robot.
 *
 * <h3>Lifecycle contract (enforced by {@link Sequencer}, relied on by every implementation)</h3>
 * <ol>
 *   <li>{@link #start()} is called exactly once per run, before any {@link #update()}. It may be called again later
 *       to run the same instance a second time, so implementations MUST reset all run state in {@code start()}.</li>
 *   <li>{@link #update()} is called once per loop after the loop that called {@code start()}, never in the same
 *       loop. That matters for a drive step: the follower still reports the previous path's state until its next
 *       update, so checking {@code isBusy()} in the loop that started the path would finish it instantly.</li>
 *   <li>Once {@code update()} returns true it is never called again until the next {@code start()}.</li>
 *   <li>A step that returns true from {@link #completesOnStart()} does all its work in {@code start()}; its
 *       {@code update()} is never called. This lets instant actions run in the same loop as the step before them.</li>
 * </ol>
 */
public interface Step {
    /** Static, human-readable name of this step ("" for anonymous composites). Never null. **/
    String label();

    /**
     * Name of what is running right now, e.g. {@code "CASE1>collect scoring flower>creep in"}. Only meaningful for
     * telemetry; MAY allocate, so do not call it every loop outside debug output.
     */
    default String activeName() {
        return label();
    }

    /** Begins this step. See the lifecycle contract above. **/
    void start();

    /** Advances this step by one loop. @return true once the step is finished **/
    boolean update();

    /**
     * Asked right AFTER {@link #start()}, never before. @return true if that {@code start()} already completed the
     * step, so {@link #update()} must not be called and the next step may start in this same loop. Fixed for instant
     * actions; composites report whether everything inside them finished during {@code start()}.
     */
    default boolean completesOnStart() {
        return false;
    }

    /**
     * Appends this step's static structure, one line per element, each prefixed by {@code indent}. Composites list
     * their children (and every arm of a branch) so the full plan can be printed before the match.
     */
    default void describe(List<String> out, String indent) {
        out.add(indent + label());
    }
}

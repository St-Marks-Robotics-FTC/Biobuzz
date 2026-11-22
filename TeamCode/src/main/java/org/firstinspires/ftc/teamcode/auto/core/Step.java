package org.firstinspires.ftc.teamcode.auto.core;

import java.util.List;

/**
 * One unit of an autonomous routine. Plain Java, no FTC types.
 * {@link #start()} runs once, then {@link #update()} runs each later loop until it returns true.
 * A step must reset all its state in {@code start()} because instances are reused.
 */
public interface Step {
    /** Static name; "" for anonymous composites. **/
    String label();

    /** Name of what is running right now, for telemetry. May allocate. **/
    default String activeName() {
        return label();
    }

    void start();

    /** @return true once finished; never called in the same loop as {@link #start()} **/
    boolean update();

    /** Asked right after {@link #start()}: true if start already finished the step, so update is skipped. **/
    default boolean completesOnStart() {
        return false;
    }

    /** Appends this step's structure (every branch arm included) for the pre-match plan. **/
    default void describe(List<String> out, String indent) {
        out.add(indent + label());
    }
}

package org.firstinspires.ftc.teamcode.auto.core;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Runs a fixed list of {@link Step}s one after another from a queue. Allocation-free after construction. **/
public final class Sequencer {
    public enum Status { NOT_STARTED, RUNNING, FINISHED, ABORTED }

    private final List<Step> steps;
    private final ArrayDeque<Step> queue;
    private Step current;
    private Status status = Status.NOT_STARTED;

    public Sequencer(List<Step> steps) {
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
        this.queue = new ArrayDeque<>(Math.max(1, steps.size()));
    }

    /** Begins (or restarts) from the first step. **/
    public void start() {
        queue.clear();
        queue.addAll(steps);
        current = null;
        status = Status.RUNNING;
    }

    /**
     * Advances one loop. A step started here is not updated until the next call.
     * @throws IllegalStateException if {@link #start()} was never called
     */
    public Status update() {
        if (status == Status.NOT_STARTED) {
            throw new IllegalStateException("Sequencer.update() called before start()");
        }
        if (status != Status.RUNNING) return status;

        while (true) {
            if (current != null) {
                if (!current.update()) return status;
                current = null;
            }
            Step next = queue.poll();
            if (next == null) {
                status = Status.FINISHED;
                return status;
            }
            next.start();
            if (!next.completesOnStart()) {
                current = next;
                return status;
            }
        }
    }

    /** Drops everything queued. Does not touch hardware; the caller stops it. **/
    public void abort() {
        queue.clear();
        current = null;
        status = Status.ABORTED;
    }

    public Status status() {
        return status;
    }

    /** Name of the running step, or "" if none. May allocate. **/
    public String currentName() {
        return current == null ? "" : current.activeName();
    }
}

package org.firstinspires.ftc.teamcode.auto.core;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Runs a fixed list of {@link Step}s one after another from a queue. It is the engine behind both the top-level
 * autonomous and every nested {@code sequence(...)}.
 *
 * <p>Steps are queued once at construction; {@link #start()} refills the queue so the same sequencer can run again.
 * Nothing allocates after construction, so it is safe in the 20 ms loop.
 */
public final class Sequencer {
    public enum Status { NOT_STARTED, RUNNING, FINISHED, ABORTED }

    private final List<Step> steps;
    private final ArrayDeque<Step> queue;
    private Step current;
    private Status status = Status.NOT_STARTED;

    /** @param steps the steps to run in order; copied, so later changes to the list are ignored **/
    public Sequencer(List<Step> steps) {
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
        this.queue = new ArrayDeque<>(Math.max(1, steps.size()));
    }

    /** Begins (or restarts) the sequence from its first step. The first step is not started until {@link #update()}. **/
    public void start() {
        queue.clear();
        queue.addAll(steps);
        current = null;
        status = Status.RUNNING;
    }

    /**
     * Advances the sequence one loop: updates the current step, and when it finishes starts the next ones until one
     * needs more loops. A step started here is not updated until the next call.
     *
     * @return the status after this loop; {@code FINISHED}/{@code ABORTED} are sticky
     * @throws IllegalStateException if {@link #start()} has not been called
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

    /** Drops everything still queued and the step in progress. Idempotent. Does not touch hardware; the caller stops it. **/
    public void abort() {
        queue.clear();
        current = null;
        status = Status.ABORTED;
    }

    public Status status() {
        return status;
    }

    /** @return {@link Step#activeName()} of the step in progress, or "" when none is. MAY allocate. **/
    public String currentName() {
        return current == null ? "" : current.activeName();
    }
}

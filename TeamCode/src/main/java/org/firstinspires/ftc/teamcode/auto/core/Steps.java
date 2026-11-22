package org.firstinspires.ftc.teamcode.auto.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/** Robot-independent {@link Step}s: composition, waits, and one-shot actions. Built once at init, reused. **/
public final class Steps {
    private Steps() {}

    // ======================================================================== composition

    /** Runs the steps in order. **/
    public static Step sequence(String label, Step... steps) {
        return new Sequence(label, Arrays.asList(steps));
    }

    /** {@link #sequence(String, Step...)} with no label. **/
    public static Step sequence(Step... steps) {
        return sequence("", steps);
    }

    /** Runs all steps at once; finishes when all have. Children must not share hardware. **/
    public static Step parallel(String label, Step... steps) {
        return new Parallel(label, steps);
    }

    /** Picks an arm when the branch starts (not when built) and runs it. {@code label} is the question asked. **/
    public static Step branch(String label, BooleanSupplier condition, Step whenTrue, Step whenFalse) {
        return new Branch(label, condition, whenTrue, whenFalse);
    }

    // ======================================================================== actions and waits

    /** Runs {@code action} once, in the same loop as the step before. **/
    public static Step instant(String label, Runnable action) {
        return new Instant(label, action);
    }

    /** Finishes when {@code condition} is true. **/
    public static Step waitUntil(String label, BooleanSupplier condition) {
        return new WaitUntil(label, condition);
    }

    /** Waits {@code millis} (read when the step starts). **/
    public static Step waitMillis(LongSupplier clockMillis, DoubleSupplier millis) {
        return new Timeout(clockMillis, millis, null);
    }

    /** Runs {@code inner}, giving up after {@code millis}. The abandoned step is not cancelled, just no longer updated. **/
    public static Step timeout(LongSupplier clockMillis, DoubleSupplier millis, Step inner) {
        if (inner == null) throw new IllegalArgumentException("timeout() needs a step to limit; use waitMillis() to just wait");
        return new Timeout(clockMillis, millis, inner);
    }

    // ======================================================================== planning

    /** @return the structure of {@code step}, one line per element, every branch arm listed **/
    public static List<String> describe(Step step) {
        List<String> lines = new ArrayList<>();
        step.describe(lines, "");
        return lines;
    }


    static String join(String a, String b) {
        if (a.isEmpty()) return b;
        if (b.isEmpty()) return a;
        return a + ">" + b;
    }

    private static String orDefault(String label, String fallback) {
        return label.isEmpty() ? fallback : label;
    }

    // ======================================================================== implementations

    private static final class Sequence implements Step {
        private final String label;
        private final List<Step> children;
        private final Sequencer inner;

        Sequence(String label, List<Step> children) {
            for (Step child : children) {
                if (child == null) throw new IllegalArgumentException("sequence '" + label + "' has a null step");
            }
            this.label = label;
            this.children = Collections.unmodifiableList(new ArrayList<>(children));
            this.inner = new Sequencer(this.children);
        }

        @Override public String label() { return label; }
        @Override public String activeName() { return join(label, inner.currentName()); }

        @Override public void start() {
            inner.start();
            inner.update(); // runs leading instants and starts the first real step now, not a loop later
        }

        @Override public boolean update() {
            return inner.update() == Sequencer.Status.FINISHED;
        }

        @Override public boolean completesOnStart() { return inner.status() == Sequencer.Status.FINISHED; }

        @Override public void describe(List<String> out, String indent) {
            out.add(indent + orDefault(label, "sequence"));
            for (Step child : children) child.describe(out, indent + "  ");
        }
    }

    private static final class Parallel implements Step {
        private final String label;
        private final Step[] children;
        private final boolean[] done;

        Parallel(String label, Step[] children) {
            for (Step child : children) {
                if (child == null) throw new IllegalArgumentException("parallel '" + label + "' has a null step");
            }
            this.label = label;
            this.children = children.clone();
            this.done = new boolean[children.length];
        }

        @Override public String label() { return label; }

        @Override public String activeName() {
            StringBuilder running = new StringBuilder();
            for (int i = 0; i < children.length; i++) {
                if (done[i]) continue;
                if (running.length() > 0) running.append('|');
                running.append(children[i].activeName());
            }
            return join(label, running.toString());
        }

        @Override public void start() {
            for (int i = 0; i < children.length; i++) {
                children[i].start();
                done[i] = children[i].completesOnStart();
            }
        }

        @Override public boolean update() {
            boolean all = true;
            for (int i = 0; i < children.length; i++) {
                if (!done[i]) done[i] = children[i].update();
                all &= done[i];
            }
            return all;
        }

        @Override public boolean completesOnStart() {
            for (boolean d : done) if (!d) return false;
            return true;
        }

        @Override public void describe(List<String> out, String indent) {
            out.add(indent + orDefault(label, "parallel") + " (all at once)");
            for (Step child : children) child.describe(out, indent + "  ");
        }
    }

    private static final class Branch implements Step {
        private final String label;
        private final BooleanSupplier condition;
        private final Step whenTrue, whenFalse;
        private Step chosen; // null until started

        Branch(String label, BooleanSupplier condition, Step whenTrue, Step whenFalse) {
            if (condition == null || whenTrue == null || whenFalse == null) {
                throw new IllegalArgumentException("branch '" + label + "' needs a condition and both arms");
            }
            this.label = label;
            this.condition = condition;
            this.whenTrue = whenTrue;
            this.whenFalse = whenFalse;
        }

        @Override public String label() { return label; }
        @Override public String activeName() { return chosen == null ? label : join(label + (chosen == whenTrue ? "=yes" : "=no"), chosen.activeName()); }

        @Override public void start() {
            chosen = condition.getAsBoolean() ? whenTrue : whenFalse;
            chosen.start();
        }

        @Override public boolean update() {
            if (chosen == null) throw new IllegalStateException("branch '" + label + "' updated before start()");
            return chosen.update();
        }

        @Override public boolean completesOnStart() { return chosen.completesOnStart(); }

        @Override public void describe(List<String> out, String indent) {
            out.add(indent + label + "?");
            out.add(indent + "  yes:");
            whenTrue.describe(out, indent + "    ");
            out.add(indent + "  no:");
            whenFalse.describe(out, indent + "    ");
        }
    }

    private static final class Instant implements Step {
        private final String label;
        private final Runnable action;

        Instant(String label, Runnable action) {
            if (action == null) throw new IllegalArgumentException("instant '" + label + "' has no action");
            this.label = label;
            this.action = action;
        }

        @Override public String label() { return label; }
        @Override public void start() { action.run(); }
        @Override public boolean update() { return true; }
        @Override public boolean completesOnStart() { return true; }
    }

    private static final class WaitUntil implements Step {
        private final String label;
        private final BooleanSupplier condition;

        WaitUntil(String label, BooleanSupplier condition) {
            if (condition == null) throw new IllegalArgumentException("waitUntil '" + label + "' has no condition");
            this.label = label;
            this.condition = condition;
        }

        @Override public String label() { return label; }
        @Override public void start() {}
        @Override public boolean update() { return condition.getAsBoolean(); }
    }

    /** A plain wait ({@code inner == null}) or a time limit around {@code inner}. **/
    private static final class Timeout implements Step {
        private final LongSupplier clockMillis;
        private final DoubleSupplier millis;
        private final Step inner; // null = plain wait
        private long startedAt;
        private double limit;
        private boolean innerDone;

        Timeout(LongSupplier clockMillis, DoubleSupplier millis, Step inner) {
            if (clockMillis == null || millis == null) throw new IllegalArgumentException("a timed step needs a clock and a duration");
            this.clockMillis = clockMillis;
            this.millis = millis;
            this.inner = inner;
        }

        @Override public String label() { return inner == null ? "wait" : "timeout"; }
        @Override public String activeName() { return inner == null ? "wait" : inner.activeName(); }

        @Override public void start() {
            startedAt = clockMillis.getAsLong();
            limit = millis.getAsDouble();
            innerDone = false;
            if (inner != null) {
                inner.start();
                innerDone = inner.completesOnStart();
            }
        }

        @Override public boolean update() {
            if (inner != null && !innerDone) innerDone = inner.update();
            return innerDone || clockMillis.getAsLong() - startedAt >= limit;
        }

        @Override public boolean completesOnStart() { return innerDone; }

        @Override public void describe(List<String> out, String indent) {
            if (inner == null) {
                out.add(indent + "wait");
            } else {
                out.add(indent + "timeout");
                inner.describe(out, indent + "  ");
            }
        }
    }
}

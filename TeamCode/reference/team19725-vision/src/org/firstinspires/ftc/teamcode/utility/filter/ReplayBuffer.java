package org.firstinspires.ftc.teamcode.utility.filter;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

/**
 * Rollback replay buffer for delayed vision measurements.
 *
 * <p>Stores a filter state snapshot after every {@code predict} over the recent window
 * (state {@code x}, covariance {@code P}, the process noise {@code Q} in effect at the
 * time, and the velocity input that produced the snapshot). When a delayed vision
 * observation arrives, the filter can first roll back to the observation instant, complete
 * the update there, and then replay the odometry predictions that followed to return to
 * the current instant, removing the lag caused by delay in the vision pipeline.
 *
 * <p>The idea follows the vision latency compensation of WPILib {@code PoseEstimator}
 * (align a timestamped observation to the instant it actually occurred, then fuse), but
 * the implementation differs: {@code PoseEstimator} interpolates an odometry pose buffer
 * and applies a uniform pose transform, whereas this class uses <b>filter state rollback +
 * motion model replay</b>, which suits recursive filters such as EKF/UKF far better. The
 * two cannot be derived from one another.
 *
 * <p>Snapshot timestamps must be strictly increasing, enforced by {@link #add}
 * (out-of-order samples are dropped); snapshots older than {@link #DEFAULT_DURATION} are
 * evicted.
 */
public final class ReplayBuffer {

    /** Default retention duration (seconds); matches the 1.5 s buffer in WPILib PoseEstimator. */
    public static final double DEFAULT_DURATION = 1.5;

    /** Single-frame state snapshot. */
    public static final class Snapshot {
        /** Snapshot timestamp (seconds). */
        public final double t;
        /** State [x, y, theta]. */
        private double[] x;
        /** State covariance (3x3). */
        private SimpleMatrix P;
        /**
         * Process noise covariance Q (3x3) used to propagate to this snapshot.
         *
         * <p>Replay must reuse the historical Q instead of the filter's current Q: under the
         * adaptive strategy Q jumps frame to frame as IMU impact/slope detection changes
         * (see Theory.md §4.4); replaying the whole history with the current Q would inject
         * the latest noise level into the covariance of that past interval and make the
         * replayed result disagree with the original propagation.
         */
        private final SimpleMatrix Q;
        /** Control input (robot-local velocity) used to propagate the previous snapshot to this one. */
        public final double vx;
        public final double vy;
        public final double omega;

        Snapshot(double t, double[] x, SimpleMatrix P, SimpleMatrix Q,
                 double vx, double vy, double omega) {
            this.t = t;
            this.x = x.clone();
            this.P = P.copy();
            this.Q = Q.copy();
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }

        /** @return a copy of the state [x, y, theta] */
        public double[] getX() {
            return x.clone();
        }

        /** @return a copy of the state covariance (3x3) */
        public SimpleMatrix getP() {
            return P.copy();
        }

        /** @return a copy of the process noise Q (3x3) used to propagate to this snapshot */
        public SimpleMatrix getQ() {
            return Q.copy();
        }

        /**
         * Refresh this snapshot with the new post-replay state (written back when the
         * rollback replay completes).
         *
         * @param x the new state [x, y, theta]
         * @param P the new state covariance (3x3)
         */
        public void refresh(double[] x, SimpleMatrix P) {
            this.x = x.clone();
            this.P = P.copy();
        }
    }

    private final double duration;
    private final ArrayDeque<Snapshot> samples = new ArrayDeque<>();

    public ReplayBuffer() {
        this(DEFAULT_DURATION);
    }

    public ReplayBuffer(double duration) {
        if (!(duration > 0)) {
            throw new IllegalArgumentException("duration must be positive, got " + duration);
        }
        this.duration = duration;
    }

    public boolean isEmpty() {
        return samples.isEmpty();
    }

    public int size() {
        return samples.size();
    }

    public void clear() {
        samples.clear();
    }

    /**
     * Records a snapshot after one predict.
     *
     * @param t     snapshot timestamp (seconds)
     * @param x     state [x, y, theta]
     * @param P     state covariance (copied)
     * @param Q     process noise covariance used to propagate to this snapshot (copied);
     *              reused by historical value during replay
     * @param vx    input vx used from the previous snapshot to this one (in/s)
     * @param vy    input vy used from the previous snapshot to this one (in/s)
     * @param omega input omega used from the previous snapshot to this one (rad/s)
     */
    public void add(double t, double[] x, SimpleMatrix P, SimpleMatrix Q,
                    double vx, double vy, double omega) {
        // Enforce strictly increasing timestamps: out-of-order samples would break the
        // monotonicity assumption behind floor()/after(), so they are dropped outright
        if (!samples.isEmpty() && t <= samples.peekLast().t) {
            return;
        }

        samples.addLast(new Snapshot(t, x, P, Q, vx, vy, omega));

        // Evict snapshots that are too old; keep at least 2 so rollback remains possible
        while (samples.size() > 2 && samples.peekFirst().t < t - duration) {
            samples.pollFirst();
        }
    }

    /**
     * Returns the newest snapshot whose timestamp is not greater than {@code t}.
     *
     * @return null if every snapshot is later than {@code t} (the observation precedes the
     *         buffer window)
     */
    public Snapshot floor(double t) {
        Snapshot result = null;
        for (Snapshot s : samples) {
            if (s.t <= t) {
                result = s;
            } else {
                break;
            }
        }
        return result;
    }

    /**
     * Returns all snapshots whose timestamp is strictly greater than {@code t}, in
     * ascending time order.
     */
    public List<Snapshot> after(double t) {
        List<Snapshot> list = new ArrayList<>();
        for (Snapshot s : samples) {
            if (s.t > t) {
                list.add(s);
            }
        }
        return list;
    }
}

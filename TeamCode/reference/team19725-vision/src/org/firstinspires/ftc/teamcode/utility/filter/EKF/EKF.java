package org.firstinspires.ftc.teamcode.utility.filter.EKF;

import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer;

/**
 * 3-dimensional extended Kalman filter for fusing Pinpoint (odometry velocity) and
 * Limelight (visual pose) into a single localization estimate.
 *
 * State vector: [x, y, theta] (robot pose in the global frame)
 * Control input (predict): robot-local velocity [vx, vy, omega] supplied by Pinpoint
 * Observation (update): global pose [x, y, theta] supplied by Limelight
 *
 * Reference: Kou & Haggenmiller (2023), "Extended Kalman Filter State Estimation
 *       for Autonomous Competition Robots"
 */
public class EKF {

    /** State vector [x, y, theta] (3x1) */
    private SimpleMatrix state;

    /** State covariance matrix (3x3) */
    private SimpleMatrix P;

    /** Process noise covariance matrix Q (3x3) */
    private SimpleMatrix Q;

    /** Measurement noise covariance matrix R (3x3) */
    private SimpleMatrix R;

    /** Observation matrix H = I (3x3), because the observation is the state itself */
    private final SimpleMatrix H = SimpleMatrix.identity(3);

    /** Timestamp of the previous predict call, used to compute dt */
    private Double lastPredictTime = null;

    /** Timestamp of the previous update call, used to reject stale observations */
    private Double lastUpdateTime = null;

    /** Whether visual-timestamp rollback replay is enabled */
    private final boolean allowReplay;

    /** Rollback replay buffer (created only when allowReplay is true) */
    private final ReplayBuffer replayBuffer;

    /** Maximum allowed gap (seconds) between the rollback baseline and the observation instant; beyond it the rollback is abandoned (consistent with the dt guard in predict) */
    private static final double MAX_REPLAY_GAP = 1.0;

    // ==================== Construction ====================

    /**
     * @param initialX      initial x
     * @param initialY      initial y
     * @param initialTheta  initial heading (radians)
     */
    public EKF(double initialX, double initialY, double initialTheta) {
        this(initialX, initialY, initialTheta, false);
    }

    /**
     * @param initialX      initial x
     * @param initialY      initial y
     * @param initialTheta  initial heading (radians)
     * @param allowReplay   whether to enable visual-timestamp rollback replay
     *                      (on a delayed visual observation, roll back to the observation
     *                      instant, update there, then replay the odometry predictions)
     */
    public EKF(double initialX, double initialY, double initialTheta, boolean allowReplay) {
        state = new SimpleMatrix(new double[][]{
                {initialX},
                {initialY},
                {initialTheta}
        });

        // Initial covariance: moderate position uncertainty, smaller angular uncertainty
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });

        // Default process noise (dt-dependent; actually multiplied by dt in predict)
        Q = new SimpleMatrix(new double[][]{
                {0.002, 0,     0     },
                {0,     0.002, 0     },
                {0,     0,     0.002 }
        });

        // Default measurement noise (visual measurements usually have larger angular noise than position noise)
        R = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.05}
        });

        this.allowReplay = allowReplay;
        this.replayBuffer = allowReplay ? new ReplayBuffer() : null;
    }

    // ==================== Core filtering ====================

    /**
     * Prediction step — called every frame with the robot-local velocity supplied by Pinpoint.
     * dt is computed automatically from the timestamps; the first call only records the
     * timestamp and performs no prediction.
     *
     * @param vx        x-direction velocity in the robot-local frame
     * @param vy        y-direction velocity in the robot-local frame
     * @param omega     robot angular velocity (radians/second)
     * @param timestamp current timestamp (seconds); must share the same time base as update
     */
    public void predict(double vx, double vy, double omega, double timestamp) {
        // First call: only record the timestamp, no prediction
        if (lastPredictTime == null) {
            lastPredictTime = timestamp;
            if (allowReplay) {
                replayBuffer.add(timestamp, getPose(), P, Q, 0, 0, 0);
            }
            return;
        }

        double dt = timestamp - lastPredictTime;
        lastPredictTime = timestamp;

        // Abnormal dt guard
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        applyMotion(vx, vy, omega, dt, Q);

        // Record a snapshot for visual rollback replay (together with the historical Q used for this segment)
        if (allowReplay) {
            replayBuffer.add(timestamp, getPose(), P, Q, vx, vy, omega);
        }
    }

    /**
     * Propagates the state and covariance (nonlinear motion model + Jacobian linearization),
     * without any timestamp or buffer logic.
     *
     * @param vx    robot-local x-direction velocity (in/s)
     * @param vy    robot-local y-direction velocity (in/s)
     * @param omega angular velocity (rad/s)
     * @param dt    propagation time (seconds)
     * @param Qused process noise Q used for this propagation segment (the current Q for a normal predict; the snapshot's historical Q during replay)
     */
    private void applyMotion(double vx, double vy, double omega, double dt, SimpleMatrix Qused) {
        double x = state.get(0, 0);
        double y = state.get(1, 0);
        double theta = state.get(2, 0);

        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);

        // ---- 1. State propagation (nonlinear) ----
        // Transform the local velocity into the global frame
        double xNew = x + dt * (vx * cosTheta - vy * sinTheta);
        double yNew = y + dt * (vx * sinTheta + vy * cosTheta);
        double thetaNew = theta + dt * omega;

        // ---- 2. Compute the Jacobian A = df/dx (3x3) ----
        // A[i][j] = ∂f_i / ∂x_j
        SimpleMatrix A = new SimpleMatrix(new double[][]{
                {1, 0, dt * (-vx * sinTheta - vy * cosTheta)},
                {0, 1, dt * ( vx * cosTheta - vy * sinTheta)},
                {0, 0, 1                                     }
        });

        // ---- 3. Covariance propagation P = A * P * A^T + Q*dt ----
        // Scaling Q by dt reflects the accumulation of process noise over time; during replay
        // this receives the historical Q recorded with that segment's snapshot
        SimpleMatrix Qdt = Qused.scale(dt);
        P = A.mult(P).mult(A.transpose()).plus(Qdt);

        // ---- 4. Update the state ----
        state = new SimpleMatrix(new double[][]{
                {xNew},
                {yNew},
                {normalizeAngle(thetaNew)}
        });
    }

    /**
     * Update step — called only when Limelight has a valid detection, with the visual global pose.
     * Timestamps are compared so that observations older than the previous update are rejected.
     *
     * @param xMeas        global x measured by Limelight
     * @param yMeas        global y measured by Limelight
     * @param thetaMeas    global heading measured by Limelight (radians)
     * @param timestamp    observation timestamp (seconds)
     */
    public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        // Reject stale observations (kept regardless of whether replay is enabled)
        if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
            return;
        }

        if (allowReplay) {
            // Roll back to the observation instant, update there, then replay the later predictions;
            // observations older than the buffer window are dropped
            if (replayUpdate(xMeas, yMeas, thetaMeas, timestamp)) {
                lastUpdateTime = timestamp;
            }
            return;
        }

        lastUpdateTime = timestamp;
        applyVisionUpdate(xMeas, yMeas, thetaMeas);
    }

    /**
     * Performs one observation update at the current state (standard EKF update step).
     *
     * @param xMeas     visual global x
     * @param yMeas     visual global y
     * @param thetaMeas visual global heading (radians)
     */
    private void applyVisionUpdate(double xMeas, double yMeas, double thetaMeas) {
        // ---- 1. Observation vector z = [xMeas, yMeas, thetaMeas]^T ----
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });

        // ---- 2. Innovation y_innov = z - H*x (H=I, so y_innov = z - x) ----
        SimpleMatrix yInnov = z.minus(state);
        // The angular innovation must be normalized to [-π, π]
        yInnov.set(2, 0, normalizeAngle(yInnov.get(2, 0)));

        // ---- 3. Innovation covariance S = H * P * H^T + R ----
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

        // ---- 4. Kalman gain K = P * H^T * S^{-1} ----
        SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

        // ---- 5. State update x = x + K * y_innov ----
        state = state.plus(K.mult(yInnov));
        state.set(2, 0, normalizeAngle(state.get(2, 0)));

        // ---- 6. Covariance update P = (I - K*H) * P ----
        SimpleMatrix I = SimpleMatrix.identity(3);
        P = I.minus(K.mult(H)).mult(P);
    }

    /**
     * Rollback-replay update — rewinds the filter to the visual observation instant, performs the
     * update there, then replays the subsequent odometry predictions back to the current instant,
     * eliminating the lag bias caused by visual pipeline latency.
     *
     * @return true if the observation was applied; false if it was dropped because it predates the
     *         buffer window or lies too far from the rollback baseline
     */
    private boolean replayUpdate(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        ReplayBuffer.Snapshot base = replayBuffer.floor(timestamp);
        if (base == null) {
            // The observation predates the buffer window, so no rollback is possible
            return false;
        }

        // If the gap between the rollback baseline and the observation instant is too large
        // (predictions stopped / clock anomaly), abandon the rollback to avoid replaying a huge dt
        if (timestamp - base.t > MAX_REPLAY_GAP) {
            return false;
        }

        // Every snapshot after the observation instant (all timestamps strictly greater than it)
        List<ReplayBuffer.Snapshot> tail = replayBuffer.after(base.t);

        // ---- 1. Roll back to the nearest snapshot before the observation instant ----
        state = toColumn(base.getX());
        P = base.getP();

        // ---- 2. Advance from that snapshot to the observation instant (using the next segment's inputs and historical Q) ----
        if (!tail.isEmpty()) {
            ReplayBuffer.Snapshot first = tail.get(0);
            applyMotion(first.vx, first.vy, first.omega, timestamp - base.t, first.getQ());
        }

        // ---- 3. Perform the update at the observation instant ----
        // Note: R is the noise of this particular observation (computed by this frame's adaptR from
        //       that frame's stdDev) and is independent of the time axis, so the current R is used;
        //       only Q is a time-varying process noise, replayed from each snapshot's historical Q.
        applyVisionUpdate(xMeas, yMeas, thetaMeas);

        // ---- 4. Replay every later prediction back to the current instant (each segment uses its own snapshot's historical Q) and refresh the snapshots ----
        double prevT = timestamp;
        for (ReplayBuffer.Snapshot s : tail) {
            applyMotion(s.vx, s.vy, s.omega, s.t - prevT, s.getQ());
            prevT = s.t;
            s.refresh(getPose(), P);
        }
        return true;
    }

    /** Converts [x, y, theta] into a 3x1 column vector. */
    private SimpleMatrix toColumn(double[] v) {
        return new SimpleMatrix(new double[][]{
                {v[0]},
                {v[1]},
                {v[2]}
        });
    }

    /**
     * Vision observation gating — computes the Mahalanobis distance to decide whether this visual
     * observation is an outlier.
     *
     * <p>Because the observation model is H = I, the innovation is {@code y = z - x̂} and the
     * innovation covariance is {@code S = P + R}. When the robot is far from the AprilTag the
     * visual error is very large and a direct update would drag the filter off course; gating on
     * the Mahalanobis distance rejects such outlier observations.
     *
     * @param xMeas         visual global x (inches)
     * @param yMeas         visual global y (inches)
     * @param thetaMeas     visual global heading (radians)
     * @param gateThreshold Mahalanobis-distance gate threshold (dimensionless)
     * @return true if the measurement is acceptable (Mahalanobis distance ≤ gateThreshold)
     */
    public boolean gateVision(double xMeas, double yMeas, double thetaMeas, double gateThreshold) {
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });
        SimpleMatrix innov = z.minus(state);
        innov.set(2, 0, normalizeAngle(innov.get(2, 0)));

        // H = I → S = P + R
        SimpleMatrix S = P.plus(R);
        SimpleMatrix d2 = innov.transpose().mult(safeInvert(S)).mult(innov);
        return d2.get(0, 0) <= gateThreshold * gateThreshold;
    }

    // ==================== Output ====================

    /**
     * @return the current fused pose as double[3] = {x, y, theta}
     */
    public double[] getPose() {
        return new double[]{
                state.get(0, 0),
                state.get(1, 0),
                state.get(2, 0)
        };
    }

    /**
     * @return the diagonal of the state covariance matrix {var_x, var_y, var_theta}
     */
    public double[] getCovarianceDiag() {
        return new double[]{
                P.get(0, 0),
                P.get(1, 1),
                P.get(2, 2)
        };
    }

    /**
     * @return the diagonal of the current process noise matrix Q {q_x, q_y, q_theta}
     */
    public double[] getQDiag() {
        return new double[]{
                Q.get(0, 0),
                Q.get(1, 1),
                Q.get(2, 2)
        };
    }

    /**
     * @return the diagonal of the current measurement noise matrix R {r_x, r_y, r_theta}
     */
    public double[] getRDiag() {
        return new double[]{
                R.get(0, 0),
                R.get(1, 1),
                R.get(2, 2)
        };
    }

    // ==================== Noise parameter tuning ====================

    /**
     * Sets the diagonal of the process noise covariance matrix Q at runtime.
     * Larger values = more trust in the Pinpoint velocity input, but the accumulated error grows faster.
     *
     * @param qx       x-direction process noise
     * @param qy       y-direction process noise
     * @param qtheta   theta-direction process noise
     */
    public void setQ(double qx, double qy, double qtheta) {
        Q = new SimpleMatrix(new double[][]{
                {qx, 0,  0     },
                {0,  qy, 0     },
                {0,  0,  qtheta}
        });
    }

    /**
     * Sets the complete Q matrix (3x3) directly.
     * The adaptive logic passes in an already-built SimpleMatrix object.
     */
    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    /**
     * Sets the diagonal of the measurement noise covariance matrix R at runtime.
     * Smaller values = more trust in the Limelight observation.
     *
     * @param rx       x-direction measurement noise
     * @param ry       y-direction measurement noise
     * @param rtheta   theta-direction measurement noise
     */
    public void setR(double rx, double ry, double rtheta) {
        R = new SimpleMatrix(new double[][]{
                {rx, 0,  0     },
                {0,  ry, 0     },
                {0,  0,  rtheta}
        });
    }

    /**
     * Sets the complete R matrix (3x3) directly.
     * The adaptive logic passes in an already-built SimpleMatrix object.
     */
    public void setR(SimpleMatrix R) {
        this.R = R;
    }

    // ==================== Reset ====================

    /**
     * Resets the filter to a new initial pose and clears the timestamps.
     */
    public void reset(double x, double y, double theta) {
        state = new SimpleMatrix(new double[][]{
                {x},
                {y},
                {theta}
        });
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });
        lastPredictTime = null;
        lastUpdateTime = null;
        if (replayBuffer != null) {
            replayBuffer.clear();
        }
    }

    // ==================== Rollback replay state ====================

    /** @return whether visual-timestamp rollback replay is enabled */
    public boolean isReplayEnabled() {
        return allowReplay;
    }

    /** @return the number of snapshots in the rollback buffer (0 when replay is disabled) */
    public int getReplayBufferSize() {
        return replayBuffer == null ? 0 : replayBuffer.size();
    }

    // ==================== Internal utilities ====================

    /**
     * Normalizes an angle into the range [-π, π].
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Safe inversion — S = H·P·Hᵀ + R may be singular when R is configured too small or when the
     * numerics degenerate. If inversion fails, retry after adding a diagonal jitter, avoiding a
     * thrown exception or an exploding gain.
     */
    private SimpleMatrix safeInvert(SimpleMatrix m) {
        try {
            return m.invert();
        } catch (RuntimeException e) {
            SimpleMatrix jittered = m.plus(SimpleMatrix.identity(m.numRows()).scale(1e-9));
            return jittered.invert();
        }
    }
}
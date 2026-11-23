package org.firstinspires.ftc.teamcode.utility.filter.UKF;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer;

/**
 * 3D Unscented Kalman Filter (UKF) for fused localization from Pinpoint (odometry velocity) and
 * Limelight (vision pose).
 *
 * <p>Unlike the EKF, the UKF replaces Jacobian linearization with the sigma-point unscented transform,
 * propagating the nonlinear distribution directly through 2n+1 deterministic sample points, accurate
 * to the third-order moment.
 *
 * <p>State vector: [x, y, theta] (robot pose in the field frame)
 * <br>Control input (predict): robot-frame velocity [vx, vy, omega] from Pinpoint
 * <br>Measurement (update): field-frame pose [x, y, theta] from Limelight
 *
 * <p>UKF parameters: alpha=1, beta=2, kappa=0, so lambda=0, 7 sigma points (2n+1)
 *
 * <p>References:
 * <ul>
 *   <li>Wan & van der Merwe (2001), "The Unscented Kalman Filter"
 *   <li>Julier & Uhlmann (1997), "A New Extension of the Kalman Filter
 *       to Nonlinear Systems"
 * </ul>
 */
public class UKF {

    // ==================== UKF parameters (n=3) ====================

    /** State dimension. */
    private static final int N = 3;

    /** Sigma-point spread parameter alpha = 1; controls the distance from sigma points to the mean. */
    private static final double ALPHA = 1.0;

    /** Optimal parameter beta = 2 (for Gaussian distributions). */
    private static final double BETA = 2.0;

    /** Secondary scaling parameter kappa = 0. */
    private static final double KAPPA = 0.0;

    /** Lambda = alpha^2(n+kappa) - n = 1*3 - 3 = 0. */
    private final double lambda;

    /** n + lambda = 3 + 0 = 3. */
    private final double nPlusLambda;

    /** Total sigma-point count = 2n+1 = 7. */
    private static final int NUM_SIGMA = 2 * N + 1;

    /** Sigma-point mean weights (length 7). */
    private final double[] wm;

    /** Sigma-point covariance weights (length 7). */
    private final double[] wc;

    // ==================== Cholesky regularization ====================

    /** First-level Cholesky regularization epsilon. */
    private static final double CHOL_EPS_1 = 1e-6;

    /** Second-level Cholesky regularization epsilon. */
    private static final double CHOL_EPS_2 = 1e-3;

    // ==================== Filter state ====================

    /** State vector [x, y, theta] (3x1). */
    private SimpleMatrix state;

    /** State covariance matrix (3x3). */
    private SimpleMatrix P;

    /** Process-noise covariance matrix Q (3x3). */
    private SimpleMatrix Q;

    /** Measurement-noise covariance matrix R (3x3). */
    private SimpleMatrix R;

    /** Measurement matrix H = I (3x3), because the measurement is the state directly. */
    private final SimpleMatrix H = SimpleMatrix.identity(N);

    /** Timestamp of the last predict, used to compute dt. */
    private Double lastPredictTime = null;

    /** Timestamp of the last update, used to reject stale measurements. */
    private Double lastUpdateTime = null;

    /** Whether vision-timestamp rollback replay is enabled. */
    private final boolean allowReplay;

    /** Rollback replay buffer (created only when allowReplay is true). */
    private final ReplayBuffer replayBuffer;

    /** Maximum allowed gap (seconds) between the rollback base and the measurement time; skips rollback beyond it (consistent with the predict dt guard). */
    private static final double MAX_REPLAY_GAP = 1.0;

    // ==================== Construction ====================

    /**
     * @param initialX      initial x (inches)
     * @param initialY      initial y (inches)
     * @param initialTheta  initial heading (radians)
     */
    public UKF(double initialX, double initialY, double initialTheta) {
        this(initialX, initialY, initialTheta, false);
    }

    /**
     * @param initialX      initial x (inches)
     * @param initialY      initial y (inches)
     * @param initialTheta  initial heading (radians)
     * @param allowReplay   whether to enable vision-timestamp rollback replay
     *                      (on a delayed vision measurement, first roll back to the measurement time for the
     *                      update, then replay the odometry predictions)
     */
    public UKF(double initialX, double initialY, double initialTheta, boolean allowReplay) {
        // ---- State initialization (same as EKF) ----
        state = new SimpleMatrix(new double[][]{
                {initialX},
                {initialY},
                {initialTheta}
        });

        // Initial covariance: moderate position uncertainty, smaller heading uncertainty.
        P = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.01}
        });

        // Default process noise (dt-dependent; scaled by dt in predict).
        Q = new SimpleMatrix(new double[][]{
                {0.002, 0,     0     },
                {0,     0.002, 0     },
                {0,     0,     0.002 }
        });

        // Default measurement noise (vision heading noise is usually larger than position noise).
        R = new SimpleMatrix(new double[][]{
                {0.01, 0,    0   },
                {0,    0.01, 0   },
                {0,    0,    0.05}
        });

        // ---- UKF weight computation ----
        this.lambda = ALPHA * ALPHA * (N + KAPPA) - N;        // = 0
        this.nPlusLambda = N + lambda;                         // = 3

        this.wm = new double[NUM_SIGMA];
        this.wc = new double[NUM_SIGMA];

        wm[0] = lambda / nPlusLambda;                          // = 0
        wc[0] = lambda / nPlusLambda + (1 - ALPHA * ALPHA + BETA); // = 2

        double commonWeight = 1.0 / (2.0 * nPlusLambda);       // = 1/6
        for (int i = 1; i < NUM_SIGMA; i++) {
            wm[i] = commonWeight;
            wc[i] = commonWeight;
        }

        this.allowReplay = allowReplay;
        this.replayBuffer = allowReplay ? new ReplayBuffer() : null;
    }

    // ==================== Core filter ====================

    /**
     * Predict step -- call every frame with the robot-frame velocity from Pinpoint.
     * Propagates the state distribution with the unscented transform.
     *
     * @param vx        x velocity in the robot frame (in/s)
     * @param vy        y velocity in the robot frame (in/s)
     * @param omega     robot angular velocity (rad/s)
     * @param timestamp current timestamp (seconds)
     */
    public void predict(double vx, double vy, double omega, double timestamp) {
        // First call: record the timestamp only, no prediction yet.
        if (lastPredictTime == null) {
            lastPredictTime = timestamp;
            if (allowReplay) {
                replayBuffer.add(timestamp, getPose(), P, Q, 0, 0, 0);
            }
            return;
        }

        double dt = timestamp - lastPredictTime;
        lastPredictTime = timestamp;

        // Abnormal-dt guard.
        if (dt <= 0 || dt > 1.0) {
            return;
        }

        applyMotion(vx, vy, omega, dt, Q);

        // Record a snapshot for vision rollback replay (together with the historical Q used by this segment).
        if (allowReplay) {
            replayBuffer.add(timestamp, getPose(), P, Q, vx, vy, omega);
        }
    }

    /**
     * Propagates the state and covariance (unscented transform), without timestamp/buffer logic.
     *
     * @param vx    robot-frame x velocity (in/s)
     * @param vy    robot-frame y velocity (in/s)
     * @param omega angular velocity (rad/s)
     * @param dt    propagation time (seconds)
     * @param Qused process-noise Q used by this segment (current Q for a regular predict; snapshot historical Q for replay)
     */
    private void applyMotion(double vx, double vy, double omega, double dt, SimpleMatrix Qused) {
        // ---- 1. Generate sigma points ----
        double[][] sigmaPoints = generateSigmaPoints();

        // ---- 2. Propagate each sigma point through the nonlinear motion model ----
        // Motion model: x' = x + dt * (vx * cosθ - vy * sinθ)
        //               y' = y + dt * (vx * sinθ + vy * cosθ)
        //               θ' = θ + dt * ω
        double[][] propagated = new double[NUM_SIGMA][N];
        for (int i = 0; i < NUM_SIGMA; i++) {
            double theta = sigmaPoints[i][2];
            propagated[i][0] = sigmaPoints[i][0] + dt * (vx * Math.cos(theta) - vy * Math.sin(theta));
            propagated[i][1] = sigmaPoints[i][1] + dt * (vx * Math.sin(theta) + vy * Math.cos(theta));
            propagated[i][2] = sigmaPoints[i][2] + dt * omega;
        }

        // ---- 3. Compute the weighted mean (circular mean for heading) ----
        double xMean = 0.0, yMean = 0.0;
        double sumSinTheta = 0.0, sumCosTheta = 0.0;

        for (int i = 0; i < NUM_SIGMA; i++) {
            xMean += wm[i] * propagated[i][0];
            yMean += wm[i] * propagated[i][1];
            sumSinTheta += wm[i] * Math.sin(propagated[i][2]);
            sumCosTheta += wm[i] * Math.cos(propagated[i][2]);
        }
        double thetaMean = Math.atan2(sumSinTheta, sumCosTheta);

        // ---- 4. Compute the weighted covariance P = Σ wc[i]*(Xᵢ - x̂)(Xᵢ - x̂)ᵀ + Q*dt ----
        SimpleMatrix P_new = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = propagated[i][0] - xMean;
            double dy = propagated[i][1] - yMean;
            double dtheta = normalizeAngle(propagated[i][2] - thetaMean);

            double w = wc[i];
            P_new.set(0, 0, P_new.get(0, 0) + w * dx * dx);
            P_new.set(0, 1, P_new.get(0, 1) + w * dx * dy);
            P_new.set(0, 2, P_new.get(0, 2) + w * dx * dtheta);
            P_new.set(1, 0, P_new.get(1, 0) + w * dy * dx);
            P_new.set(1, 1, P_new.get(1, 1) + w * dy * dy);
            P_new.set(1, 2, P_new.get(1, 2) + w * dy * dtheta);
            P_new.set(2, 0, P_new.get(2, 0) + w * dtheta * dx);
            P_new.set(2, 1, P_new.get(2, 1) + w * dtheta * dy);
            P_new.set(2, 2, P_new.get(2, 2) + w * dtheta * dtheta);
        }

        // Add process noise Q*dt (process noise accumulates over time); replay passes the historical Q recorded in that segment's snapshot.
        P = P_new.plus(Qused.scale(dt));

        // ---- 5. Update the state ----
        state = new SimpleMatrix(new double[][]{
                {xMean},
                {yMean},
                {thetaMean}
        });
    }

    /**
     * Update step -- call only when a Limelight detection is valid, with the global vision pose.
     * Uses the Joseph form covariance update for numerical stability.
     *
     * @param xMeas      global x measured by Limelight (inches)
     * @param yMeas      global y measured by Limelight (inches)
     * @param thetaMeas  global heading measured by Limelight (radians)
     * @param timestamp  measurement timestamp (seconds)
     */
    public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        // Reject stale measurements (kept whether or not replay is enabled).
        if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
            return;
        }

        if (allowReplay) {
            // Roll back to the measurement time for the update, then replay later predictions; measurements older than the buffer window are dropped.
            if (replayUpdate(xMeas, yMeas, thetaMeas, timestamp)) {
                lastUpdateTime = timestamp;
            }
            return;
        }

        lastUpdateTime = timestamp;
        applyVisionUpdate(xMeas, yMeas, thetaMeas);
    }

    /**
     * Applies one measurement update at the current state (standard UKF update step).
     *
     * @param xMeas     global vision x
     * @param yMeas     global vision y
     * @param thetaMeas global vision heading (radians)
     */
    private void applyVisionUpdate(double xMeas, double yMeas, double thetaMeas) {
        // ---- 1. Measurement vector z = [xMeas, yMeas, thetaMeas]T ----
        SimpleMatrix z = new SimpleMatrix(new double[][]{
                {xMeas},
                {yMeas},
                {thetaMeas}
        });

        // ---- 2. Generate sigma points (from the current state and covariance) ----
        double[][] sigmaPoints = generateSigmaPoints();

        // ---- 3. Pass sigma points through the measurement model (H = I, the measurement is the state directly) ----
        // Hence Z_i = H * X_i = X_i; no extra transform needed.
        double[][] zSigma = sigmaPoints;  // Shared reference, read-only.

        // ---- 4. Compute the predicted measurement mean zhat = S wm[i]*Z_i ----
        // Equals the current state mean (because H = I).
        double zxMean = 0.0, zyMean = 0.0;
        double sumSinZ = 0.0, sumCosZ = 0.0;
        for (int i = 0; i < NUM_SIGMA; i++) {
            zxMean += wm[i] * zSigma[i][0];
            zyMean += wm[i] * zSigma[i][1];
            sumSinZ += wm[i] * Math.sin(zSigma[i][2]);
            sumCosZ += wm[i] * Math.cos(zSigma[i][2]);
        }
        double zThetaMean = Math.atan2(sumSinZ, sumCosZ);

        // ---- 5. Innovation y_innov = z - zhat ----
        SimpleMatrix yInnov = new SimpleMatrix(new double[][]{
                {z.get(0, 0) - zxMean},
                {z.get(1, 0) - zyMean},
                {normalizeAngle(z.get(2, 0) - zThetaMean)}
        });

        // ---- 6. Innovation covariance S = S wc[i]*(Z_i - zhat)(Z_i - zhat)T + R ----
        SimpleMatrix S = computeCovarianceFromSigma(zSigma, zxMean, zyMean, zThetaMean)
                .plus(R);

        // ---- 7. Cross-covariance Pxz = S wc[i]*(X_i - xhat)(Z_i - zhat)T ----
        double xMean = state.get(0, 0);
        double yMean = state.get(1, 0);
        double thetaMean = state.get(2, 0);

        SimpleMatrix Pxz = computeCrossCovariance(
                sigmaPoints, xMean, yMean, thetaMean,
                zSigma, zxMean, zyMean, zThetaMean
        );

        // ---- 8. Kalman gain K = Pxz * S^-1 ----
        SimpleMatrix K = Pxz.mult(safeInvert(S));

        // ---- 9. State update x = x + K * y_innov ----
        state = state.plus(K.mult(yInnov));
        state.set(2, 0, normalizeAngle(state.get(2, 0)));

        // ---- 10. Joseph-form covariance update P = P - K*S*Kᵀ ----
        // Joseph form preserves symmetry and positive semi-definiteness.
        P = P.minus(K.mult(S).mult(K.transpose()));

        // Enforce symmetry (removes accumulated floating-point error).
        P = P.plus(P.transpose()).scale(0.5);
    }

    /**
     * Rollback replay update -- rolls the filter back to the vision measurement time, applies the update
     * there, then replays the later odometry predictions up to the current time, removing the lag bias
     * caused by vision-link latency.
     *
     * @return true if the measurement was applied; false if it predates the buffer window or the gap to the rollback base is too large, so it was dropped
     */
    private boolean replayUpdate(double xMeas, double yMeas, double thetaMeas, double timestamp) {
        ReplayBuffer.Snapshot base = replayBuffer.floor(timestamp);
        if (base == null) {
            // Measurement predates the buffer window; cannot roll back.
            return false;
        }

        // Skip rollback when the gap between the rollback base and the measurement time is too large (stalled predictions / clock anomaly), to avoid replaying with a huge dt.
        if (timestamp - base.t > MAX_REPLAY_GAP) {
            return false;
        }

        // All snapshots after the measurement time (timestamps strictly greater than the measurement time).
        List<ReplayBuffer.Snapshot> tail = replayBuffer.after(base.t);

        // ---- 1. Roll back to the closest snapshot before the measurement time ----
        state = toColumn(base.getX());
        P = base.getP();

        // ---- 2. Propagate from that snapshot to the measurement time (using the following segment's inputs and historical Q) ----
        if (!tail.isEmpty()) {
            ReplayBuffer.Snapshot first = tail.get(0);
            applyMotion(first.vx, first.vy, first.omega, timestamp - base.t, first.getQ());
        }

        // ---- 3. Apply the update at the measurement time ----
        // Note: R is this measurement's own noise (computed for this frame by adaptR from that frame's stdDev),
        //     independent of the time axis, so keep the current R; only Q is time-varying process noise, replayed from the snapshots' historical Q.
        applyVisionUpdate(xMeas, yMeas, thetaMeas);

        // ---- 4. Replay all later predictions up to the current time (each segment with its snapshot's historical Q) and refresh the snapshots ----
        double prevT = timestamp;
        for (ReplayBuffer.Snapshot s : tail) {
            applyMotion(s.vx, s.vy, s.omega, s.t - prevT, s.getQ());
            prevT = s.t;
            s.refresh(getPose(), P);
        }
        return true;
    }

    /** Converts [x, y, theta] to a 3x1 column vector. */
    private SimpleMatrix toColumn(double[] v) {
        return new SimpleMatrix(new double[][]{
                {v[0]},
                {v[1]},
                {v[2]}
        });
    }

    /**
     * Vision measurement gating -- computes the Mahalanobis distance to decide whether a vision measurement is an outlier.
     *
     * <p>Measurement model H = I, so the innovation is {@code y = z - xhat} and the innovation covariance is {@code S = P + R}.
     * Far from an AprilTag, vision error is huge; updating with it directly would drag the filter off; reject outlier measurements via Mahalanobis-distance gating.
     *
     * @param xMeas         global vision x (inches)
     * @param yMeas         global vision y (inches)
     * @param thetaMeas     global vision heading (radians)
     * @param gateThreshold Mahalanobis-distance gating threshold (dimensionless)
     * @return true if the measurement is acceptable (Mahalanobis distance <= gateThreshold)
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

    // ==================== Outputs ====================

    /**
     * @return current fused pose double[3] = {x, y, theta} (inches, inches, radians)
     */
    public double[] getPose() {
        return new double[]{
                state.get(0, 0),
                state.get(1, 0),
                state.get(2, 0)
        };
    }

    /**
     * @return diagonal of the state covariance matrix {var_x, var_y, var_theta}
     */
    public double[] getCovarianceDiag() {
        return new double[]{
                P.get(0, 0),
                P.get(1, 1),
                P.get(2, 2)
        };
    }

    /**
     * @return diagonal of the current process-noise matrix Q {q_x, q_y, q_theta}
     */
    public double[] getQDiag() {
        return new double[]{
                Q.get(0, 0),
                Q.get(1, 1),
                Q.get(2, 2)
        };
    }

    /**
     * @return diagonal of the current measurement-noise matrix R {r_x, r_y, r_theta}
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
     * Sets the diagonal of the process-noise covariance matrix Q live.
     *
     * @param qx       x process noise (in^2/s)
     * @param qy       y process noise (in^2/s)
     * @param qtheta   theta process noise (rad^2/s)
     */
    public void setQ(double qx, double qy, double qtheta) {
        Q = new SimpleMatrix(new double[][]{
                {qx, 0,  0     },
                {0,  qy, 0     },
                {0,  0,  qtheta}
        });
    }

    /**
     * Sets the full Q matrix directly (3x3).
     */
    public void setQ(SimpleMatrix Q) {
        this.Q = Q;
    }

    /**
     * Sets the diagonal of the measurement-noise covariance matrix R live.
     *
     * @param rx       x measurement noise (in^2)
     * @param ry       y measurement noise (in^2)
     * @param rtheta   theta measurement noise (rad^2)
     */
    public void setR(double rx, double ry, double rtheta) {
        R = new SimpleMatrix(new double[][]{
                {rx, 0,  0     },
                {0,  ry, 0     },
                {0,  0,  rtheta}
        });
    }

    /**
     * Sets the full R matrix directly (3x3).
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

    // ==================== Rollback replay status ====================
    /** @return whether vision-timestamp rollback replay is enabled */
    public boolean isReplayEnabled() {
        return allowReplay;
    }
    /** @return number of snapshots in the rollback buffer (0 when replay is disabled) */
    public int getReplayBufferSize() {
        return replayBuffer == null ? 0 : replayBuffer.size();
    }

    // ==================== Sigma-point generation ====================
    /**
     * Generates 2n+1 sigma points from the current state and covariance.
     *
     * <p>Algorithm:
     * <ol>
     *   <li>Compute the scaled covariance S = (n+lambda)*P</li>
     *   <li>Cholesky factorize S = L*Lᵀ (with regularized fallback)</li>
     *   <li>sigma_0 = state</li>
     *   <li>sigma_i = state + L_col_i (i=1..n)</li>
     *   <li>sigma_{i+n} = state - L_col_i (i=1..n)</li>
     * </ol>
     *
     * @return double[7][3] sigma-point array
     */
    private double[][] generateSigmaPoints() {
        double[][] sigmaPoints = new double[NUM_SIGMA][N];

        // Scaled covariance: S = (n+lambda) * P = 3 * P
        DMatrixRMaj L = choleskyDecompose(P, nPlusLambda);

        double x0 = state.get(0, 0);
        double y0 = state.get(1, 0);
        double t0 = state.get(2, 0);

        // Sigma point 0: the mean itself.
        sigmaPoints[0][0] = x0;
        sigmaPoints[0][1] = y0;
        sigmaPoints[0][2] = t0;

        // Sigma points 1..N: state + L_col
        // Sigma points N+1..2N: state - L_col
        for (int j = 0; j < N; j++) {
            double lj0 = L.get(0, j);
            double lj1 = L.get(1, j);
            double lj2 = L.get(2, j);

            // +L_col
            sigmaPoints[j + 1][0] = x0 + lj0;
            sigmaPoints[j + 1][1] = y0 + lj1;
            sigmaPoints[j + 1][2] = t0 + lj2;

            // -L_col
            sigmaPoints[j + N + 1][0] = x0 - lj0;
            sigmaPoints[j + N + 1][1] = y0 - lj1;
            sigmaPoints[j + N + 1][2] = t0 - lj2;
        }

        return sigmaPoints;
    }

    // ==================== Cholesky decomposition (with regularized fallback) ====================

    /**
     * Computes the Cholesky factorization L of the scaled covariance matrix such that L*Lᵀ = scale*P.
     *
     * <p>Three-level fallback strategy:
     * <ol>
     *   <li>Factorize scale*P directly</li>
     *   <li>On failure, retry scale*P + 1e-6*I</li>
     *   <li>On further failure, retry scale*P + 1e-3*I</li>
     *   <li>If all fail, return the diagonal fallback sqrt(trace(scale*P)/3)*I</li>
     * </ol>
     *
     * @param Pmat  covariance matrix P
     * @param scale scale factor (n+lambda)
     * @return Cholesky lower-triangular matrix L (3x3)
     */
    private DMatrixRMaj choleskyDecompose(SimpleMatrix Pmat, double scale) {
        CholeskyDecomposition_F64<DMatrixRMaj> chol =
                DecompositionFactory_DDRM.chol(N, true);  // true = lower triangular

        // Attempt 1: factorize directly.
        DMatrixRMaj S = toDenseScaled(Pmat, scale);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // Attempt 2: add small regularization eps_1.
        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_1);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // Attempt 3: add larger regularization eps_2.
        S = toDenseScaledRegularized(Pmat, scale, CHOL_EPS_2);
        if (chol.decompose(S)) {
            return extractL(chol);
        }

        // Degraded fallback: diagonal matrix sqrt(trace(scale*P)/3)*I.
        double trace = scale * (Pmat.get(0, 0) + Pmat.get(1, 1) + Pmat.get(2, 2));
        double diag = Math.sqrt(Math.max(trace / N, 1e-12));

        DMatrixRMaj L = new DMatrixRMaj(N, N);
        L.set(0, 0, diag);
        L.set(1, 1, diag);
        L.set(2, 2, diag);
        return L;
    }

    /** Builds scale*P, copied as a DMatrixRMaj. */
    private DMatrixRMaj toDenseScaled(SimpleMatrix Pmat, double scale) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        result.set(0, 0, scale * Pmat.get(0, 0));
        result.set(0, 1, scale * Pmat.get(0, 1));
        result.set(0, 2, scale * Pmat.get(0, 2));
        result.set(1, 0, scale * Pmat.get(1, 0));
        result.set(1, 1, scale * Pmat.get(1, 1));
        result.set(1, 2, scale * Pmat.get(1, 2));
        result.set(2, 0, scale * Pmat.get(2, 0));
        result.set(2, 1, scale * Pmat.get(2, 1));
        result.set(2, 2, scale * Pmat.get(2, 2));
        return result;
    }

    /** Builds scale*P + eps*I, copied as a DMatrixRMaj. */
    private DMatrixRMaj toDenseScaledRegularized(SimpleMatrix Pmat, double scale, double eps) {
        DMatrixRMaj result = new DMatrixRMaj(N, N);
        result.set(0, 0, scale * Pmat.get(0, 0) + eps);
        result.set(0, 1, scale * Pmat.get(0, 1));
        result.set(0, 2, scale * Pmat.get(0, 2));
        result.set(1, 0, scale * Pmat.get(1, 0));
        result.set(1, 1, scale * Pmat.get(1, 1) + eps);
        result.set(1, 2, scale * Pmat.get(1, 2));
        result.set(2, 0, scale * Pmat.get(2, 0));
        result.set(2, 1, scale * Pmat.get(2, 1));
        result.set(2, 2, scale * Pmat.get(2, 2) + eps);
        return result;
    }

    /** Extracts the lower-triangular matrix L from the Cholesky factorization. */
    private DMatrixRMaj extractL(CholeskyDecomposition_F64<DMatrixRMaj> chol) {
        DMatrixRMaj L = new DMatrixRMaj(N, N);
        chol.getT(L);
        return L;
    }

    // ==================== Covariance computation helpers ====================
    /**
     * Computes the weighted covariance matrix from sigma points.
     * S wc[i]*(X_i - mean)(X_i - mean)ᵀ
     */
    private SimpleMatrix computeCovarianceFromSigma(
            double[][] sigma, double mx, double my, double mtheta) {
        SimpleMatrix result = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = sigma[i][0] - mx;
            double dy = sigma[i][1] - my;
            double dt = normalizeAngle(sigma[i][2] - mtheta);
            double w = wc[i];

            result.set(0, 0, result.get(0, 0) + w * dx * dx);
            result.set(0, 1, result.get(0, 1) + w * dx * dy);
            result.set(0, 2, result.get(0, 2) + w * dx * dt);
            result.set(1, 0, result.get(1, 0) + w * dy * dx);
            result.set(1, 1, result.get(1, 1) + w * dy * dy);
            result.set(1, 2, result.get(1, 2) + w * dy * dt);
            result.set(2, 0, result.get(2, 0) + w * dt * dx);
            result.set(2, 1, result.get(2, 1) + w * dt * dy);
            result.set(2, 2, result.get(2, 2) + w * dt * dt);
        }
        return result;
    }

    /**
     * Computes the cross-covariance matrix Pxz.
     * S wc[i]*(X_i - xMean)(Z_i - zMean)ᵀ
     */
    private SimpleMatrix computeCrossCovariance(
            double[][] xSigma, double xm, double ym, double tm,
            double[][] zSigma, double zxm, double zym, double ztm) {
        SimpleMatrix result = new SimpleMatrix(N, N);
        for (int i = 0; i < NUM_SIGMA; i++) {
            double dx = xSigma[i][0] - xm;
            double dy = xSigma[i][1] - ym;
            double dt = normalizeAngle(xSigma[i][2] - tm);

            double dzx = zSigma[i][0] - zxm;
            double dzy = zSigma[i][1] - zym;
            double dzt = normalizeAngle(zSigma[i][2] - ztm);

            double w = wc[i];

            result.set(0, 0, result.get(0, 0) + w * dx * dzx);
            result.set(0, 1, result.get(0, 1) + w * dx * dzy);
            result.set(0, 2, result.get(0, 2) + w * dx * dzt);
            result.set(1, 0, result.get(1, 0) + w * dy * dzx);
            result.set(1, 1, result.get(1, 1) + w * dy * dzy);
            result.set(1, 2, result.get(1, 2) + w * dy * dzt);
            result.set(2, 0, result.get(2, 0) + w * dt * dzx);
            result.set(2, 1, result.get(2, 1) + w * dt * dzy);
            result.set(2, 2, result.get(2, 2) + w * dt * dzt);
        }
        return result;
    }

    // ==================== Internal utilities ====================
    /**
     * Normalizes an angle to the [-pi, pi] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Safe inversion -- S = Pzz + R can be singular when R is configured too small or numerically degenerate.
     * On inversion failure, adds diagonal jitter and retries, avoiding exceptions or exploding gains.
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
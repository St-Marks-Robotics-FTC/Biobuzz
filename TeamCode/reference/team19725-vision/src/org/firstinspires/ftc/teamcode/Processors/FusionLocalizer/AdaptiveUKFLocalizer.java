package org.firstinspires.ftc.teamcode.Processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF;

/**
 * Fusion localizer -- odometry + Limelight MegaTag1 (vision) + UKF fusion + Hub IMU adaptive Q.
 *
 * <p>Supports two odometry modes:
 * <ul>
 *   <li><b>D2</b> (default): {@link PinpointLocalizer}, standard 2D odometry without ramp compensation</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}, 3D ramp-compensated odometry</li>
 * </ul>
 *
 * <p>Call {@link #update()} once per loop to complete:
 * <ol>
 *   <li>Odometry velocity prediction ({@link UKF#predict})</li>
 *   <li>Limelight vision update (conditional, {@link UKF#update})</li>
 *   <li>Adaptively adjust Q (IMU angular velocity/acceleration) and R (vision stdDev)</li>
 * </ol>
 *
 * <p><b>Adaptive Q policy</b> (D2 differs from D3):
 * <ul>
 *   <li><b>D2</b>: roll/pitch angular acceleration -> impact detection; yaw angular acceleration -> rotational impact</li>
 *   <li><b>D3</b>: roll/pitch angular velocity -> slope-change detection; yaw angular acceleration -> rotational impact</li>
 *   <li>Uncertainty direction is perpendicular to the rotation axis: pitch about body Y -> lift-off along X, roll about body X -> lift-off along Y</li>
 * </ul>
 *
 * <p><b>Adaptive R</b>: per-axis MT1 stdDev -> per-axis independent R matrix
 *
 * <p><b>Vision gating</b>: R adaptation and Mahalanobis-distance gating run only when both
 * {@link MT1Localizer#isValid()} and {@link MT1Localizer#isHiveEstimated()} are true.
 * A false latter means the HIVE tilt for this frame is unsolved or rejected
 * (no tags / no solution / root selection beyond CellUpAngle / attitude cross-check failed / pivot height uncalibrated),
 * in which case {@code MT1Localizer#getPose()} falls back to an uncorrected contaminated pose and the whole frame must be dropped.
 */
@Config
public class AdaptiveUKFLocalizer implements Localizer {

    private final UKF ukf;
    /** Odometry localizer (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    private final boolean useD3;

    // ---- Time base ----
    private double lastTimestamp = 0;

    /** Cached odometry velocity from the most recent update */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ---- Adaptive Q: IMU detection ----
    /** Previous-frame roll rate (rad/s) -- about body X axis (left/right) */
    private double lastRollRate = 0;
    /** Previous-frame pitch rate (rad/s) -- about body Y axis (front/back) */
    private double lastPitchRate = 0;
    /** Previous-frame yaw rate (rad/s) -- about body Z axis (vertical) */
    private double lastYawRate = 0;
    /** Whether the first-frame angular rates were sampled (prevents a bogus (rate - 0) / dt Q boost on the first frame) */
    private boolean ratesInitialized = false;

    /** Q base value (in^2/s) */
    public static double qBase = 0.002;
    /** Q boost factor (runtime state, per instance; must not be static or instances would pollute each other) */
    private double qBoostX = 1.0;
    private double qBoostY = 1.0;
    private double qBoostTheta = 1.0;

    // ---- D2: angular-acceleration thresholds (impact detection) ----
    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;  // rad/s^2 (pitch/roll angular-acceleration threshold)
    public static double JERK_THRESHOLD = 4.0;           // rad/s^2 (yaw angular jerk threshold)

    // ---- D3: angular-velocity thresholds (slope-change detection) ----
    public static double ANGULAR_VEL_THRESHOLD = 1.0;    // rad/s (pitch/roll angular-velocity threshold for slope change)
    public static double VEL_BOOST_MAX = 10.0;            // max angular-velocity Q boost factor (slope change, higher)
    public static double ACCEL_BOOST_MAX = 4.0;         // max angular-acceleration Q boost factor (impact, lower)

    public static double Q_BOOST_MAX = 10.0;
    public static double Q_DECAY = 0.85;

    // ---- Adaptive R: MT1 stdDev ----
    /** Unit conversion: 1 m = 39.3701 in */
    public static double M_TO_INCH = 39.37007874;

    /** stdDev thresholds (inches) -- correspond to the original 0.05 m / 0.15 m */
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;

    /** stdDev thresholds (radians) -- heading-only, correspond to about 2 deg / 10 deg */
    public static double STD_LOW_ANGLE = 0.035;   // rad (≈2°)
    public static double STD_HIGH_ANGLE = 0.175;  // rad (≈10°)

    public static double R_MAX_SCALE = 20.0;

    // ---- Vision distance / tag-count scaling and gating (9/2 improvement) ----
    /** Reference distance (meters); beyond it the vision position R grows quadratically with distance */
    public static double DIST_REF_M = 1.0;
    /** Reference tag count; below it the vision position R is inflated */
    public static double TAG_REF = 2.0;
    /** Maximum inflation factor for R when too few tags are visible */
    public static double TAG_SCALE_MAX = 4.0;
    /** Mahalanobis-distance gating threshold (dimensionless) */
    public static double GATE_THRESHOLD = 4.0;

    /** Cached R base value (for dashboard display, per instance) */
    private double rBase = 0.01;

    // ==================== Construction ====================

    /**
     * D2 mode constructor (standard 2D odometry).
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu")
     * @param initialPose    initial pose (x, y, heading)
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, false, false);
    }

    /**
     * Simplified D2 constructor: uses the default IMU name "imu" and initial pose (0, 0, 0).
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", new Pose2d(0, 0, 0));
    }

    /**
     * D3 mode constructor (3D ramp-compensated odometry).
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu"), shared by odometry and adaptQ
     * @param initialPose    initial pose (x, y, heading)
     * @param useD3          true for D3 ramp-compensated odometry, false for standard 2D odometry
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, useD3, false);
    }

    /**
     * Full constructor; vision timestamp rollback replay can be enabled.
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu"), shared by odometry and adaptQ
     * @param initialPose    initial pose (x, y, heading)
     * @param useD3          true for D3 ramp-compensated odometry, false for standard 2D odometry
     * @param allowReplay    whether to enable vision timestamp rollback replay (default false)
     */
    public AdaptiveUKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3,
                           boolean allowReplay) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble(), allowReplay);
        if (useD3) {
            this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        } else {
            this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        }
        this.mt1 = new MT1Localizer(limelight);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.useD3 = useD3;
        this.lastTimestamp = getNow();
    }

    // ==================== Main loop ====================

    /**
     * Call once per loop; completes:
     * <ol>
     *   <li>Odometry update -> velocity</li>
     *   <li>IMU angular-velocity/acceleration check -> adaptive Q (D2/D3 policies differ)</li>
     *   <li>UKF prediction</li>
     *   <li>Limelight update -> adaptive R (3x3 matrix) + UKF update</li>
     * </ol>
     *
     * @return current velocity estimate
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. Odometry velocity ----
        lastVel = odom.update();

        // ---- 2. IMU check -> adaptive Q matrix (D2/D3 policies differ) ----
        ukf.setQ(adaptQ(dt));

        // ---- 3. UKF prediction ----
        ukf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 4. MT1 vision -> adaptive R matrix + gating + UKF update ----
        mt1.update();
        // A false isHiveEstimated() means the HIVE solution for this frame failed or was rejected (no tags / no solution /
        // root selection beyond CellUpAngle / attitude cross-check failed / pivot height uncalibrated); mt1.getPose() is then
        // an uncorrected contaminated pose and the whole frame is dropped.
        if (mt1.isValid() && mt1.isHiveEstimated()) {
            ukf.setR(adaptR());
            Pose2d visionPose = mt1.getPose();              // (inches, inches, radians)
            if (ukf.gateVision(
                    visionPose.position.x,
                    visionPose.position.y,
                    visionPose.heading.toDouble(),
                    GATE_THRESHOLD
            )) {
                ukf.update(
                        visionPose.position.x,                  // inches
                        visionPose.position.y,                  // inches
                        visionPose.heading.toDouble(),          // radians
                        // Rollback replay must share the predict snapshot time base (System.nanoTime()); otherwise use the Limelight hardware timestamp
                        ukf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
                );
            }
        }

        return lastVel;
    }

    // ==================== Adaptive Q (SimpleMatrix output) ====================

    /**
     * Builds a 3x3 diagonal Q matrix from IMU angular velocity/acceleration.
     *
     * <p><b>D2 mode</b> (angular acceleration):
     * <ul>
     *   <li><b>x, y</b>: roll/pitch angular acceleration -> rotate into the field frame -> cross-map (pitch->X, roll->Y)</li>
     *   <li><b>theta</b>: yaw angular acceleration -> rotational-impact detection</li>
     * </ul>
     *
     * <p><b>D3 mode</b> (angular velocity):
     * <ul>
     *   <li><b>x, y</b>: roll/pitch angular velocity -> rotate into the field frame -> cross-map (pitch->X, roll->Y)</li>
     *   <li><b>theta</b>: yaw angular acceleration -> rotational-impact detection</li>
     * </ul>
     *
     * @param dt frame interval (seconds)
     * @return 3x3 diagonal process-noise covariance matrix Q
     */
    private SimpleMatrix adaptQ(double dt) {
        double safeDt = Math.max(dt, 1e-6);

        // Use the compensated heading (including the setPose offset), not the raw pinpoint heading:
        // rotating pitch/roll rates into the field frame needs the robot's true heading on the field;
        // compensated heading = txWorldPinpoint.heading + pinpointTheta, i.e. the pose heading tracked by the UKF
        double theta = getPose().heading.toDouble();
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        if (hubImu != null) {
            AngularVelocity angVel = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);
            if (angVel != null) {
                // IMU body frame (X=right, Y=forward) -> RR body frame (X=forward, Y=left):
                //   xRotationRate (about Robot X/right) -> pitch (about RR Y/left, fore-aft tilt)
                //   yRotationRate (about Robot Y/forward) -> roll (about RR X/forward, left-right tilt)
                double pitchRate = angVel.xRotationRate;
                double rollRate  = angVel.yRotationRate;
                double yawRate   = angVel.zRotationRate;  // only used for rate-of-change detection, not as an absolute value

                if (!ratesInitialized) {
                    // First frame only samples the previous rates to avoid a bogus (rate - 0) / dt spike that would wrongly boost Q
                    lastPitchRate = pitchRate;
                    lastRollRate  = rollRate;
                    lastYawRate   = yawRate;
                    ratesInitialized = true;
                } else {
                // ---- pitch/roll angular acceleration (impact) ----
                double pitchAccel = (pitchRate - lastPitchRate) / safeDt;
                double rollAccel  = (rollRate  - lastRollRate)  / safeDt;

                // Body frame -> field frame rotation (Rz(theta))
                // angular velocity vector in body: [rollRate, pitchRate]^T
                double fieldX = rollAccel * cosT - pitchAccel * sinT;
                double fieldY = rollAccel * sinT + pitchAccel * cosT;

                if (useD3) {
                    // ========== D3: angular velocity (slope-change detection) ==========
                    // Rotate into the field frame, then cross-map:
                    // pitch about body Y -> wheels lift along X -> X uncertainty (takes the fieldY component)
                    // roll about body X -> wheels lift along Y -> Y uncertainty (takes the fieldX component)
                    double fieldVelX = rollRate * cosT - pitchRate * sinT;
                    double fieldVelY = rollRate * sinT + pitchRate * cosT;

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldVelY), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldVelX), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);

                    // yaw: angular acceleration (rotational impact)
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);

                } else {
                    // ========== D2: angular acceleration (impact detection) ==========
                    // Cross-map: pitch about body Y -> wheels lift along X -> X uncertainty (takes the fieldY component)
                    //           roll about body X -> wheels lift along Y -> Y uncertainty (takes the fieldX component)

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldY), ANGULAR_ACCEL_THRESHOLD);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldX), ANGULAR_ACCEL_THRESHOLD);

                    // yaw angular acceleration
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);
                }

                lastPitchRate = pitchRate;
                lastRollRate  = rollRate;
                lastYawRate   = yawRate;
                }
            }
        }

        // Build the 3x3 diagonal Q matrix
        SimpleMatrix Q = new SimpleMatrix(3, 3);
        Q.set(0, 0, qBase * qBoostX);
        Q.set(1, 1, qBase * qBoostY);
        Q.set(2, 2, qBase * qBoostTheta);
        return Q;
    }

    // ==================== Adaptive R (SimpleMatrix output) ====================

    /**
     * Builds a 3x3 diagonal R matrix from the per-axis MT1 stdDevs.
     *
     * <p>Each axis maps independently (std -> R):
     * <pre>
     *   std &lt; STD_LOW (0.05)   →  R = 0.01
     *   std &gt; STD_HIGH (0.15)  →  R = 0.01 x R_MAX_SCALE
     *   in between → linear interpolation
     * </pre>
     *
     * @return 3x3 diagonal measurement-noise covariance matrix R
     */
    private SimpleMatrix adaptR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (meters, degrees)

        // Distance and tag-count scaling: vision position error grows far from tags / with too few tags, so inflate position R
        double distF = computeDistFactor();
        double tagF = computeTagFactor();

        // Position: stdDev from meters -> inches to match the UKF state units
        double rX = mapStdToR(stdDevs[0] * M_TO_INCH) * distF * tagF;
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH) * distF * tagF;
        // Heading: yaw from degrees -> radians, using the heading-only thresholds
        double rTheta = mapStdToRAngle(Math.toRadians(stdDevs[5]));

        // Cache rBase for dashboard display
        rBase = rTheta;

        SimpleMatrix R = new SimpleMatrix(3, 3);
        R.set(0, 0, rX);
        R.set(1, 1, rY);
        R.set(2, 2, rTheta);
        return R;
    }

    // ==================== Helpers ====================

    /**
     * Updates the per-axis Q boost factor (with a custom maxBoost).
     *
     * @param current   current boost value
     * @param magnitude signal magnitude
     * @param threshold trigger threshold
     * @param maxBoost  maximum boost cap
     * @return updated boost value
     */
    private double updateBoost(double current, double magnitude, double threshold, double maxBoost) {
        if (magnitude > threshold) {
            return Math.min(maxBoost, current * (1.0 + magnitude / threshold));
        } else {
            return Math.max(1.0, current * Q_DECAY);
        }
    }

    /**
     * Updates the per-axis Q boost factor (D2 compatibility, uses the default Q_BOOST_MAX).
     */
    private double updateBoost(double current, double magnitude, double threshold) {
        return updateBoost(current, magnitude, threshold, Q_BOOST_MAX);
    }

    /**
     * Maps one position-axis std to R (three stages: trust / linear interpolation / distrust).
     * Uses the position thresholds (STD_LOW_INCH / STD_HIGH_INCH).
     *
     * @param std standard deviation (inches)
     */
    private double mapStdToR(double std) {
        return mapStdToR(std, STD_LOW_INCH, STD_HIGH_INCH);
    }

    /**
     * Maps a heading-axis std to R, using the heading-only thresholds.
     *
     * @param std angular standard deviation (radians)
     */
    private double mapStdToRAngle(double std) {
        return mapStdToR(std, STD_LOW_ANGLE, STD_HIGH_ANGLE);
    }

    /**
     * Generic std -> R mapping (three stages: trust / linear interpolation / distrust).
     */
    private double mapStdToR(double std, double low, double high) {
        if (std <= low) {
            return 0.01;
        }
        // Linear growth; beyond high it keeps growing linearly without clamping the upper bound (9/2 improvement)
        double t = (std - low) / (high - low);
        return 0.01 * (1.0 + t * (R_MAX_SCALE - 1.0));
    }

    /**
     * Distance scale factor for vision position measurement noise (grows quadratically far from tags).
     */
    private double computeDistFactor() {
        double dist = mt1.getAvgDist();  // meters
        if (Double.isNaN(dist) || Double.isInfinite(dist) || dist <= DIST_REF_M) {
            return 1.0;
        }
        double ratio = dist / DIST_REF_M;
        return ratio * ratio;
    }

    /**
     * Tag-count scale factor for vision position measurement noise (inflated when too few tags are seen).
     */
    private double computeTagFactor() {
        int tags = mt1.getTagCount();
        if (tags >= TAG_REF) {
            return 1.0;
        }
        if (tags <= 0) {
            return TAG_SCALE_MAX;
        }
        double ratio = TAG_REF / tags;
        return Math.min(TAG_SCALE_MAX, ratio);
    }

    // ==================== Localizer interface ====================

    /** Sets the localizer pose. */
    @Override
    public void setPose(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
        ratesInitialized = false;
    }

    // ==================== Outputs ====================

    /** @return fused pose {x, y, heading} (inches, inches, radians) */
    @Override
    public Pose2d getPose() {
        double[] pose = ukf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return raw UKF instance */
    public UKF getUKF() { return ukf; }

    /** @return MT1 vision localizer */
    public MT1Localizer getMT1() { return mt1; }

    /** @return odometry localizer (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /** @return whether D3 mode is active */
    public boolean isD3() { return useD3; }

    /** @return Q boost factor along x */
    public double getQBoostX() { return qBoostX; }
    /** @return Q boost factor along y */
    public double getQBoostY() { return qBoostY; }
    /** @return Q boost factor along theta */
    public double getQBoostTheta() { return qBoostTheta; }

    // ==================== Reset ====================

    /** Resets localization to the given pose. */
    public void reset(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
        lastTimestamp = getNow();
        qBoostX = 1.0;
        qBoostY = 1.0;
        qBoostTheta = 1.0;
        lastPitchRate = 0;
        lastRollRate = 0;
        lastYawRate = 0;
        ratesInitialized = false;
    }

    // ==================== Internal utilities ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}
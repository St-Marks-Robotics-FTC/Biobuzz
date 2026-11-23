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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.ejml.simple.SimpleMatrix;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.utility.filter.EKF.EKF;

/**
 * Fusion localizer: odometry + Limelight MegaTag1 (vision) + EKF fusion + Hub IMU adaptive Q.
 *
 * <p>Supports two odometry modes:
 * <ul>
 *   <li><b>D2</b> (default): {@link PinpointLocalizer}, standard 2D odometry without ramp compensation</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}, 3D ramp-compensated odometry</li>
 * </ul>
 *
 * <p>Call {@link #update()} once per frame to complete:
 * <ol>
 *   <li>Odometry velocity prediction ({@link EKF#predict})</li>
 *   <li>Limelight vision update (conditional, {@link EKF#update})</li>
 *   <li>Adaptive tuning of Q (IMU angular velocity/angular acceleration) and R (vision stdDev)</li>
 * </ol>
 *
 * <p><b>Adaptive Q policy</b> (differs between D2 and D3):
 * <ul>
 *   <li><b>D2</b>: roll/pitch angular acceleration for impact detection; yaw angular acceleration for rotational impact</li>
 *   <li><b>D3</b>: roll/pitch angular velocity for slope-change detection; yaw angular acceleration for rotational impact</li>
 *   <li>Uncertainty axis is perpendicular to the rotation axis: pitch about body Y lifts wheels along X, roll about body X lifts wheels along Y</li>
 * </ul>
 *
 * <p><b>R adaptation</b>: per-axis MT1 stdDev values map to independent per-axis entries of the R matrix
 *
 * <p><b>Vision gating</b>: R adaptation and Mahalanobis gating run only when both {@link MT1Localizer#isValid()} and {@link MT1Localizer#isHiveEstimated()}
 * are true. A false latter means the HIVE tilt for this frame has no solution or was rejected
 * (no tags / no solution / root selection beyond CellUpAngle / attitude cross-check failed / pivot height uncalibrated),
 * in which case {@code MT1Localizer#getPose()} falls back to an uncorrected contaminated pose, so the whole frame must be dropped.
 */

// TODO: forward MT1Localizer HIVE tilt/state observations to RobotPosition (rejecting invalid vision updates is already implemented)
@Config
public class AdaptiveEKFLocalizer implements Localizer {

    private final EKF ekf;
    /** Odometry localizer (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final IMU hubImu;
    private final boolean useD3;

    // ---- Time base ----
    private double lastTimestamp = 0;

    /** Cached odometry velocity from the latest frame */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    /**
     * IMU attitude sampled this frame: passed down to the D3 odometry (ramp-compensated pitch/roll),
     * and served up to RobotPosition getYaw / getYawPitchRollAngles readers, so the IMU is read exactly once per frame.
     */
    private YawPitchRollAngles imuAngles =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    /** IMU angular velocity (rad/s) sampled this frame, used for Q adaptation */
    private AngularVelocity imuAngularVelocity;

    // ---- Q adaptation: IMU detection ----
    /** Previous-frame roll rate (rad/s), about body X (lateral) */
    private double lastRollRate = 0;
    /** Previous-frame pitch rate (rad/s), about body Y (longitudinal) */
    private double lastPitchRate = 0;
    /** Previous-frame yaw rate (rad/s), about body Z (vertical) */
    private double lastYawRate = 0;
    /** Whether first-frame rates were sampled (prevents a spurious (rate - 0) / dt Q boost on the first frame) */
    private boolean ratesInitialized = false;

    /** Q base value (in^2/s) */
    public static double qBase = 0.002;
    /** Q boost factors (runtime state, per instance; must not be static or instances would contaminate each other) */
    private double qBoostX = 1.0;
    private double qBoostY = 1.0;
    private double qBoostTheta = 1.0;

    // ---- D2: angular-acceleration thresholds (impact detection) ----
    public static double ANGULAR_ACCEL_THRESHOLD = 5.0;  // rad/s^2 (pitch/roll angular-acceleration threshold)
    public static double JERK_THRESHOLD = 4.0;           // rad/s^2 (yaw angular jerk threshold)

    // ---- D3: angular-velocity thresholds (slope-change detection) ----
    public static double ANGULAR_VEL_THRESHOLD = 1.0;    // rad/s (pitch/roll angular-velocity threshold for slope changes)
    public static double VEL_BOOST_MAX = 10.0;            // Max Q boost factor for angular velocity (slope changes, higher)
    public static double ACCEL_BOOST_MAX = 4.0;         // Max Q boost factor for angular acceleration (impacts, lower)

    public static double Q_BOOST_MAX = 10.0;
    public static double Q_DECAY = 0.85;

    // ---- R adaptation: MT1 stdDev ----
    /** Unit conversion: 1 m = 39.3701 in */
    public static double M_TO_INCH = 39.37007874;

    /** stdDev thresholds (inches), originally 0.05 m / 0.15 m */
    public static double STD_LOW_INCH = 2.0;
    public static double STD_HIGH_INCH = 6.0;

    /** stdDev thresholds (radians) for the heading component, about 2 deg / 10 deg */
    public static double STD_LOW_ANGLE = 0.035;   // rad (about 2 deg)
    public static double STD_HIGH_ANGLE = 0.175;  // rad (about 10 deg)

    public static double R_MAX_SCALE = 20.0;

    // ---- Vision distance / tag-count scaling and gating (9/2 update) ----
    /** Reference distance (m); beyond it the vision position R grows quadratically with distance */
    public static double DIST_REF_M = 1.0;
    /** Reference tag count; below it the vision position R is inflated */
    public static double TAG_REF = 2.0;
    /** Max inflation factor for R when too few tags are visible */
    public static double TAG_SCALE_MAX = 4.0;
    /** Mahalanobis gating threshold (dimensionless) */
    public static double GATE_THRESHOLD = 4.0;

    /** Cached R base value (for dashboard display, per instance) */
    private double rBase = 0.01;

    // ==================== Constructors ====================

    /**
     * D2 constructor (standard 2D odometry), defaults to red alliance (Limelight pipeline 0).
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu")
     * @param initialPose    initial pose (x, y, heading)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, false, TeamColor.RED);
    }

    /**
     * D2 simplified constructor: default IMU name "imu", initial pose (0,0,0), default red alliance.
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, "imu", new Pose2d(0, 0, 0), false, TeamColor.RED);
    }

    /**
     * D3 constructor (3D ramp-compensated odometry), defaults to red alliance (Limelight pipeline 0).
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu"), shared by the odometry and adaptQ
     * @param initialPose    initial pose (x, y, heading)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, useD3, TeamColor.RED);
    }

    /**
     * Full constructor.
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu"), shared by the odometry and adaptQ
     * @param initialPose    initial pose (x, y, heading)
     * @param useD3          true for D3 ramp-compensated odometry, false for standard 2D odometry
     * @param teamColor      alliance color: red loads Limelight pipeline 0, blue loads pipeline 1
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3,
                           TeamColor teamColor) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, useD3, teamColor, false);
    }

    /**
     * Full constructor with optional vision timestamp rollback replay.
     *
     * @param hardwareMap    hardware map
     * @param limelight      started Limelight3A instance
     * @param imuDeviceName  IMU device name (e.g. "imu"), shared by the odometry and adaptQ
     * @param initialPose    initial pose (x, y, heading)
     * @param useD3          true for D3 ramp-compensated odometry, false for standard 2D odometry
     * @param teamColor      alliance color: red loads Limelight pipeline 0, blue loads pipeline 1
     * @param allowReplay    whether to enable vision timestamp rollback replay (default false)
     */
    public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                           String imuDeviceName, Pose2d initialPose, boolean useD3,
                           TeamColor teamColor, boolean allowReplay) {
        this.ekf = new EKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble(), allowReplay);
        if (useD3) {
            this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        } else {
            this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        }
        this.mt1 = new MT1Localizer(limelight, teamColor);
        this.hubImu = hardwareMap.get(IMU.class, imuDeviceName);
        this.useD3 = useD3;
        this.lastTimestamp = getNow();
    }

    // ==================== Core loop ====================

    /**
     * Called once per frame, completing:
     * <ol>
     *   <li>Odometry update to get velocity</li>
     *   <li>IMU angular velocity/acceleration check to adapt Q (D2/D3 policies differ)</li>
     *   <li>EKF prediction</li>
     *   <li>Limelight update to adapt R (3x3 matrix) + EKF update</li>
     * </ol>
     *
     * @return current velocity estimate
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        // ---- 1. IMU sample: the only one this frame. Attitude goes down to the odometry and up to RobotPosition; angular velocity feeds Q adaptation ----
        imuAngles = hubImu.getRobotYawPitchRollAngles();
        imuAngularVelocity = hubImu.getRobotAngularVelocity(AngleUnit.RADIANS);

        // ---- 2. Odometry velocity (D3 reuses this frame's IMU attitude instead of reading the IMU again) ----
        lastVel = useD3
                ? ((PinpointD3Localizer) odom).update(imuAngles)
                : odom.update();

        // ---- 3. IMU angular velocity to adaptive Q matrix (D2/D3 policies differ) ----
        ekf.setQ(adaptQ(dt));

        // ---- 4. EKF prediction ----
        ekf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 5. MT1 vision to adaptive R matrix + gating + EKF update ----
        mt1.update();
        // A false isHiveEstimated() means this frame's HIVE solution failed or was rejected (no tags / no solution / root selection beyond CellUpAngle /
        // attitude cross-check failed / pivot height uncalibrated); mt1.getPose() is then an uncorrected contaminated pose, so drop it outright
        if (mt1.isValid() && mt1.isHiveEstimated()) {
            ekf.setR(adaptR());
            Pose2d visionPose = mt1.getPose();              // (inches, inches, radians)
            if (ekf.gateVision(
                    visionPose.position.x,
                    visionPose.position.y,
                    visionPose.heading.toDouble(),
                    GATE_THRESHOLD
            )) {
                ekf.update(
                        visionPose.position.x,                  // inches
                        visionPose.position.y,                  // inches
                        visionPose.heading.toDouble(),          // radians
                        // Rollback replay must share the predict snapshot time base (System.nanoTime()); otherwise keep the Limelight hardware timestamp
                        ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
                );
            }
        }

        return lastVel;
    }

    // ==================== Adaptive Q (SimpleMatrix output) ====================

    /**
     * Builds the 3x3 diagonal Q matrix from IMU angular velocity/angular acceleration.
     *
     * <p><b>D2 mode</b> (angular acceleration):
     * <ul>
     *   <li><b>x, y</b>: roll/pitch angular acceleration, rotated into the field frame, then cross-mapped (pitch to X, roll to Y)</li>
     *   <li><b>theta</b>: yaw angular acceleration detects rotational impacts</li>
     * </ul>
     *
     * <p><b>D3 mode</b> (angular velocity):
     * <ul>
     *   <li><b>x, y</b>: roll/pitch angular velocity, rotated into the field frame, then cross-mapped (pitch to X, roll to Y)</li>
     *   <li><b>theta</b>: yaw angular acceleration detects rotational impacts</li>
     * </ul>
     *
     * @param dt frame interval (seconds)
     * @return 3x3 diagonal process-noise covariance matrix Q
     */
    private SimpleMatrix adaptQ(double dt) {
        double safeDt = Math.max(dt, 1e-6);

        // Use the compensated heading (including the setPose offset), not the raw pinpoint heading:
        // rotating pitch/roll rates into the field frame needs the robot's true field heading,
        // and the compensated heading = txWorldPinpoint.heading + pinpointTheta, which is the heading the EKF tracks
        double theta = getPose().heading.toDouble();
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);

        if (hubImu != null) {
            AngularVelocity angVel = imuAngularVelocity;  // Sampled by update() this frame; do not read hardware again inside adaptQ
            if (angVel != null) {
                // IMU body frame (X=right, Y=forward) to RR body frame (X=forward, Y=left):
                //   xRotationRate (about robot X/right) acts as pitch (about RR Y/left, fore-aft tilt)
                //   yRotationRate (about robot Y/forward) acts as roll (about RR X/forward, lateral tilt)
                double pitchRate = angVel.xRotationRate;
                double rollRate  = angVel.yRotationRate;
                double yawRate   = angVel.zRotationRate;  // Only the angular-velocity rate of change is used, not its absolute value

                if (!ratesInitialized) {
                    // First frame only records the previous rates, avoiding a bogus large angular acceleration from (rate - 0) / dt that would wrongly boost Q
                    lastPitchRate = pitchRate;
                    lastRollRate  = rollRate;
                    lastYawRate   = yawRate;
                    ratesInitialized = true;
                } else {
                // ---- pitch/roll angular acceleration (impacts) ----
                double pitchAccel = (pitchRate - lastPitchRate) / safeDt;
                double rollAccel  = (rollRate  - lastRollRate)  / safeDt;

                // Body frame to field frame rotation (Rz(theta))
                // angular velocity vector in body: [rollRate, pitchRate]^T
                double fieldX = rollAccel * cosT - pitchAccel * sinT;
                double fieldY = rollAccel * sinT + pitchAccel * cosT;

                if (useD3) {
                    // ========== D3: angular velocity (slope-change detection) ==========
                    // Rotate into the field frame, then cross-map:
                    // pitch about body Y lifts wheels along X, raising X uncertainty (take the fieldY component)
                    // roll about body X lifts wheels along Y, raising Y uncertainty (take the fieldX component)
                    double fieldVelX = rollRate * cosT - pitchRate * sinT;
                    double fieldVelY = rollRate * sinT + pitchRate * cosT;

                    qBoostX = updateBoost(qBoostX, Math.abs(fieldVelY), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);
                    qBoostY = updateBoost(qBoostY, Math.abs(fieldVelX), ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX);

                    // yaw: angular acceleration (rotational impact)
                    double yawAccel = Math.abs((yawRate - lastYawRate) / safeDt);
                    qBoostTheta = updateBoost(qBoostTheta, yawAccel, JERK_THRESHOLD);

                } else {
                    // ========== D2: angular acceleration (impact detection) ==========
                    // Cross-map: pitch about body Y lifts wheels along X, raising X uncertainty (take the fieldY component)
                    //           roll about body X lifts wheels along Y, raising Y uncertainty (take the fieldX component)

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
     * Builds the 3x3 diagonal R matrix from the per-axis MT1 stdDev values.
     *
     * <p>Each axis is mapped independently (std to R):
     * <pre>
     *   std &lt; STD_LOW (0.05)   -> R = 0.01
     *   std &gt; STD_HIGH (0.15)  -> R = 0.01 * R_MAX_SCALE
     *   middle: linear interpolation
     * </pre>
     *
     * @return 3x3 diagonal measurement-noise covariance matrix R
     */
    private SimpleMatrix adaptR() {
        double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (meters, degrees)

        // Distance and tag-count scaling: far from tags or too few tags means large vision position error, so inflate the position R
        double distF = computeDistFactor();
        double tagF = computeTagFactor();

        // Position: convert stdDev from meters to inches to match the EKF state units
        double rX = mapStdToR(stdDevs[0] * M_TO_INCH) * distF * tagF;
        double rY = mapStdToR(stdDevs[1] * M_TO_INCH) * distF * tagF;
        // Heading: convert yaw from degrees to radians and use the angle-specific thresholds
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
     * Updates the per-axis Q boost factor (D2-compatible, uses the default Q_BOOST_MAX).
     */
    private double updateBoost(double current, double magnitude, double threshold) {
        return updateBoost(current, magnitude, threshold, Q_BOOST_MAX);
    }

    /**
     * Maps one axis std to R (three segments: trusted / linear interpolation / distrusted).
     * Uses the position thresholds (STD_LOW_INCH / STD_HIGH_INCH).
     *
     * @param std standard deviation (inches)
     */
    private double mapStdToR(double std) {
        return mapStdToR(std, STD_LOW_INCH, STD_HIGH_INCH);
    }

    /**
     * Maps the heading-axis std to R using the angle-specific thresholds.
     *
     * @param std heading standard deviation (radians)
     */
    private double mapStdToRAngle(double std) {
        return mapStdToR(std, STD_LOW_ANGLE, STD_HIGH_ANGLE);
    }

    /**
     * Generic std-to-R mapping (three segments: trusted / linear interpolation / distrusted).
     */
    private double mapStdToR(double std, double low, double high) {
        if (std <= low) {
            return 0.01;
        }
        // Grows linearly; above high it keeps growing linearly without an upper cap (9/2 update)
        double t = (std - low) / (high - low);
        return 0.01 * (1.0 + t * (R_MAX_SCALE - 1.0));
    }

    /**
     * Distance scale factor for the vision position measurement noise (grows quadratically when far from tags).
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
     * Tag-count scale factor for the vision position measurement noise (inflated when too few tags are visible).
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
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
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
        double[] pose = ekf.getPose();
        return new Pose2d(pose[0], pose[1], pose[2]);
    }

    /** @return raw EKF instance */
    public EKF getEKF() { return ekf; }

    /** @return MT1 vision localizer */
    public MT1Localizer getMT1() { return mt1; }

    /** @return odometry localizer (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    public Localizer getOdom() { return odom; }

    /**
     * @return IMU attitude sampled by {@link #update()} this frame (yaw/pitch/roll).
     *         The IMU is read exactly once per frame here in the fusion chain; RobotPosition takes it via this accessor
     *         instead of reading the hardware itself.
     */
    public YawPitchRollAngles getImuAngles() { return imuAngles; }

    /** @return whether this is D3 mode */
    public boolean isD3() { return useD3; }

    /** @return x-axis Q boost factor */
    public double getQBoostX() { return qBoostX; }
    /** @return y-axis Q boost factor */
    public double getQBoostY() { return qBoostY; }
    /** @return heading Q boost factor */
    public double getQBoostTheta() { return qBoostTheta; }

    // ==================== Reset ====================

    /** Resets localization to the given pose. */
    public void reset(Pose2d pose) {
        ekf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
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

    // ==================== Internal helpers ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}
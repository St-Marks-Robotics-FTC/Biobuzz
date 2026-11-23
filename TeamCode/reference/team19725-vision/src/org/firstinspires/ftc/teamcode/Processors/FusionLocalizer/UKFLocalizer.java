package org.firstinspires.ftc.teamcode.Processors.FusionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Processors.D3Localizer.PinpointD3Localizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.utility.filter.UKF.UKF;

/**
 * Baseline UKF localizer -- Pinpoint (odometry) + Limelight MegaTag1 (vision) + UKF fusion.
 *
 * <p>Supports two odometry modes:
 * <ul>
 *   <li><b>D2</b> (default): {@link PinpointLocalizer}, standard 2D odometry, no ramp compensation</li>
 *   <li><b>D3</b>: {@link PinpointD3Localizer}, 3D ramp-compensated odometry, requires Hub IMU</li>
 * </ul>
 *
 * <p>As the <b>fixed-weight baseline</b>, differences from {@link AdaptiveUKFLocalizer}:
 * <ul>
 *   <li>Q and R are fixed values (constructor defaults, rewritten every frame), with no IMU/vision adaptation</li>
 *   <li>Vision updates skip Mahalanobis gating and only apply validity filtering
 *       ({@link MT1Localizer#isValid()} and {@link MT1Localizer#isHiveEstimated()},
 *       when the latter is false, {@code MT1Localizer#getPose()} falls back to the uncorrected contaminated raw pose)</li>
 *   <li>No IMU hardware needed (D2 mode)</li>
 * </ul>
 *
 * <p>Call {@link #update()} once per frame to:
 * <ol>
 *   <li>Update odometry -> get velocity</li>
 *   <li>UKF predict (fixed Q)</li>
 *   <li>When Limelight is valid -> UKF update (fixed R)</li>
 * </ol>
 */
@Config
public class UKFLocalizer implements Localizer {

    private final UKF ukf;
    /** Odometry localizer (D2: PinpointLocalizer, D3: PinpointD3Localizer) */
    private final Localizer odom;
    private final MT1Localizer mt1;
    private final boolean useD3;

    // ---- Fixed Q/R parameters (position and heading independent) ----
    /** Process noise -- position (in^2/s) */
    public static double QbasePos = 0.01;
    /** Process noise -- heading (rad^2/s) */
    public static double QbaseAngle = 0.01;
    /** Measurement noise -- position (in^2) */
    public static double RbasePos = 0.01;
    /** Measurement noise -- heading (rad^2) */
    public static double RbaseAngle = 0.05;

    /** Most recent odometry velocity cache */
    private PoseVelocity2d lastVel = new PoseVelocity2d(new Vector2d(0, 0), 0);

    // ==================== Construction ====================

    /**
     * D2 mode constructor (standard 2D odometry).
     *
     * @param hardwareMap  hardware map
     * @param limelight    started Limelight3A instance
     * @param initialPose  initial pose (x, y, heading)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose) {
        this(hardwareMap, limelight, initialPose, false);
    }

    /**
     * D2 mode constructor (standard 2D odometry) with optional vision timestamp rollback replay.
     *
     * @param hardwareMap  hardware map
     * @param limelight    started Limelight3A instance
     * @param initialPose  initial pose (x, y, heading)
     * @param allowReplay  whether to enable vision timestamp rollback replay (default false)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose,
                        boolean allowReplay) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble(), allowReplay);
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = false;
    }

    /**
     * D2 mode simplified constructor: initial pose (0, 0, 0).
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight) {
        this(hardwareMap, limelight, new Pose2d(0, 0, 0), false);
    }

    /**
     * D3 mode constructor (3D ramp-compensated odometry).
     *
     * @param hardwareMap   hardware map
     * @param limelight     started Limelight3A instance
     * @param imuDeviceName Hub IMU device name (e.g. "imu")
     * @param initialPose   initial pose (x, y, heading)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                        String imuDeviceName, Pose2d initialPose) {
        this(hardwareMap, limelight, imuDeviceName, initialPose, false);
    }

    /**
     * D3 mode constructor (3D ramp-compensated odometry) with optional vision timestamp rollback replay.
     *
     * @param hardwareMap   hardware map
     * @param limelight     started Limelight3A instance
     * @param imuDeviceName Hub IMU device name (e.g. "imu")
     * @param initialPose   initial pose (x, y, heading)
     * @param allowReplay   whether to enable vision timestamp rollback replay (default false)
     */
    public UKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                        String imuDeviceName, Pose2d initialPose, boolean allowReplay) {
        this.ukf = new UKF(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble(), allowReplay);
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);
        this.odom = new PinpointD3Localizer(hardwareMap, 0.001999, imuDeviceName, initialPose);
        this.mt1 = new MT1Localizer(limelight);
        this.useD3 = true;
    }

    // ==================== Main loop ====================
    /**
     * Call once per frame to:
     * <ol>
     *   <li>Update odometry -> get velocity</li>
     *   <li>UKF predict (fixed Q)</li>
     *   <li>When Limelight is valid -> UKF update (fixed R)</li>
     * </ol>
     *
     * @return current velocity estimate
     */
    @Override
    public PoseVelocity2d update() {
        double now = getNow();

        // ---- 1. Odometry velocity ----
        lastVel = odom.update();

        // ---- 2. Fixed Q/R (baseline: rewrite constants every frame, no adaptation) ----
        ukf.setQ(QbasePos, QbasePos, QbaseAngle);
        ukf.setR(RbasePos, RbasePos, RbaseAngle);

        // ---- 3. UKF predict (using fixed Q) ----
        ukf.predict(lastVel.linearVel.x, lastVel.linearVel.y, lastVel.angVel, now);

        // ---- 4. MT1 vision -> UKF update (fixed R, no gating, validity filtering only) ----
        mt1.update();
        // Consistent with AdaptiveUKFLocalizer: isHiveEstimated() == false means the HIVE tilt solution failed or was rejected this frame
        // (no tags / no solution / root selection above CellUpAngle / pose cross-check failed / pivot height uncalibrated),
        // in which case mt1.getPose() is an uncorrected contaminated pose and must not be used as a measurement
        if (mt1.isValid() && mt1.isHiveEstimated()) {
            Pose2d visionPose = mt1.getPose();              // (inches, inches, radians)
            ukf.update(
                    visionPose.position.x,                  // inches
                    visionPose.position.y,                  // inches
                    visionPose.heading.toDouble(),          // radians
                    // Rollback replay must share the predict snapshot time base (System.nanoTime()); otherwise use the Limelight hardware timestamp
                    ukf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
            );
        }

        return lastVel;
    }

    // ==================== Localizer interface ====================
    /** Set the localizer pose. */
    @Override
    public void setPose(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== Q/R setter interface ====================
    /**
     * Allow manually adjusting the Q matrix.
     * Note: update() overwrites it every frame with constants such as {@link #QbasePos}.
     */
    public void setQ(double qx, double qy, double qtheta) {
        ukf.setQ(qx, qy, qtheta);
    }

    /**
     * Allow manually adjusting the R matrix.
     * Note: update() overwrites it every frame with constants such as {@link #RbasePos}.
     */
    public void setR(double rx, double ry, double rtheta) {
        ukf.setR(rx, ry, rtheta);
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

    /** @return whether this is D3 mode */
    public boolean isD3() { return useD3; }

    // ==================== Reset ====================
    /** Reset localization to the given pose. */
    public void reset(Pose2d pose) {
        ukf.reset(pose.position.x, pose.position.y, pose.heading.toDouble());
        odom.setPose(pose);
    }

    // ==================== Internal helpers ====================

    private double getNow() {
        return System.nanoTime() / 1e9;
    }
}

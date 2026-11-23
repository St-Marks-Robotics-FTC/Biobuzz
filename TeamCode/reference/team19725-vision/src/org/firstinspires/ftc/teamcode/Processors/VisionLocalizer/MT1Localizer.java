package org.firstinspires.ftc.teamcode.Processors.VisionLocalizer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;

import java.util.List;

/**
 * Vision localizer based on Limelight MegaTag1.
 *
 * <p>After pulling the latest result from Limelight each frame via {@link #update()}:
 * <ul>
 *   <li>{@link #getPose()}    corrected global pose (inches, inches, radians)</li>
 *   <li>{@link #getRawPose()} uncorrected raw pose (for comparison/debugging)</li>
 *   <li>{@link #getHiveAngle()} / {@link #getHiveState()} / {@link #isHiveEstimated()} HIVE tilt-angle observations</li>
 *   <li>{@link #getAmbiguity()} / {@link #getStdDevs()} quality metrics (for adaptive EKF measurement noise R)</li>
 * </ul>
 *
 * <p><b>HIVE solution</b> (implementation: MT1Localizer.md section 3, derivation: OneMap_MT1Localizer.md):
 * Limelight loads the map of the HIVE in its <b>level reference state</b>, while the real HIVE
 * has rotated about the pivot (passing through (0,0,h) with its axis along the y axis) by theta,
 * so botpose differs from the true pose by a rotation about the y axis:
 * {@code T' = T_rot(phi)*T}, where {@code phi = -theta}. Solve for phi using the robot
 * ground-contact constraint (z = 0), then recover the true pose and obtain the tilt observation theta = -phi.
 *
 * <p><b>Team color</b>: switch the Limelight pipeline by {@link TeamColor} at construction time
 * (red 0 / blue 1) to load the field map for our side's HIVE; defaults to red when no color is given.
 *
 * <p><b>This class outputs per-frame observations only</b> (tilt angle and instantaneous state),
 * with no temporal filtering and no state latching -
 * continuous estimation (filtering / hysteresis / latching the last state) is handled by
 * outer modules such as RobotPosition.
 */
@Config
public class MT1Localizer implements Localizer {

    /** HIVE raised state: per-frame instantaneous classification, no hysteresis (MT1Localizer.md section 3.4) */
    public enum HiveState {
        /** Tilt is small enough (|theta| <= hiveCellDownAngleDeg); HIVE treated as level/centered */
        MIDDLE,
        /** AUDIENCE_UP side raised */
        AUDIENCE_UP,
        /** AUDIENCE_DOWN side raised */
        AUDIENCE_DOWN,
        /** No valid observation in this frame (no tags / no solution / attitude cross-check failed / tilt rejected) */
        UNKNOWN
    }

    private final Limelight3A limelight;

    /** Unit conversion: 1 m = 39.3701 in */
    private static final double M_TO_INCH = 39.37007874;

    /** Limelight pipeline index used for red */
    private static final int PIPELINE_RED = 0;
    /** Limelight pipeline index used for blue */
    private static final int PIPELINE_BLUE = 1;

    // ---- Latest result cache ----
    private LLResult latestResult;
    private Pose3D botpose;
    private boolean valid;

    // ---- MegaTag1 quality metrics ----
    /** MT1 stddevs [x, y, z, roll, pitch, yaw] (meters/degrees) */
    private double[] stdDevs;
    /** Number of tags used in the solution */
    private int tagCount;
    /** Average tag distance (meters) */
    private double avgDist;
    /** Average tag area */
    private double avgArea;
    /** Tag span (meters) */
    private double span;
    /** Pose timestamp (seconds) */
    private double timestamp;
    /** Capture latency (ms; see the SDK LLResult.getCaptureLatency documentation) */
    private double captureLatency;
    /**
     * Capture-time timestamp of this frame (seconds), converted to the {@link System#nanoTime()} time base.
     * The Limelight hardware timestamp and System.nanoTime() use different clocks and must never be
     * subtracted/compared directly; this value shares the time base of the EKF/UKF predict snapshots
     * and is used for vision timestamp rollback replay.
     *
     * <p>Refreshed only when a new physical frame is detected: held constant when the same physical
     * frame is read repeatedly. Otherwise every read would produce a growing estimate, breaking the
     * filter's timestamp-based stale-observation dedup (lastUpdateTime) and causing the same
     * observation to be fused repeatedly.
     */
    private double timestampNano;

    /**
     * Limelight hardware timestamp (seconds) of the previous physical frame, used to decide whether
     * this frame is new. Initialized to NaN (unequal to any value) so the first frame counts as new.
     */
    private double lastFrameTimestamp = Double.NaN;

    // ---- Max per-tag skew (larger means more ambiguous) ----
    private double maxFiducialSkew;

    // ---- Raw pose (uncorrected, inches/radians) ----
    private double rawXIn;
    private double rawYIn;
    private double rawZIn;
    private double rawYaw;
    private double rawPitch;

    // ---- HIVE solution observations (per-frame, no filtering/no state latching) ----
    /** HIVE tilt observation theta = -phi (radians); NaN when unsolved */
    private double hiveAngle = Double.NaN;
    /** Instantaneous state of this frame's HIVE tilt observation */
    private HiveState hiveState = HiveState.UNKNOWN;
    /** Whether this frame successfully solved the tilt observation */
    private boolean hiveEstimated = false;
    /** Corrected true pose (inches, inches, radians) */
    private double correctedXIn;
    private double correctedYIn;
    private double correctedYaw;
    /**
     * Attitude cross-check error (degrees): absolute difference between phi solved from position
     * and phi independently estimated from rawPitch. A mismatch means this frame's z' is
     * degenerate (see {@link #getRawZIn()}), so the frame is rejected. NaN when there is no observation.
     */
    private double pitchCheckErrDeg = Double.NaN;

    // ==================== Construction ====================

    /**
     * Initialize with the red alliance default (pipeline 0).
     *
     * @param limelight Limelight3A instance that has been initialized with {@link Limelight3A#start()} already called
     */
    public MT1Localizer(Limelight3A limelight) {
        this(limelight, TeamColor.RED);
    }

    /**
     * @param limelight Limelight3A instance that has been initialized with {@link Limelight3A#start()} already called
     * @param teamColor team color: {@link TeamColor#RED} loads pipeline 0, {@link TeamColor#BLUE} loads pipeline 1
     */
    public MT1Localizer(Limelight3A limelight, TeamColor teamColor) {
        this.limelight = limelight;
        this.valid = false;
        this.stdDevs = new double[6];
        // Switch pipeline by team color (red 0 / blue 1); selects which side's HIVE field map to load
        limelight.pipelineSwitch(teamColor == TeamColor.BLUE ? PIPELINE_BLUE : PIPELINE_RED);
    }

    // ==================== Core update ====================

    /**
     * Pull the latest MegaTag1 result from Limelight and solve the HIVE tilt observation plus corrected pose.
     * Call once per loop iteration.
     *
     * @return zero velocity (a vision localizer cannot provide velocity)
     */
    @Override
    public PoseVelocity2d update() {
        latestResult = limelight.getLatestResult();

        if (latestResult == null || !latestResult.isValid()) {
            valid = false;
            resetHiveObservation();
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        botpose = latestResult.getBotpose();
        if (botpose == null) {
            valid = false;
            resetHiveObservation();
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }

        // Extract MegaTag1 quality metrics
        // Firmware may return null or a short array; overwriting directly would make getStdDevs()/adaptR() throw NPE/index errors.
        // Keep the last valid values (avoids all-zero being mistaken for high confidence) so stdDevs stays non-null with length 6.
        double[] mt1StdDevs = latestResult.getStddevMt1();
        if (mt1StdDevs != null && mt1StdDevs.length >= stdDevs.length) {
            stdDevs = mt1StdDevs;
        }
        tagCount = latestResult.getBotposeTagCount();
        avgDist = latestResult.getBotposeAvgDist();
        avgArea = latestResult.getBotposeAvgArea();
        span = latestResult.getBotposeSpan();
        double frameTimestamp = latestResult.getTimestamp();
        // Decide whether this is a new physical frame: when the Limelight frame rate is lower than the loop rate,
        // getLatestResult() returns the same frame repeatedly with a constant hardware timestamp. Only a changed
        // hardware timestamp counts as a new frame.
        boolean isNewFrame = frameTimestamp != lastFrameTimestamp;
        lastFrameTimestamp = frameTimestamp;
        timestamp = frameTimestamp;

        captureLatency = latestResult.getCaptureLatency();
        // Convert to the System.nanoTime() time base: estimate the frame capture time as the local receive time minus capture latency.
        // (The Limelight hardware timestamp and System.nanoTime() use different clocks and must never be compared directly)
        // Refresh timestampNano on new frames only: keep the timestamp constant on repeated reads of the same frame so the
        // EKF/UKF lastUpdateTime dedup and ReplayBuffer rollback replay are not retriggered by the same physical frame.
        if (isNewFrame) {
            timestampNano = System.nanoTime() / 1e9 - captureLatency / 1000.0;
        }

        // Compute the max single-tag skew (larger skew means a more ambiguous attitude solution)
        maxFiducialSkew = 0.0;
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fr : fiducials) {
                double skew = fr.getSkew();
                if (skew > maxFiducialSkew) {
                    maxFiducialSkew = skew;
                }
            }
        }

        // Raw pose: meters to inches, degrees to radians
        rawXIn = botpose.getPosition().x * M_TO_INCH;
        rawYIn = botpose.getPosition().y * M_TO_INCH;
        rawZIn = botpose.getPosition().z * M_TO_INCH;
        rawYaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        rawPitch = botpose.getOrientation().getPitch(AngleUnit.RADIANS);

        valid = true;

        // Single-map HIVE solution: output the tilt observation and the corrected true pose
        solveHiveObservation();

        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    // ==================== HIVE solution (MT1Localizer.md section 3) ====================

    /**
     * Solve this frame's HIVE tilt observation and corrected pose (MT1Localizer.md sections 3.2-3.4).
     *
     * <p>On solution failure (no solution / attitude cross-check failed / illegal solution parameters),
     * {@link #isHiveEstimated()} is false and {@link #getPose()} falls back to the uncorrected raw pose.
     */
    private void solveHiveObservation() {
        resetHiveObservation();

        final double h = HypParams.hivePivotHeightIn;
        if (!(h > 0)) {
            // Pivot height not calibrated: skip the solution to avoid emitting a meaningless tilt observation
            return;
        }

        // ---- Section 3.2 ground-contact constraint: A*sin(phi) + B*cos(phi) = C ----
        final double a = rawXIn;
        final double b = rawZIn - h;
        final double c = -h;
        final double radius = Math.hypot(a, b);

        // Solvability check: no solution for this frame when |C| > R
        if (radius < 1e-9 || Math.abs(c) > radius) {
            return;
        }

        final double phi = solvePhi(a, b, c, Math.toRadians(HypParams.hiveCellUpAngleDeg));
        if (Double.isNaN(phi)) {
            return;
        }

        // ---- Degenerate-frame check: independently cross-check phi via attitude (pitch) ----
        // phi has two independent information paths:
        //   Position path: phi solved from the z' height constraint (line above)
        //   Attitude path: from R' = Ry(phi)*R_true with the robot flat on the ground (true pitch = 0),
        //             pitch' = asin(sin(phi) * cos(psi'))
        // A degenerate frame with |z'| near 0 collapses the position path back to phi = 0, while the
        // attitude path still reflects the true rotation. A large mismatch marks the frame degenerate
        // (its position correction is untrustworthy too, so the whole frame is dropped).
        // Note: this check assumes the robot sits flat on the ground and fails when the robot is on a ramp.
        final double sinPredicted = Math.sin(phi) * Math.cos(rawYaw);
        final double predictedPitch = Math.asin(Math.max(-1.0, Math.min(1.0, sinPredicted)));
        pitchCheckErrDeg = Math.toDegrees(Math.abs(normalize(rawPitch - predictedPitch)));
        if (pitchCheckErrDeg > HypParams.hivePitchCheckTolDeg) {
            return;
        }

        // ---- Section 3.3 position correction ----
        final double cosPhi = Math.cos(phi);
        final double sinPhi = Math.sin(phi);
        correctedXIn = a * cosPhi - rawZIn * sinPhi + h * sinPhi;
        correctedYIn = rawYIn;

        // ---- Section 3.3 attitude correction: R = Ry(-phi)*R', yaw = atan2(R10, R00) ----
        // The first column of R' is the robot x-axis expressed in the field frame (SDK convention: intrinsic yaw -> pitch -> roll)
        final double r00 = Math.cos(rawYaw) * Math.cos(rawPitch);
        final double r10 = Math.sin(rawYaw) * Math.cos(rawPitch);
        final double r20 = -Math.sin(rawPitch);
        correctedYaw = Math.atan2(r10, cosPhi * r00 - sinPhi * r20);

        // ---- Sections 3.1/3.4 tilt observation and instantaneous state ----
        hiveAngle = -phi;                       // theta = -phi
        hiveState = classifyHive(hiveAngle);
        hiveEstimated = true;
    }

    /**
     * Solve the trigonometric equation A*sin(phi) + B*cos(phi) = C (MT1Localizer.md section 3.2-2).
     *
     * <p>The two solution families are alpha - atan2(B,A) and pi - alpha - atan2(B,A) (alpha = asin(C/R)),
     * which always differ by pi - 2*alpha in [0, 2*pi], so except for degenerate geometry only one family
     * falls inside the +/-maxTilt window; if both fall inside, take the one with smaller |phi|
     * (this class keeps no phi_last; see the class comment).
     *
     * @param a       A = x' (inches)
     * @param b       B = z' - h (inches)
     * @param c       C = -h (inches)
     * @param maxTilt max tilt (radians) used for root selection
     * @return phi (radians), NaN when there is no solution
     */
    private static double solvePhi(double a, double b, double c, double maxTilt) {
        final double radius = Math.hypot(a, b);
        final double base = Math.atan2(b, a);
        final double alpha = Math.asin(c / radius);

        final double phi1 = normalize(alpha - base);
        final double phi2 = normalize(Math.PI - alpha - base);

        final boolean in1 = Math.abs(phi1) <= maxTilt;
        final boolean in2 = Math.abs(phi2) <= maxTilt;
        if (in1 && in2) {
            return Math.abs(phi1) <= Math.abs(phi2) ? phi1 : phi2;
        }
        if (in1) {
            return phi1;
        }
        if (in2) {
            return phi2;
        }
        return Double.NaN;
    }

    /**
     * Per-frame instantaneous classification (MT1Localizer.md section 3.4), <b>with no hysteresis</b>:
     * <ul>
     *   <li>|theta| <= hiveCellDownAngleDeg results in {@link HiveState#MIDDLE}</li>
     *   <li>hiveCellDownAngleDeg &lt; |theta| &lt; hiveCellUpAngleDeg means the HIVE stably rests on one side;
     *       pick {@link HiveState#AUDIENCE_UP} / {@link HiveState#AUDIENCE_DOWN} by the sign of theta</li>
     * </ul>
     * Frames with |theta| >= hiveCellUpAngleDeg are already rejected during root selection in {@link #solvePhi}
     * and never reach this method.
     */
    private static HiveState classifyHive(double theta) {
        if (Math.abs(theta) <= Math.toRadians(HypParams.hiveCellDownAngleDeg)) {
            return HiveState.MIDDLE;
        }
        final boolean up = HypParams.hivePositiveAngleIsAudienceUp == (theta > 0);
        return up ? HiveState.AUDIENCE_UP : HiveState.AUDIENCE_DOWN;
    }

    /** Normalize an angle to [-pi, pi). */
    private static double normalize(double angle) {
        double result = (angle + Math.PI) % (2 * Math.PI);
        if (result < 0) {
            result += 2 * Math.PI;
        }
        return result - Math.PI;
    }

    /** Clear this frame's HIVE observation (called on invalid frames or solution failure). */
    private void resetHiveObservation() {
        hiveEstimated = false;
        hiveAngle = Double.NaN;
        hiveState = HiveState.UNKNOWN;
        pitchCheckErrDeg = Double.NaN;
    }

    // ==================== Pose outputs (Localizer interface) ====================

    /**
     * @return corrected true pose when the HIVE solution succeeds, otherwise the uncorrected raw pose
     *         (inches, inches, radians); (0,0,0) when there is no valid result.
     *         Frame: FTC standard field coordinate system with the origin at the field center
     */
    @Override
    public Pose2d getPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        if (hiveEstimated) {
            return new Pose2d(correctedXIn, correctedYIn, correctedYaw);
        }
        return getRawPose();
    }

    /** A vision localizer does not support setting the pose. */
    @Override
    public void setPose(Pose2d pose) {
        // no-op
    }

    /**
     * @return uncorrected raw pose without HIVE correction (inches, inches, radians), for comparison against the corrected pose
     */
    public Pose2d getRawPose() {
        if (!valid || botpose == null) {
            return new Pose2d(0, 0, 0);
        }
        return new Pose2d(rawXIn, rawYIn, rawYaw);
    }

    /**
     * @return height z of the raw pose (inches).
     *
     * <p><b>z carries the tilt information of the position path</b>: from the solution model
     * F(phi) = x'*sin(phi) + (z'-h)*cos(phi) + h, F(0) = z' always holds. So when |z'| = 0 the
     * phi solved from the position path necessarily degenerates to 0 (theta = 0), regardless of the
     * true HIVE tilt. Such frames are identified and dropped by the attitude-path cross-check of
     * {@link #getHivePitchCheckErrDeg()}. If the HIVE is visibly tilted but this value stays near 0,
     * the loaded fmap does not contain this side's HIVE <b>level reference state</b> tags
     * (or the pose comes from other fixed field tags) - a configuration issue, not an algorithm issue (for debugging).
     */
    public double getRawZIn() {
        return rawZIn;
    }

    /**
     * @return pitch of the raw pose (radians, uncorrected).
     *         When the map rotates with the HIVE, pitch changes noticeably and serves as corroborating
     *         evidence for the tilt (with theta = 30 deg and a level-reference-state map, this value is
     *         about +/-30 deg * cos(heading)).
     */
    public double getRawPitch() {
        return rawPitch;
    }

    /**
     * @return all fiducial IDs seen in this frame, comma-separated.
     *         For debugging: verify only this side's HIVE tags are present (mixing in fixed field tags dilutes the tilt signal)
     */
    public String getTagIds() {
        if (latestResult == null) {
            return "";
        }
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        if (fiducials == null) {
            return "";
        }
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (ids.length() > 0) {
                ids.append(',');
            }
            ids.append(fr.getFiducialId());
        }
        return ids.toString();
    }

    /** @return array form {x, y, theta} of the current output pose (inches, inches, radians) */
    public double[] getPoseArray() {
        Pose2d pose = getPose();
        return new double[]{pose.position.x, pose.position.y, pose.heading.toDouble()};
    }

    /**
     * @return raw Pose3D object containing the full 6DOF pose
     */
    public Pose3D getBotpose() {
        return botpose;
    }

    /**
     * @return stddevs of the pose {@code double[6] = {x, y, z, roll, pitch, yaw}} (meters/degrees)
     */
    public double[] getStdDevs() {
        // Return a copy so callers cannot overwrite internal state directly
        return stdDevs.clone();
    }

    // ==================== HIVE observation outputs ====================

    /**
     * @return HIVE tilt observation theta = -phi (radians); {@link Double#NaN} when unsolved
     */
    public double getHiveAngle() {
        return hiveAngle;
    }

    /**
     * @return instantaneous state of this frame's HIVE tilt (no hysteresis); {@link HiveState#UNKNOWN} when there is no valid observation
     */
    public HiveState getHiveState() {
        return hiveState;
    }

    /**
     * @return whether this frame successfully solved the HIVE tilt observation.
     *         Callers use this to tell "truly level (MIDDLE)" apart from "unseen / unsolvable (UNKNOWN)".
     */
    public boolean isHiveEstimated() {
        return hiveEstimated;
    }

    /**
     * @return attitude cross-check error (degrees): difference between phi solved from the position path
     *         and phi estimated from the attitude path. Frames exceeding
     *         {@link HypParams#hivePitchCheckTolDeg} are treated as degenerate z' frames and dropped,
     *         so this value is within tolerance whenever {@link #isHiveEstimated()} is true.
     *         {@link Double#NaN} when there is no observation
     */
    public double getHivePitchCheckErrDeg() {
        return pitchCheckErrDeg;
    }

    // ==================== Uncertainty ====================

    /**
     * Combined uncertainty metric.
     * Planar position uncertainty (meters) computed from the MT1 stddevs.
     * Smaller values mean more reliable localization.
     *
     * <p>Formula: sqrt(stdX^2 + stdY^2)
     * <p>Typical threshold reference:
     * <ul>
     *   <li>&lt; 0.05  means high confidence</li>
     *   <li>0.05-0.15 means medium confidence</li>
     *   <li>&gt; 0.15  means low confidence, consider dropping</li>
     * </ul>
     *
     * @return planar position uncertainty (meters)
     */
    public double getAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 2) {
            return Double.MAX_VALUE;
        }
        return Math.sqrt(stdDevs[0] * stdDevs[0] + stdDevs[1] * stdDevs[1]);
    }

    /**
     * Angular uncertainty (radians).
     * From the yaw component of the MT1 stddevs.
     *
     * @return yaw uncertainty (radians)
     */
    public double getAngularAmbiguity() {
        if (!valid || stdDevs == null || stdDevs.length < 6) {
            return Double.MAX_VALUE;
        }
        return Math.toRadians(stdDevs[5]);
    }

    // ==================== Quality metrics ====================

    /** @return number of AprilTags used in the solution */
    public int getTagCount() {
        return tagCount;
    }

    /** @return average tag distance (meters) */
    public double getAvgDist() {
        return avgDist;
    }

    /** @return average tag area */
    public double getAvgArea() {
        return avgArea;
    }

    /** @return tag span (meters) */
    public double getSpan() {
        return span;
    }

    /** @return max single-tag skew (related to attitude ambiguity) */
    public double getMaxFiducialSkew() {
        return maxFiducialSkew;
    }

    /** @return pose timestamp (seconds), on a different time base from {@link System#nanoTime()} */
    public double getTimestamp() {
        return timestamp;
    }

    /** @return capture latency (milliseconds) */
    public double getCaptureLatency() {
        return captureLatency;
    }

    /**
     * @return capture-time timestamp of this frame (seconds), converted to the {@link System#nanoTime()} time base,
     *         sharing the time base of the EKF/UKF predict snapshots, for vision timestamp rollback replay
     */
    public double getTimestampNanoBase() {
        return timestampNano;
    }

    /** @return whether a valid localization result is currently available */
    public boolean isValid() {
        return valid;
    }

    // ==================== Convenience checks ====================

    /**
     * Check whether the current localization is reliable enough (for EKF update gating).
     * Combined condition: valid + tag count >= 2 + uncertainty below threshold.
     *
     * @param ambiguityThreshold uncertainty threshold (meters)
     * @return true if localization is reliable
     */
    public boolean isReliable(double ambiguityThreshold) {
        return valid && tagCount >= 2 && getAmbiguity() < ambiguityThreshold;
    }
}

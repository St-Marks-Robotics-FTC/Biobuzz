package org.firstinspires.ftc.teamcode.Processors.RobotPosition;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.Sensors.BallCounter;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;
// Handles all state estimation except vision
@Config
public class RobotPosition {
    static MecanumDrive drive;
    HardwareMap hardwareMap;

    /** Fusion localizer: Pinpoint velocity + Limelight vision + adaptive EKF (authoritative pose source) */
    private AdaptiveEKFLocalizer fusionLocalizer;

    /**
     * Tracked HIVE state: updated to the observation when MT1 has one, held at the previous
     * state when there is no observation or the solve is rejected.
     * Initialized to MIDDLE (HIVE level); the tracked value is never {@link MT1Localizer.HiveState#UNKNOWN}.
     */
    private MT1Localizer.HiveState hiveState = MT1Localizer.HiveState.MIDDLE;

    /** IMU, reusing the already-initialized Road Runner instance (device name "imu"), used only for resetYaw() */
    private IMU imu;

    /** Alliance color (saved at init, used to pick the goal by color) */
    private TeamColor teamColor = TeamColor.RED;

    /**
     * Whether the currently targeted goal is on the AudienceDown side.
     * Derived automatically from the MT1 HIVE state by {@link #update()} while non-MIDDLE observations arrive;
     * with no valid observation (no reading this frame, or MIDDLE) the previous frame's choice is kept,
     * and {@link #setManualGoalDown(boolean)} may overwrite it.
     * <b>There is exactly one rule throughout (lifted side = shoot that side)</b>:
     * HIVE on the AUDIENCE_UP side -> shoot the AudienceUp goal; AUDIENCE_DOWN side -> shoot the AudienceDown goal;
     * default to AudienceUp when no lifted side has ever been observed or manually set.
     */
    private boolean goalDown = false;

    /**
     * Most recent valid lifted side ({@code AUDIENCE_UP} / {@code AUDIENCE_DOWN}).
     * Updated by a non-MIDDLE MT1 observation or by the operator manual setting
     * ({@link #setManualGoalDown(boolean)}).
     * {@code null} means no valid lifted side has been observed yet; falls back to the default AudienceUp goal.
     */
    private MT1Localizer.HiveState lastLiftedSide = null;

    /** Cached value of the current goal: refreshed at init and every update frame; getGoalPose only reads this cache */
    private Pose2d cachedGoalPose = HypParams.RedAudienceUp;

    /** Ball channel counter (rising-edge count on the break-beam gate, cleared on reset); this class only forwards it */
    private BallCounter ballCounter;

    /** Device name of the ball channel break-beam gate in the robot configuration */
    private static final String BREAK_BEAM_NAME = "breakBeam";

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }

    /**
     * @return the initialized instance; null when not yet initialized.
     *         For optional consumers such as visualization, so they can access it safely
     *         without {@link #getInstance()} throwing.
     */
    public static RobotPosition getInstanceOrNull(){
        return instance;
    }
    private RobotPosition(){
    }

;
    /**
     * Initialize for RED alliance (Limelight pipeline 0).
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {
        return RobotPositioninit(hardwareMap, initpose, TeamColor.RED);
    }

    /**
     * @param hardwareMap hardware map
     * @param initpose initial pose
     * @param teamColor alliance color: RED loads Limelight pipeline 0, BLUE loads pipeline 1
     */
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose,
                                                  TeamColor teamColor) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);

        // ---- Goal selection: save the alliance color and refresh the default goal (AudienceUp) ----
        instance.teamColor = (teamColor != null) ? teamColor : TeamColor.RED;
        instance.goalDown = false;
        instance.cachedGoalPose = instance.goalPose(false);

        // ---- Build the fusion localizer first: Pinpoint velocity prediction + Limelight vision update + adaptive Q/R ----
        // The alliance color selects the Limelight pipeline (RED 0 / BLUE 1), switched by MT1Localizer by color.
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        instance.fusionLocalizer = new AdaptiveEKFLocalizer(
                hardwareMap, limelight, "imu", instance.currentPose, false, teamColor);

        // ---- Inject the fusion localizer into Road Runner as the single localizer of MecanumDrive ----
        // Pose and velocity both come straight from the EKF; no external setPose write-back is needed.
        // The localizer is advanced only here, once per frame via RobotPosition.update() calling
        // drive.updatePoseEstimate(); trajectory / turn Actions now reuse that frame's result internally.
        instance.drive = new MecanumDrive(hardwareMap, instance.fusionLocalizer);
        // IMU reuses the already-initialized Road Runner instance
        instance.imu = instance.drive.lazyImu.get();

        // ---- Ball channel counter: the gate is created and read by BallCounter itself; this class only holds and forwards it ----
        // The Sweeper stops intake on full based on getBallCount().
        // When the config lacks that analog port, degrade to no counting (ballCounter = null)
        // so OpModes without a channel (including autos) do not fail outright at Init.
        try {
            instance.ballCounter = new BallCounter(hardwareMap, BREAK_BEAM_NAME);
        } catch (RuntimeException e) {
            instance.ballCounter = null;
        }
        return instance;
    }

    /**
     * Reset the robot pose.
     * The MecanumDrive localizer is the fusion localizer itself, so one setPose call
     * resets both the EKF state and the internal Pinpoint odometry.
     *
     * @param pose target pose (true pose)
     */
    public void ResetPoseTo(Pose2d pose) {
        drive.localizer.setPose(pose);
        currentPose = pose;
    }

    // Called every frame: advance the fusion localizer and return the current pose
    public Pose2d update() {
        // Single advance point: call exactly once per frame so one frame's vision observation is not counted twice into the EKF
        // (trajectory / turn Actions inside MecanumDrive now reuse this frame's result internally)
        currentVelocity2d = drive.updatePoseEstimate();

        // HIVE state tracking: update when MT1 successfully solves a tilt observation, otherwise keep the previous state
        MT1Localizer mt1 = fusionLocalizer.getMT1();
        boolean freshObservation = mt1.isHiveEstimated();
        if (freshObservation) {
            hiveState = mt1.getHiveState();
        }

        // Auto-estimate the aim goal: exactly one rule throughout - lifted side = shoot that side
        //   AUDIENCE_UP   -> aim at the AudienceUp-side goal
        //   AUDIENCE_DOWN -> aim at the AudienceDown-side goal
        //   MIDDLE / no observation this frame -> keep the previous frame's choice (including the operator manual setting)
        // Only overwrite when this frame truly has a non-MIDDLE observation; that way the operator input is not
        // washed out every frame by leftover state from the previous frame when MT1 has no reading or reads MIDDLE.
        // An early implementation mapped MIDDLE to "the opposite of the most recent lifted side", with the opposite
        // sign convention of the two branches above: a single jittered MIDDLE frame from the HIVE would flip
        // the target to the other side, swinging the turret 180 degrees and changing every gear/speed setpoint.
        if (freshObservation
                && (hiveState == MT1Localizer.HiveState.AUDIENCE_UP
                || hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN)) {
            lastLiftedSide = hiveState;
            goalDown = (hiveState == MT1Localizer.HiveState.AUDIENCE_DOWN);
        } else if (lastLiftedSide == null) {
            goalDown = false;   // No valid lifted side observed yet: default to AudienceUp
        }
        // MIDDLE with history: goalDown stays unchanged (hysteresis, same "lifted side = same side" rule)
        cachedGoalPose = goalPose(goalDown);

        currentPose = drive.localizer.getPose();
        // Ball channel counting: this class "updates" its child BallCounter every frame (see the channel-counting diagram);
        // gate sampling happens exactly once here, and getBallCount() only reads the cached value.
        if (ballCounter != null) {
            ballCounter.update();
        }

        return currentPose;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }

    // ==================== Goal estimation (derived automatically from the HIVE state, read by turret auto-aim) ====================

    /** @return whether currently aiming at the AudienceDown-side goal (derived automatically from the HIVE state) */
    public boolean isGoalDown() {
        return goalDown;
    }

    /** @return current goal point (only x/y are meaningful; heading is meaningless) */
    public Pose2d getGoalPose() {
        return cachedGoalPose;
    }

    /**
     * Operator manually selects the current aim goal (mapped from co-driver LT/RT in TeleOp).
     *
     * <p>Priority: the non-MIDDLE MT1 observation of this frame wins, and {@link #update()} overwrites this choice;
     * with no valid observation this choice holds as the "most recent lifted side" until a new non-MIDDLE observation arrives.
     * So call it <b>before</b> {@link #update()} in the same frame.
     *
     * @param down true = AudienceDown-side goal, false = AudienceUp-side goal
     */
    public void setManualGoalDown(boolean down) {
        goalDown = down;
        lastLiftedSide = down ? MT1Localizer.HiveState.AUDIENCE_DOWN
                : MT1Localizer.HiveState.AUDIENCE_UP;
        cachedGoalPose = goalPose(down);
    }

    /**
     * @param down true = AudienceDown side, false = AudienceUp side
     * @return goal point selected by alliance color
     */
    private Pose2d goalPose(boolean down) {
        if (teamColor == TeamColor.BLUE) {
            return down ? HypParams.BlueAudienceDown : HypParams.BlueAudienceUp;
        }
        return down ? HypParams.RedAudienceDown : HypParams.RedAudienceUp;
    }
    /** @return field-frame X linear velocity (in/s). The body-frame input velocity is rotated into the field frame by R(theta). */
    public double getVx(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.cos(theta) - vyBody * Math.sin(theta);
    }
    /** @return field-frame Y linear velocity (in/s). The body-frame input velocity is rotated into the field frame by R(theta). */
    public double getVy(){
        double vxBody = currentVelocity2d.linearVel.x;
        double vyBody = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxBody * Math.sin(theta) + vyBody * Math.cos(theta);
    }
    public MecanumDrive getDrive(){return drive;}
    /** @return adaptive EKF fusion localizer (exposes MT1 HIVE tilt/state observations and Q/R debug values) */
    public AdaptiveEKFLocalizer getFusionLocalizer(){return fusionLocalizer;}

    /**
     * @return currently tracked HIVE state (MIDDLE / AUDIENCE_UP / AUDIENCE_DOWN).
     *         Updated to the observation in {@link #update()} when MT1 successfully solves a tilt observation;
     *         held at the previous state when there is no observation or the solve is rejected; never UNKNOWN.
     */
    public MT1Localizer.HiveState getHiveState(){return hiveState;}
    public double getOmega(){return currentVelocity2d.angVel;}

    // ---- Ball channel count pass-through (BallCounter: rising-edge counting, cleared on reset) ----

    /** @return accumulated ball count in the channel (rising-edge count; reset to the in-beam count 0/1 after resetBallCount; always 0 with no gate) */
    public int getBallCount() {
        return ballCounter == null ? 0 : ballCounter.getCount();
    }

    /** Reset the ball channel count and the edge-detection baseline (counts as 1 when a ball is still in the beam; forwards to {@link BallCounter#reset()}) */
    public void resetBallCount() {
        if (ballCounter == null) {
            return;
        }
        ballCounter.reset();
    }

    // ---- IMU functions (formerly IMUSensor.java, merged here) ----

    /**
     * Latest frame's IMU yaw/pitch/roll angles.
     * Sampled once per frame by {@code AdaptiveEKFLocalizer.update()} and published here; no hardware read here.
     */
    public YawPitchRollAngles getYawPitchRollAngles() {
        return fusionLocalizer.getImuAngles();
    }

    /**
     * Get yaw (heading) in the given unit, from the latest frame's IMU sample in {@code AdaptiveEKFLocalizer}.
     * @param angleUnit angle unit
     */
    public double getYaw(AngleUnit angleUnit) {
        return fusionLocalizer.getImuAngles().getYaw(angleUnit);
    }

    /**
     * Reset the IMU yaw to 0.
     * Note: Road Runner localization integrates IMU yaw for heading increments; calling this while running
     * corrupts the pose estimate, so use it only at init / calibration time.
     */
    public void resetYaw() {
        imu.resetYaw();
    }

}
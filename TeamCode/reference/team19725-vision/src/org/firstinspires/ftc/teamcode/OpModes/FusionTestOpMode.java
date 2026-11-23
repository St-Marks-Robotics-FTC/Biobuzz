package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveEKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.AdaptiveUKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.EKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.FusionLocalizer.UKFLocalizer;
import org.firstinspires.ftc.teamcode.Processors.VisionLocalizer.MT1Localizer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Multi-localizer comparison test OpMode.
 *
 * <p>Drive the robot with the gamepad while drawing the trajectories and poses of
 * {@link PinpointLocalizer}, {@link MT1Localizer}, {@link EKFLocalizer},
 * {@link AdaptiveEKFLocalizer} on the FTC Dashboard at the same time.
 *
 * <h3>Error measurement</h3>
 * <ol>
 *   <li>Set {@code testX / testY / testHeading} to the target pose in the FTC Dashboard</li>
 *   <li>Drive the robot to the TestPose</li>
 *   <li>Press gamepad button A to record every localizer pose and compute the errors</li>
 *   <li>Errors are shown in telemetry as (dx, dy, dtheta in degrees)</li>
 * </ol>
 *
 * <p>Color mapping:
 * <ul>
 *   <li>Green - PinpointLocalizer</li>
 *   <li>Orange - MT1Localizer (vision)</li>
 *   <li>Pink - EKFLocalizer (fixed Q/R)</li>
 *   <li>Purple - AdaptiveEKFLocalizer</li>
 *   <li>Cyan - UKFLocalizer (fixed Q/R)</li>
 *   <li>Blue - AdaptiveUKFLocalizer</li>
 *   <li>Yellow - AdaptiveEKFLocalizer + Replay (vision-timestamp rollback replay)</li>
 *   <li>Blue-gray - AdaptiveUKFLocalizer + Replay (vision-timestamp rollback replay)</li>
 *   <li>Red dashed line - TestPose (target point)</li>
 * </ul>
 *
 * <p>Note: the MT1 trail, current pose, and error are recorded/drawn only when the HIVE tilt
 * estimation for that frame succeeds
 * ({@code isValid() && isHiveEstimated()}) - when the estimation fails,
 * {@code MT1Localizer#getPose()} falls back to the uncorrected, contaminated pose.
 */
@Config
@TeleOp(name = "Fusion Test", group = "Fusion")
public class FusionTestOpMode extends LinearOpMode {

    private static final double IN_PER_TICK = 0.001999;

    // ---- TestPose (adjustable live on the Dashboard) ----
    public static double testX = 61.57;
    public static double testY = 59.27;
    public static double testHeading = Math.PI/2;  // radians

    // ---- Error records ----
    private boolean prevAPressed = false;
    private boolean errorRecorded = false;

    private double errPinpointX, errPinpointY, errPinpointTheta;
    private double errMt1X,       errMt1Y,       errMt1Theta;
    private double errEkfX,       errEkfY,       errEkfTheta;
    private double errAdaptiveX,  errAdaptiveY,  errAdaptiveTheta;
    private double errUkfX,       errUkfY,       errUkfTheta;
    private double errAdaptiveUkfX, errAdaptiveUkfY, errAdaptiveUkfTheta;
    private double errAdaptiveReplayX, errAdaptiveReplayY, errAdaptiveReplayTheta;
    private double errAdaptiveUkfReplayX, errAdaptiveUkfReplayY, errAdaptiveUkfReplayTheta;

    // ---- Trail history (for drawing) ----
    private final List<Pose2d> pinpointHistory    = new LinkedList<>();
    private final List<Pose2d> mt1History         = new LinkedList<>();
    private final List<Pose2d> ekfHistory         = new LinkedList<>();
    private final List<Pose2d> adaptiveHistory    = new LinkedList<>();
    private final List<Pose2d> ukfHistory         = new LinkedList<>();
    private final List<Pose2d> adaptiveUkfHistory = new LinkedList<>();
    private final List<Pose2d> adaptiveReplayHistory    = new LinkedList<>();
    private final List<Pose2d> adaptiveUkfReplayHistory = new LinkedList<>();
    private static final int MAX_HISTORY = 80;

    /** Recording duration of the post-start calibration phase (milliseconds) */
    public static int CALIB_DURATION_MS = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- Initialize Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        Pose2d initialPose = new Pose2d(0, 0, 0);

        // ---- Create all localizers ----
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, IN_PER_TICK, initialPose);
        MT1Localizer mt1 = new MT1Localizer(limelight);
        EKFLocalizer ekf = new EKFLocalizer(hardwareMap, limelight, initialPose);
        AdaptiveEKFLocalizer adaptiveEkf = new AdaptiveEKFLocalizer(hardwareMap, limelight, "imu", initialPose, false);
        UKFLocalizer ukf = new UKFLocalizer(hardwareMap, limelight, initialPose);
        AdaptiveUKFLocalizer adaptiveUkf = new AdaptiveUKFLocalizer(hardwareMap, limelight, "imu", initialPose, false);
        // Rollback-replay control group: AdaptiveEKF / AdaptiveUKF with vision-timestamp rollback replay enabled
        AdaptiveEKFLocalizer adaptiveEkfReplay = new AdaptiveEKFLocalizer(
                hardwareMap, limelight, "imu", initialPose, false, TeamColor.RED, true);
        AdaptiveUKFLocalizer adaptiveUkfReplay = new AdaptiveUKFLocalizer(
                hardwareMap, limelight, "imu", initialPose, false, true);

        // ---- Create the drive ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        // ---- Post-start calibration: press A to record the MT1 pose and average it as the start pose ----
        Pose2d calibratedPose = calibrateViaVision(mt1);
        pinpoint.setPose(calibratedPose);
        ekf.setPose(calibratedPose);
        adaptiveEkf.setPose(calibratedPose);
        ukf.setPose(calibratedPose);
        adaptiveUkf.setPose(calibratedPose);
        adaptiveEkfReplay.setPose(calibratedPose);
        adaptiveUkfReplay.setPose(calibratedPose);
        drive.localizer.setPose(calibratedPose);

        telemetry.addLine("Calibration done, driving enabled");
        telemetry.update();

        // ---- Main loop ----
        while (opModeIsActive()) {
            // === Gamepad drive ===
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // === Update all localizers ===
            drive.updatePoseEstimate();
            pinpoint.update();
            mt1.update();
            ekf.update();
            adaptiveEkf.update();
            ukf.update();
            adaptiveUkf.update();
            adaptiveEkfReplay.update();
            adaptiveUkfReplay.update();

            // === Read poses ===
            Pose2d pinpointPose    = pinpoint.getPose();
            Pose2d mt1Pose         = mt1.getPose();
            Pose2d ekfPose         = ekf.getPose();
            Pose2d adaptiveEkfPose = adaptiveEkf.getPose();
            Pose2d ukfPose         = ukf.getPose();
            Pose2d adaptiveUkfPose = adaptiveUkf.getPose();
            Pose2d adaptiveEkfReplayPose = adaptiveEkfReplay.getPose();
            Pose2d adaptiveUkfReplayPose = adaptiveUkfReplay.getPose();

            // The MT1 pose is uncontaminated only when this frame's HIVE tilt estimation succeeds:
            // on failed frames getPose() falls back to the uncorrected raw pose (see the MT1Localizer
            // class comment), so it must not be mixed into the trail or the error statistics.
            boolean mt1Usable = mt1.isValid() && mt1.isHiveEstimated();

            // === Record trail history ===
            addToHistory(pinpointHistory, pinpointPose);
            if (mt1Usable) {
                addToHistory(mt1History, mt1Pose);
            }
            addToHistory(ekfHistory, ekfPose);
            addToHistory(adaptiveHistory, adaptiveEkfPose);
            addToHistory(ukfHistory, ukfPose);
            addToHistory(adaptiveUkfHistory, adaptiveUkfPose);
            addToHistory(adaptiveReplayHistory, adaptiveEkfReplayPose);
            addToHistory(adaptiveUkfReplayHistory, adaptiveUkfReplayPose);

            // === Button A triggers error recording ===
            boolean aPressed = gamepad1.a;
            if (aPressed && !prevAPressed) {
                recordErrors(pinpointPose, mt1Pose, mt1Usable, ekfPose, adaptiveEkfPose, ukfPose, adaptiveUkfPose,
                        adaptiveEkfReplayPose, adaptiveUkfReplayPose);
            }
            prevAPressed = aPressed;

            // ============ Telemetry: poses ============
            telemetry.addData("Pinpoint",    formatPose(pinpointPose));
            telemetry.addData("MT1",         formatPose(mt1Pose));
            telemetry.addData("EKF",         formatPose(ekfPose));
            telemetry.addData("AdaptiveEKF", formatPose(adaptiveEkfPose));
            telemetry.addData("UKF",         formatPose(ukfPose));
            telemetry.addData("AdaptiveUKF", formatPose(adaptiveUkfPose));
            telemetry.addData("AEKF+Replay", formatPose(adaptiveEkfReplayPose));
            telemetry.addData("AUKF+Replay", formatPose(adaptiveUkfReplayPose));

            // ============ Telemetry: TestPose ============
            Pose2d testPose = new Pose2d(testX, testY, testHeading);
            telemetry.addData("TestPose", formatPose(testPose));
            telemetry.addData("Press A to record error", "");

            // ============ Telemetry: recorded errors ============
            if (errorRecorded) {
                telemetry.addLine("=== Errors (Δx, Δy, Δθ°) ===");
                telemetry.addData("Pinpoint err",
                        formatErr(errPinpointX, errPinpointY, errPinpointTheta));
                telemetry.addData("MT1 err",
                        formatErr(errMt1X, errMt1Y, errMt1Theta));
                telemetry.addData("EKF err",
                        formatErr(errEkfX, errEkfY, errEkfTheta));
                telemetry.addData("AdaptiveEKF err",
                        formatErr(errAdaptiveX, errAdaptiveY, errAdaptiveTheta));
                telemetry.addData("UKF err",
                        formatErr(errUkfX, errUkfY, errUkfTheta));
                telemetry.addData("AdaptiveUKF err",
                        formatErr(errAdaptiveUkfX, errAdaptiveUkfY, errAdaptiveUkfTheta));
                telemetry.addData("AEKF+Replay err",
                        formatErr(errAdaptiveReplayX, errAdaptiveReplayY, errAdaptiveReplayTheta));
                telemetry.addData("AUKF+Replay err",
                        formatErr(errAdaptiveUkfReplayX, errAdaptiveUkfReplayY, errAdaptiveUkfReplayTheta));
            }

            // ============ Telemetry: MT1Localizer live quality metrics ============
            telemetry.addLine("--- MT1Localizer ---");
            telemetry.addData("MT1.valid",               mt1.isValid());
            telemetry.addData("MT1.tagIds",              mt1.getTagIds());
            telemetry.addData("MT1.tagCount",            mt1.getTagCount());
            telemetry.addData("MT1.avgDist (m)",         mt1.getAvgDist());
            telemetry.addData("MT1.avgArea",             mt1.getAvgArea());
            telemetry.addData("MT1.span (m)",            mt1.getSpan());
            telemetry.addData("MT1.maxFiducialSkew",     mt1.getMaxFiducialSkew());
            telemetry.addData("MT1.ambiguity (m)",       mt1.getAmbiguity());
            telemetry.addData("MT1.angularAmb (rad)",    mt1.getAngularAmbiguity());
            telemetry.addData("MT1.captureLatency (ms)", mt1.getCaptureLatency());
            telemetry.addData("MT1.timestamp (s)",       mt1.getTimestamp());

            // ============ Telemetry: MT1Localizer HIVE observations ============
            telemetry.addLine("--- MT1Localizer HIVE ---");
            telemetry.addData("MT1.hiveEstimated",       mt1.isHiveEstimated());
            telemetry.addData("MT1.hiveState",           mt1.getHiveState());
            telemetry.addData("MT1.hiveAngle (deg)",     Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("MT1.pitchCheckErr (deg)", mt1.getHivePitchCheckErrDeg());
            telemetry.addData("MT1.rawZ (in)",           mt1.getRawZIn());
            telemetry.addData("MT1.rawPitch (deg)",      Math.toDegrees(mt1.getRawPitch()));

            double[] std = mt1.getStdDevs();
            if (std != null && std.length >= 6) {
                telemetry.addData("MT1.stdX (m)",         std[0]);
                telemetry.addData("MT1.stdY (m)",         std[1]);
                telemetry.addData("MT1.stdZ (m)",         std[2]);
                telemetry.addData("MT1.stdRoll (deg)",    std[3]);
                telemetry.addData("MT1.stdPitch (deg)",   std[4]);
                telemetry.addData("MT1.stdYaw (deg)",     std[5]);
            }

            // ============ Telemetry: actually applied Q/R matrices of each filter (diagonal values) ============
            // EKF/UKF (control group): both Q and R are fixed.
            // AdaptiveEKF/UKF: Q adapts every cycle to the IMU operating condition (Q = qBase x boost); R adapts to vision.
            double[] q, r;

            // --- EKF (control) ---
            telemetry.addLine("--- EKF (control, fixed Q/R) ---");
            q = ekf.getEKF().getQDiag();
            r = ekf.getEKF().getRDiag();
            telemetry.addData("EKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("EKF.R diag (x,y,θ)", formatDiag(r));

            // --- AdaptiveEKF ---
            telemetry.addLine("--- AdaptiveEKF (adaptive Q/R) ---");
            q = adaptiveEkf.getEKF().getQDiag();
            r = adaptiveEkf.getEKF().getRDiag();
            telemetry.addData("AEKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("AEKF.R diag (x,y,θ)", formatDiag(r));

            // --- UKF (control) ---
            telemetry.addLine("--- UKF (control, fixed Q/R) ---");
            q = ukf.getUKF().getQDiag();
            r = ukf.getUKF().getRDiag();
            telemetry.addData("UKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("UKF.R diag (x,y,θ)", formatDiag(r));

            // --- AdaptiveUKF ---
            telemetry.addLine("--- AdaptiveUKF (adaptive Q/R) ---");
            q = adaptiveUkf.getUKF().getQDiag();
            r = adaptiveUkf.getUKF().getRDiag();
            telemetry.addData("AUKF.Q diag (x,y,θ)", formatDiag(q));
            telemetry.addData("AUKF.R diag (x,y,θ)", formatDiag(r));

            // ============ Telemetry: rollback-replay status ============
            telemetry.addLine("--- Replay (vision-timestamp rollback replay) ---");
            telemetry.addData("AEKF+Replay.enabled", adaptiveEkfReplay.getEKF().isReplayEnabled());
            telemetry.addData("AEKF+Replay.buffer",  adaptiveEkfReplay.getEKF().getReplayBufferSize());
            telemetry.addData("AUKF+Replay.enabled", adaptiveUkfReplay.getUKF().isReplayEnabled());
            telemetry.addData("AUKF+Replay.buffer",  adaptiveUkfReplay.getUKF().getReplayBufferSize());


            telemetry.update();

            // === Dashboard drawing ===
            TelemetryPacket packet = new TelemetryPacket();

            // TestPose - red
            packet.fieldOverlay().setStroke("#F44336");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), testPose);

            // Trail drawing
            drawTrail(packet.fieldOverlay(), pinpointHistory,    "#4CAF50", 2);
            drawTrail(packet.fieldOverlay(), mt1History,         "#FF9800", 2);
            drawTrail(packet.fieldOverlay(), ekfHistory,         "#E91E63", 2);
            drawTrail(packet.fieldOverlay(), adaptiveHistory,    "#9C27B0", 2);
            drawTrail(packet.fieldOverlay(), ukfHistory,         "#00BCD4", 2);
            drawTrail(packet.fieldOverlay(), adaptiveUkfHistory, "#2196F3", 2);
            drawTrail(packet.fieldOverlay(), adaptiveReplayHistory,    "#FFEB3B", 2);
            drawTrail(packet.fieldOverlay(), adaptiveUkfReplayHistory, "#607D8B", 2);

            // Current-pose drawing
            packet.fieldOverlay().setStroke("#4CAF50");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), pinpointPose);

            if (mt1Usable) {
                packet.fieldOverlay().setStroke("#FF9800");
                packet.fieldOverlay().setStrokeWidth(2);
                Drawing.drawRobot(packet.fieldOverlay(), mt1Pose);
            }

            packet.fieldOverlay().setStroke("#E91E63");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ekfPose);

            packet.fieldOverlay().setStroke("#9C27B0");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveEkfPose);

            packet.fieldOverlay().setStroke("#00BCD4");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), ukfPose);

            packet.fieldOverlay().setStroke("#2196F3");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveUkfPose);

            packet.fieldOverlay().setStroke("#FFEB3B");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveEkfReplayPose);

            packet.fieldOverlay().setStroke("#607D8B");
            packet.fieldOverlay().setStrokeWidth(2);
            Drawing.drawRobot(packet.fieldOverlay(), adaptiveUkfReplayPose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    // ==================== Post-start calibration ====================

    /**
     * Record the start pose via MT1 vision localization: after pressing A, collect valid poses
     * over a period of time and return their mean as the calibration result (arithmetic mean
     * for position, circular mean for heading).
     */
    private Pose2d calibrateViaVision(MT1Localizer mt1) {
        telemetry.addLine("Calibration: press A to record start pose (MT1)");
        telemetry.update();

        boolean prevA = false;
        boolean isRecording = false;
        long recordingStartMs = 0;
        List<Pose2d> samples = new ArrayList<>();

        while (opModeIsActive()) {
            mt1.update();
            Pose2d pose = mt1.getPose();

            // Only frames with a successful HIVE tilt estimation may be used for calibration: on failed
            // frames getPose() falls back to the contaminated raw pose.
            boolean usable = mt1.isValid() && mt1.isHiveEstimated();

            telemetry.addLine("--- Calibration (MT1) ---");
            if (usable) {
                telemetry.addData("X (in)", "%.2f", pose.position.x);
                telemetry.addData("Y (in)", "%.2f", pose.position.y);
                telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading.toDouble()));
            } else {
                telemetry.addLine("No valid pose (needs valid + hiveEstimated)");
            }
            telemetry.addData("Tag Count", mt1.getTagCount());
            telemetry.addData("Ambiguity (m)", "%.4f", mt1.getAmbiguity());
            telemetry.addData("Hive State", mt1.getHiveState());
            telemetry.addData("Hive Angle (deg)", "%.2f", Math.toDegrees(mt1.getHiveAngle()));
            telemetry.addData("Pitch Check Err (deg)", "%.2f", mt1.getHivePitchCheckErrDeg());

            boolean a = gamepad1.a;
            if (a && !prevA && !isRecording) {
                isRecording = true;
                recordingStartMs = System.currentTimeMillis();
                samples.clear();
            }
            prevA = a;

            if (isRecording) {
                if (usable) {
                    samples.add(pose);
                }
                if (System.currentTimeMillis() - recordingStartMs >= CALIB_DURATION_MS) {
                    isRecording = false;
                    if (!samples.isEmpty()) {
                        return computeMeanPose(samples);
                    }
                    telemetry.addLine("No valid samples, press A to retry");
                }
            }

            telemetry.addLine();
            telemetry.addData("Recording", isRecording ? "Active" : "Inactive");
            if (isRecording) {
                telemetry.addData("Time remaining (ms)",
                        CALIB_DURATION_MS - (System.currentTimeMillis() - recordingStartMs));
                telemetry.addData("Samples", samples.size());
            }
            telemetry.update();
        }
        // Execution reaches here only when the OpMode stops.
        return new Pose2d(0, 0, 0);
    }

    /** Arithmetic mean for position, circular mean for heading. */
    private static Pose2d computeMeanPose(List<Pose2d> samples) {
        double sumX = 0, sumY = 0;
        double sumSin = 0, sumCos = 0;
        for (Pose2d p : samples) {
            sumX += p.position.x;
            sumY += p.position.y;
            sumSin += Math.sin(p.heading.toDouble());
            sumCos += Math.cos(p.heading.toDouble());
        }
        return new Pose2d(sumX / samples.size(), sumY / samples.size(), Math.atan2(sumSin, sumCos));
    }

    // ==================== Error computation ====================

    private void recordErrors(Pose2d pp, Pose2d mt, boolean mtUsable, Pose2d ek, Pose2d ae, Pose2d uk, Pose2d au,
                              Pose2d aer, Pose2d aur) {
        Pose2d ref = new Pose2d(testX, testY, testHeading);

        errPinpointX     = pp.position.x - ref.position.x;
        errPinpointY     = pp.position.y - ref.position.y;
        errPinpointTheta = normalizeAngle(pp.heading.toDouble() - ref.heading.toDouble());

        if (mtUsable) {
            errMt1X      = mt.position.x - ref.position.x;
            errMt1Y      = mt.position.y - ref.position.y;
            errMt1Theta  = normalizeAngle(mt.heading.toDouble() - ref.heading.toDouble());
        } else {
            // The HIVE tilt has no solution or was rejected for this frame, so the MT1 pose is the
            // uncorrected, contaminated pose and is excluded from the error statistics.
            errMt1X     = Double.NaN;
            errMt1Y     = Double.NaN;
            errMt1Theta = Double.NaN;
        }

        errEkfX          = ek.position.x - ref.position.x;
        errEkfY          = ek.position.y - ref.position.y;
        errEkfTheta      = normalizeAngle(ek.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveX     = ae.position.x - ref.position.x;
        errAdaptiveY     = ae.position.y - ref.position.y;
        errAdaptiveTheta = normalizeAngle(ae.heading.toDouble() - ref.heading.toDouble());

        errUkfX          = uk.position.x - ref.position.x;
        errUkfY          = uk.position.y - ref.position.y;
        errUkfTheta      = normalizeAngle(uk.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveUkfX     = au.position.x - ref.position.x;
        errAdaptiveUkfY     = au.position.y - ref.position.y;
        errAdaptiveUkfTheta = normalizeAngle(au.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveReplayX     = aer.position.x - ref.position.x;
        errAdaptiveReplayY     = aer.position.y - ref.position.y;
        errAdaptiveReplayTheta = normalizeAngle(aer.heading.toDouble() - ref.heading.toDouble());

        errAdaptiveUkfReplayX     = aur.position.x - ref.position.x;
        errAdaptiveUkfReplayY     = aur.position.y - ref.position.y;
        errAdaptiveUkfReplayTheta = normalizeAngle(aur.heading.toDouble() - ref.heading.toDouble());

        errorRecorded = true;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== Utilities ====================

    private static void addToHistory(List<Pose2d> history, Pose2d pose) {
        history.add(pose);
        if (history.size() > MAX_HISTORY) {
            history.remove(0);
        }
    }

    private static void drawTrail(com.acmerobotics.dashboard.canvas.Canvas c,
                                  List<Pose2d> history, String color, int width) {
        if (history.size() < 2) return;
        c.setStroke(color);
        c.setStrokeWidth(width);
        c.setFill(color);
        double[] xs = new double[history.size()];
        double[] ys = new double[history.size()];
        for (int i = 0; i < history.size(); i++) {
            xs[i] = history.get(i).position.x;
            ys[i] = history.get(i).position.y;
        }
        c.strokePolyline(xs, ys);
    }

    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
                pose.position.x, pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }

    /** Format the Q/R matrix diagonal {a, b, c} as a string (%.5g, auto-scales for large/small values). */
    private static String formatDiag(double[] d) {
        return String.format("[%.5g, %.5g, %.5g]", d[0], d[1], d[2]);
    }

    private static String formatErr(double dx, double dy, double dtheta) {
        // The MT1 error is set to NaN when the HIVE tilt has no solution at the recording moment (see recordErrors).
        if (Double.isNaN(dx) || Double.isNaN(dy) || Double.isNaN(dtheta)) {
            return "n/a";
        }
        return String.format("(%+.2f, %+.2f, %+.1f°)",
                dx, dy, Math.toDegrees(dtheta));
    }
}
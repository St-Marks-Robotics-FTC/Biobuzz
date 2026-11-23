package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * Global hyperparameter configuration.
 * Centralizes every hyperparameter of the robot control system; hyperparameters are
 * constants that need no fitting, such as engineering parameters.
 */
@Config
// TODO: add goal positions, initial HIVE state, HIVE height thresholds, and other hyperparameters.
public class HypParams {
    /**
     * Chassis maximum speed (inches/second).
     */
    public static double maxV = 2.0;

    /**
     * Chassis maximum angular velocity.
     */
    public static double maxOmega = Math.PI;

    /**
     * TODO: red-alliance start pose (units: inches, radians).
     * Holds the start position (x, y) and start heading (theta).
     */
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * TODO: blue-alliance start pose (units: inches, radians).
     * Holds the start position (x, y) and start heading (theta).
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);
    /**
     * TODO: blue-alliance autonomous park pose (units: inches, radians).
     */
    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);
    /**
     * TODO: red-alliance autonomous park pose (units: inches, radians).
     */
    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);
    /**
     * Red-alliance first autonomous shooting area (units: inches, convex-polygon vertices).
     * Vertices: (24,-12), (48,-30), (48,-3), (72,-30), (72,-3).
     */
    public static ConvexPolygon ShootFirstAreaRed = new ConvexPolygon(
            new Vector2D(24, -12),
            new Vector2D(48, -30),
            new Vector2D(48, -3),
            new Vector2D(72, -30),
            new Vector2D(72, -3)
    );
    /**
     * Blue-alliance first autonomous shooting area: point reflection of the red first shooting
     * area about the origin (negate both x and y).
     * Vertices: (-24,12), (-48,30), (-48,3), (-72,30), (-72,3).
     */
    public static ConvexPolygon ShootFirstAreaBlue = new ConvexPolygon(
            new Vector2D(-24, 12),
            new Vector2D(-48, 30),
            new Vector2D(-48, 3),
            new Vector2D(-72, 30),
            new Vector2D(-72, 3)
    );
    /**
     * Red-alliance second autonomous shooting area: mirror of the red first shooting area
     * across the y axis (negate x).
     * Vertices: (-24,-12), (-48,-30), (-48,-3), (-72,-30), (-72,-3).
     */
    public static ConvexPolygon ShootSecondAreaRed = new ConvexPolygon(
            new Vector2D(-24, -12),
            new Vector2D(-48, -30),
            new Vector2D(-48, -3),
            new Vector2D(-72, -30),
            new Vector2D(-72, -3)
    );
    /**
     * Blue-alliance second autonomous shooting area: point reflection of the blue first shooting
     * area about the origin (negate both x and y).
     * Vertices: (24,12), (48,30), (48,3), (72,30), (72,3).
     */
    public static ConvexPolygon ShootSecondAreaBlue = new ConvexPolygon(
            new Vector2D(24, 12),
            new Vector2D(48, 30),
            new Vector2D(48, 3),
            new Vector2D(72, 30),
            new Vector2D(72, 3)
    );
    /**
     * Chassis half length (inches): distance from the chassis center to the midpoint of the
     * rearmost / foremost edge.
     * Used to convert between "rearmost-midpoint coordinates" and "chassis-center coordinates",
     * and to back out the center pose from an intake target point.
     * TODO: calibrate by measuring on the real robot.
     */
    public static double chassisHalfLengthIn = 9.0;

    /**
     * First ball-pickup point (intake target point, units: inches).
     */
    public static Vector2D EatPoint1 = new Vector2D(24, -72);
    /**
     * Second ball-pickup point (intake target point, units: inches).
     */
    public static Vector2D EatPoint2 = new Vector2D(-72, -24);

    /**
     * Chassis half width (inches): distance from the chassis center to the midpoint of the
     * left / right edge.
     * Together with {@link #chassisHalfLengthIn} forms the chassis bounding rectangle, used to
     * test "whole-robot footprint fully inside a region".
     * TODO: calibrate by measuring on the real robot.
     */
    public static double chassisHalfWidthIn = 9.0;

    /**
     * Rectangle directly under the HIVE (red alliance): (24,-25), (24,-1), (-24,-1), (-24,-25),
     * i.e. x in [-24,24], y in [-25,-1]; blue uses its point reflection ({@link #HiveRectBlue}).
     */
    public static ConvexPolygon HiveRectRed = new ConvexPolygon(
            new Vector2D(24, -25),
            new Vector2D(24, -1),
            new Vector2D(-24, -1),
            new Vector2D(-24, -25)
    );
    /**
     * Rectangle directly under the HIVE (blue alliance): point reflection of the red rectangle
     * about the origin, (x,y) to (-x,-y), i.e. x in [-24,24], y in [1,25].
     */
    public static ConvexPolygon HiveRectBlue = new ConvexPolygon(
            new Vector2D(-24, 25),
            new Vector2D(-24, 1),
            new Vector2D(24, 1),
            new Vector2D(24, 25)
    );

    /**
     * HIVE forbidden line (red alliance): segment between (24,-24) and (-24,-24).
     * The robot (including its whole footprint) must not touch it.
     * Blue uses its point-reflected segment
     * ({@link #HiveForbiddenLineBlueA}/{@link #HiveForbiddenLineBlueB}).
     */
    public static Vector2D HiveForbiddenLineRedA = new Vector2D(24, -24);
    public static Vector2D HiveForbiddenLineRedB = new Vector2D(-24, -24);
    /**
     * HIVE forbidden line (blue alliance): after point reflection, the segment between
     * (-24,24) and (24,24).
     */
    public static Vector2D HiveForbiddenLineBlueA = new Vector2D(-24, 24);
    public static Vector2D HiveForbiddenLineBlueB = new Vector2D(24, 24);

    /**
     * Extra safety margin (inches) beyond the HIVE rectangle edge for the whole-robot footprint
     * when routing around the HIVE rectangle, used by the {@code GoTo*AvoidHiveAction} detours.
     * Calibrate on the real robot.
     */
    public static double hiveAvoidClearanceIn = 2.0;

    /**
     * TODO: red-alliance reset pose (units: inches, radians).
     * Holds the start position (x, y) and start heading (theta).
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);

    /**
     * TODO: blue-alliance reset pose (units: inches, radians).
     * Holds the start position (x, y) and start heading (theta).
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);

    /**
     * Initial drive-mode flag.
     * true = headless mode (field frame), false = headed mode (robot frame).
     */
    public static boolean InitialUseNoHeadMode = false;

    /**
     * TODO: park time threshold (units: milliseconds).
     * Parks once the remaining autonomous time drops below this value.
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * FTC field half side length (inches): the 12 ft x 12 ft field is centered on the origin,
     * with borders at x/y = +/-72.
     * Used by the adaptive parking action to test "does the whole-robot bounding rectangle
     * touch the field border".
     */
    public static double fieldHalfSizeIn = 72.0;

    /**
     * Safety margin (inches) kept between the whole-robot bounding rectangle and the field
     * border during adaptive parking.
     * After contact is detected, the robot is pushed back to where the
     * "bounding rectangle stays at least this far from the border".
     */
    public static double parkBorderMarginIn = 3.0;

    /**
     * Total autonomous duration (units: milliseconds).
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;

    /**
     * Goal positions (aim targets; only x/y are meaningful, heading is ignored):
     * Red_Audience_Up: (17.7, -12.75)
     * Red_Audience_Down: (-17.7, -12.75)
     * Blue_Audience_Up: (17.7, 12.75)
     * Blue_Audience_Down: (-17.7, 12.75)
     */
    public static Pose2d RedAudienceUp = new Pose2d(17.7, -12.75, 0);
    public static Pose2d RedAudienceDown = new Pose2d(-17.7, -12.75, 0);
    public static Pose2d BlueAudienceUp = new Pose2d(17.7, 12.75, 0);
    public static Pose2d BlueAudienceDown = new Pose2d(-17.7, 12.75, 0);

    /**
     * No-shoot zone (convex polygon, field frame, used for the turret auto-aim preload check).
     * <p>{@code null} means disabled -- every position counts as shootable; the test point is
     * the chassis center.
     * <p>Inside this zone the preload spin-up speed is 0 and the pitch takes {@code pitchMin}.
     * TODO: fill in the vertices after calibrating on the real robot.
     */
    public static ConvexPolygon NoShootArea = null;

    // ==================== HIVE single-tag solver hyperparameters ====================
    // Unit conventions: lengths always in inches (same as the field frame), angles always in
    // degrees (variable names carry the Deg suffix); the solver converts to radians internally
    // (see MT1Localizer.solveHiveObservation).

    /**
     * TODO: height h of the HIVE pivot center above the field floor (inches).
     * Used by the ground-contact-constraint solver in MT1Localizer.md section 3.2; its error
     * feeds linearly into the x correction, so it must be calibrated on the real robot.
     */
    public static double hivePivotHeightIn = 43.95;

    /**
     * TODO: tilt-angle upper bound (degrees) for a CELL resting down (HIVE level / middle).
     * |theta| at or below this value means MIDDLE; this is also the tilt lower bound for
     * "stably resting on one side". Calibrate on the real robot.
     */
    public static double hiveCellDownAngleDeg = 23.0;

    /**
     * TODO: tilt-angle upper bound (degrees) for a CELL resting up (HIVE stably on one side);
     * must be greater than hiveCellDownAngleDeg.
     * |theta| above this value rejects the frame outright (this value also bounds root selection
     * for the trigonometric equation). Calibrate on the real robot.
     */
    public static double hiveCellUpAngleDeg = 35.0;

    /**
     * TODO: attitude cross-check tolerance (degrees).
     * When the phi solved from position and the phi independently derived from rawPitch differ
     * by more than this value, the frame is judged a degenerate z-prime frame and the HIVE
     * observation of that frame is dropped.
     * Assumes the robot sits flat on the ground (true pitch approx 0). Calibrate on the real robot.
     */
    public static double hivePitchCheckTolDeg = 10.0;

    /**
     * TODO: HIVE tilt sign convention.
     * true means theta > 0 indicates the AUDIENCE_UP side is raised.
     * false means theta > 0 indicates the AUDIENCE_DOWN side is raised.
     * The exact mapping depends on the fmap and the HIVE installation orientation;
     * calibrate on the real robot.
     */
    public static boolean hivePositiveAngleIsAudienceUp = Boolean.FALSE;
}
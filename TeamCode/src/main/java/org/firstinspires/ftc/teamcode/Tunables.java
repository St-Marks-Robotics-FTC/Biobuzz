package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;



@Configurable
public class Tunables {
    public static double slowModeMin = 0.2;
    public static double minShootDistance = 30.0;
    public static double maxShootDistance = 45.0;
    public static double maxAngleErrorDeg = 35.0;
    public static double shootP = 2.0; //PIDF can be tuned
    public static double shootI = 0.0;
    public static double shootD = 0.05;
    public static double shootF = 0.0;
    public static double aimMaxPower = 0.8;
    public static double aimToleranceDeg = 2.0; //Gap between target and acceptable actual
    public static double aimIntegralLimit = 1.0;
    public static double shooterTargetRpm = 3000.0;
    public static double shooterRampRpmPerSec = 2500.0;
    public static double shooterToleranceRpm = 20.0; //Gap between target and acceptable actual
    public static double shooterFireIntervalSec = 1; //Minimum time passing between shots
    public static double autoShootRpm = 3000; // flywheel RPM used by every auto volley (BozoTeleOp's default setRPM)
    public static long shootSpinUpTimeoutMs = 3000; // fire anyway if the flywheel hasn't reached autoShootRpm by now
    public static long intakeTimeMs = 1000;
    public static long pathTimeoutMs = 10000;
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = true; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /** BozoTeleOp tunables (used in BozoTeleOp.java) **/
    public static double adjustRPM = 50;
    public static double shootHeadingMargin = Math.toRadians(5);

    /** flywheel tunables (used in Flywheel.java) **/

    // PIDF tuned 9-28-2026
    public static double flywheelP = 0.006;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0.0004;
    public static double maxFlywheelBraking = 0.05; // max amount of breaking (negative power) to apply to launch motors
    // disable this for now
    public static double flywheelQuickStartThreshold = 10000000; // if TPS diff is greater than this, set full power (~1000rpm)

    /** intake tunables (used in Intake.java) **/

    public static double intakeForwardPower = 1.0;
    public static double intakeForwardLaunchingPower = 0.3;

    /** transfer tunables (used in Transfer.java) **/

    public static double transferOpen = 0.254;
    public static double transferClosed = 0.00;

    /** robot tunables (used in Robot.java) **/

    public static double launchOpenTime = 1500;

    /** field tunables (used in Field.java) **/

    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball

    /** vision / localization tunables (used in Vision.java and BozoTeleOp.java) **/

    public static boolean visionLocalization = true; // correct odometry x/y with the Limelight in TeleOp
    public static int visionMinTags = 2; // ignore Limelight frames that see fewer tags than this
    public static double visionMaxStalenessMs = 200; // ignore Limelight frames older than this
    public static double visionMaxSpeed = 8.0; // in/s; only correct odometry while slower than this (frame latency skews a moving fix)
    public static double visionMaxTurnRate = 0.6; // rad/s; only correct odometry while turning slower than this
    public static double visionBlend = 0.4; // fraction of the vision-odometry gap closed per correction
    public static double visionSnapDistance = 10.0; // in; bigger gaps are closed in a single correction
    public static double visionFuseIntervalMs = 100; // minimum time between odometry corrections
    public static int hiveConfirmFrames = 3; // consecutive frames that must agree before the hive state changes
    public static double hiveMinTiltDeg = 3.0; // tilts smaller than this look flat and leave the hive state alone

    /** auto turn tunables (used in BozoTeleOp.java) **/

    public static double aimSettleMs = 150; // heading must stay on target this long before auto turn gives the sticks back
    public static double aimTimeoutMs = 3000; // auto turn gives the sticks back after this long no matter what
}

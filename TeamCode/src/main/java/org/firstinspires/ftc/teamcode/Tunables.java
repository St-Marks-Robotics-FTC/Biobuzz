package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;



@Configurable
public class Tunables {
    public static double shooterToleranceRpm = 15.0; //Gap between target and acceptable actual
    public static double autoShootRpm = 3170; // flywheel RPM used by every auto volley (BozoTeleOp's default setRPM)
    public static long autoWaitBudgetMs = 10000; // all rev-up, shot, intake and tip-spin waits in one auto must fit in this; path travel is not counted
    public static long tipSpinMs = 600; // how long the post-preload tip-spin runs when the hive did not tip
    public static long intakeTimeMs = 700;
    public static long pathTimeoutMs = 3000;
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

    /** flower tunables (used in Flower.java) **/
    public static double flowerDown = 0.4;
    public static double flowerUp = 0.9;

    /** transfer tunables (used in Transfer.java) **/

    public static double transferOpen = 0.280;
    public static double transferClosed = 0.00;

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 800;

    /** field tunables (used in Field.java) **/

    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball

    /** vision tunables (used in Vision.java) **/

    public static int visionMinTags = 2; // ignore Limelight frames that see fewer tags than this
    public static double visionMaxStalenessMs = 200; // ignore Limelight frames older than this
    public static int hiveConfirmFrames = 3; // consecutive frames that must agree before the hive state changes
    public static double hiveMinTiltDeg = 3.0; // tilts smaller than this look flat and leave the hive state alone
}

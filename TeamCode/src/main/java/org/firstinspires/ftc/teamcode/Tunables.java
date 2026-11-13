/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Tunables {
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = false; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /** BozoTeleOp tunables (used in BozoTeleOp.java) **/

    public static double manualStartingRPM = 3150;
    public static double adjustRPM = 50;
    public static double shootHeadingMargin = Math.toRadians(5);
    public static boolean teleBrake = true;

    /** Adjustable Hood tunables (will be used in Hood.java) **/

    public static double nectarPos = 0.15;
    public static double pollenPos = 0.7;

    public static double flowerRpm = 3700;
    public static double pollenRpm = 3100;

    /** flywheel tunables (used in Flywheel.java) **/

    // PIDF tuned 9-28-2026
    public static double flywheelP = 0.008;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0.0004;
    public static double maxFlywheelBraking = 1.0; // max amount of breaking (negative power) to apply to launch motors
    public static double flywheelVoltageStart = 12.0; // start voltage of compensation scale
    public static double flywheelVoltageEnd = 13.5;   // end voltage of compensation scale
    public static double flywheelCompensationMax = 1.2; // max amount to scale voltage

    /** intake tunables (used in Intake.java) **/

    public static boolean intakeUsePowerSave = true;
    public static int intakePowerSaveWaitInterval = 30; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 30; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 6; // if we are drawing more amps than this, trigger power save
    public static double intakePowerSaveTriggerVelocity = 0; // if we have a tps less than this, trigger power save
    public static double intakeHoldPower = 0.0; // minimum power for balls to not fall out of intake

    /** transfer tunables (used in Transfer.java) **/

    public static double transferOpen = 0.41;
    public static double transferClosed = 0.108;

    public static int    shakeMillis = 60; // flower shake: time between flips, in ms (real period is rounded up to the loop length)
    public static double flowerDown = 0.7872;
    public static double flowerUp = 0.2461;
    public static double flowerShake = 0.05; // flower shake: servo position amplitude (NOT a time) around flowerDown
    public static double flowerDistance = 13.125; // how far from the edge of the field where the flower is should we refuel

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 525;

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double tagThreshold = 2; // amount of tags we need at least
    public static double shootRPM = 2975; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long flowerRefuelDurationMillis = 550; // how long to intake while driving through the refuel line
    public static long refuelDurationMillis = 200; // how long to intake while driving through the refuel line
    public static double maxTravelToRefuelTime = 1500; // timeout for travel to picking up balls (if it gets stuck on a ball)

    /** vision tunables (used in subsys/Vision.java) **/

    // HIVE correction; fit with limelight/fit_hive.py (h, pivot_y, z_offset, z_per_dist)
    public static double visionHivePivotHeight = 43.95; // in
    public static double visionHivePivotY = 72; // in, pedro y
    public static double visionZOffset = 0; // in, subtracted from raw z
    public static double visionZPerDist = 0; // in/in of avg tag distance, subtracted from raw z

    // camera offset
    public static double visionCameraErrorForward = 0.446;
    public static double visionCameraErrorLeft = -4.165;
    public static double visionBlueBiasX = -4.001;
    public static double visionBlueBiasY = -0.267;
    public static double visionRedBiasX = -1.392;
    public static double visionRedBiasY = 0.006;

    public static double visionMinGoalDist = 35; // in, ignore vision closer than this to the hive
    public static double visionMaxZSensitivity = 1.4; // reject if hive math amplifies z error this much
    public static double visionSensitivityRef = 1.2; // sensitivity above this inflates std dev
    public static double visionJumpMax = 8; // in, reject frames this far from the 9-frame median

    public static double visionMaxSpeed = 81; // in/s, pedro maxAchievableForwardVelocity
    public static double visionJumpMargin = 4; // in, allowed frame noise
    public static double visionPoseGain = 0.1;
    public static double visionPoseMaxSpeed = 10; // in/s, teleop relocalizes only below this

    public static double visionTunerSpotX = 72;
    public static double visionTunerSpotY = 72;

    public static double visionTunerSpeed = 0.3;
    public static double visionTunerRobotLength = 20;
    public static double visionTunerRobotWidth = 18;
    public static double visionTunerWallMargin = 3;
    public static double visionTunerObstacleAMinX = 0, visionTunerObstacleAMinY = 48, visionTunerObstacleAMaxX = 43, visionTunerObstacleAMaxY = 96;
    public static double visionTunerObstacleBMinX = 96, visionTunerObstacleBMinY = 48, visionTunerObstacleBMaxX = 144, visionTunerObstacleBMaxY = 96;
    public static double visionTunerGridSpacing = 8;
    public static double visionTunerGoalHeadingLimitDegrees = 100; // sweep +-this around goal
    public static double visionTunerSpinDegPerSec = 45;
    public static double visionTunerSpinEnterRadius = 4; // in, start spin this close to spot
    public static double visionTunerNoRecordMinY = 48; // full-width no-record band
    public static double visionTunerNoRecordMaxY = 96;
    public static double visionTunerLatencyOffsetMillis = 0; // extra camera delay
    public static int visionTunerMinTags = 2;
    public static double visionTunerTrueZ = 0;
    public static double visionTunerMoveTimeoutMillis = 6000; // stall timeout

    public static int cameraHeight = 9;
    public static int minTagCount = 2; // minimum amount of tags seen in a frame necessary to update our botpose

    /** fusion tunables (used in Fusion.java) **/

    // all distances in inches; "std" = standard deviation = typical size of the error
    public static double fusionStartStd = 2; // how unsure we are about our starting pose after auto
    public static double fusionOdoVariancePerInch = 0.02; // how much odometry uncertainty grows per inch driven (in^2 per in)
    public static double fusionVisionBaseStd = 0.187; // vision error with ONE tag at fusionVisionRefDistance
    public static double fusionVisionRefDistance = 59; // distance (in) that fusionVisionBaseStd was measured at
    public static double fusionTimingErrorMs = 10; // how far off our vision timing might be; turns driving speed into error
    public static double fusionTurnStd = 1.0; // extra vision error per rad/s of turning (motion blur)
    public static double fusionMinVisionStd = 0.5; // never trust a vision frame more than this, even if everything says it's perfect
    public static double fusionGateSigmas = 3; // reject vision frames more than this many std devs from our estimate
    public static int fusionRejectsBeforeDoubt = 10; // rejected frames in a row before we start doubting odometry instead
    public static double fusionExtraLatencyMs = 0; // extra delay on top of what the Limelight reports; tune if fused pose wobbles when driving

    /** field tunables (used in Field.java) **/

    // auto RPM
    public static double autoRPMMultiplier = 1; // multiply auto RPM output by this
    public static double autoRPMAddition = 0; // add this many RPMs to auto RPM output

    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));
    /*

        blueAudienceRight (dx > 0): (94.7, 57.55)

        redAudienceLeft (dx < 0): (49.0, 57.55)


        blueAudienceLeft (dx < 0): (75, 57.55)

        redAudienceRight (dx > 0): (69, 57.55)

        blueGoalRight (dx > 0): (94.7, 88)

        blueGoalLeft (dx < 0): (75, 88)

        redGoalRight (dx > 0): (69, 88)

        redGoalLeft (dx < 0): (49.0, 88)

         */

    //dy + dx multipliers
    public static double dyMultiplier = 0.1;
    public static double dxMultiplier = 0.07;
    public static double centerThreshold = 13.5;


    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball

    /** hardware scheduler tunables (used in HWScheduler.java) **/

    public static long teleTargetLoopLength = 20; // target loop length in ms
    public static long autoTargetLoopLength = 25; // autos generally have longer loops

    public static double stalenessReduction = 1000000; // how important staleness is. higher values are less important
    public static boolean softScheduling = false; // if true (or isDebugging is true): update device once max staleness has been exceeded
}

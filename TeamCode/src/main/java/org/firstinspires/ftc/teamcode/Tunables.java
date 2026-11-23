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

    public static double manualStartingRPM = 3050;
    public static double adjustRPM = 50;
    public static double shootHeadingMargin = Math.toRadians(1);
    public static boolean teleBrake = true;

    /** Robot Dimensions **/

    public static double robotWidthToCenter = 6.72;
    public static double robotLengthToCenter = 6.754;

    /** Adjustable Hood tunables (will be used in Hood.java) **/

    public static double nectarPos = 0.15;
    public static double pollenPos = 0.7;

    public static double flowerRpm = 3700;
    public static double pollenRpm = 3100;

    /** flywheel tunables (used in Flywheel.java) **/

    // PIDF tuned 9-28-2026
    public static double flywheelP = 0.006;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0.0004;
    public static double maxFlywheelBraking = 1.0; // max amount of breaking (negative power) to apply to launch motors
    public static double flywheelVoltageStart = 12.0; // start voltage of compensation scale
    public static double flywheelVoltageEnd = 13.5;   // end voltage of compensation scale
    public static double flywheelCompensationMax = 1.2; // max amount to scale voltage
    public static boolean flywheelUsingVoltageCompensation = false;

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

    public static int    shakeMillis = 80; // flower shake: time between flips, in ms (real period is rounded up to the loop length)
    public static double flowerDown = 0.52;
    public static double flowerUp = 0.008;
    public static double flowerShake = 0.08; // flower shake: servo position amplitude (NOT a time) around flowerDown
    public static double flowerDistance = 13.500; // how far from the edge of the field where the flower is should we refuel

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 650;

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double tagThreshold = 2; // amount of tags we need at least
    public static double shootRPM = 3100; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long flowerRefuelDurationMillis = 625; // how long to intake while driving through the refuel line
    public static long refuelDurationMillis = 150; // how long to intake while driving through the refuel line
    public static double flowerIntakeSpeed = 0.3; // 0-1 speed to intake flower (lower=slower)
    public static double maxTravelToRefuelTime = 1500; // timeout for travel to picking up balls (if it gets stuck on a ball)
    public static double crossFieldHoldUntil = 0.8; // fraction (0-1] of a cross-field path spent holding 90/270 heading before turning to the final heading; read when paths are built at init
    public static double flowerApproachSpeed = .25;

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

    public static double visionMaxSpeed = 81; // in/s, pedro maxAchievableForwardVelocity
    public static double visionJumpMargin = 4; // in, allowed frame noise
    public static int visionJumpRejectsBeforeReset = 5; // jump-rejected frames before vision re-acquires a moved robot
    public static int minTagCount = 2; // minimum amount of tags seen in a frame necessary to update our botpose
    public static double visionPitchCheckTolDeg = 10; // deg, max gap between camera pitch and the pitch the solved HIVE tilt predicts

    /** field tunables (used in Field.java) **/

    // auto RPM
    public static double autoRPMAddition = 0; // add this many RPMs to auto RPM output
    public static double autoRPMMultiplier = 1; // multiply auto RPM output by this

    public static double centerThreshold = 20;

    public static double audienceY = 64;

    public static double scoringY = 80;
    public static double autoRPMSwitchoffAngle = 0.965251;

    public static double maxScoringAbsRelativeX = 36;
    public static double maxScoringRelativeY = 48;
    public static double minScoringRelativeY = 10;

    /** fusion tunables (used in Fusion.java) **/
    public static double fusionStartStd = 2; // how unsure we are about our starting pose after auto
    public static double fusionOdoVariancePerInch = 0.017; // how much odometry uncertainty grows per inch driven (in^2 per in)
    public static double fusionOdoMaxSpeed = 56.53; // Maximum speed of robot
    public static double fusionOdoJumpMargin = 4; // in, allowed odometry noise/timing error before a sample is dropped
    public static int fusionOdoRejectsBeforeReset = 5; // consecutive teleports before accepting a real re-localization
    public static double fusionVisionBaseStd = 0.187; // vision error with ONE tag at fusionVisionRefDistance
    public static double fusionVisionRefDistance = 59; // distance (in) that fusionVisionBaseStd was measured at
    public static double fusionTimingErrorMs = 10; // how far off our vision timing might be; turns driving speed into error
    public static double fusionTurnStd = 1.0; // extra vision error per rad/s of turning (motion blur)
    public static double fusionMinVisionStd = 0.7; // never trust a vision frame more than this, even if everything says it's perfect
    public static double fusionGateSigmas = 3; // reject vision frames more than this many std devs from our estimate
    public static int fusionRejectsBeforeDoubt = 10; // rejected frames in a row before we start doubting odometry instead
    public static double fusionExtraLatencyMs = 0; // extra delay on top of what the Limelight reports; tune if fused pose wobbles when driving

    /** hardware scheduler tunables (used in HWScheduler.java) **/

    public static long teleTargetLoopLength = 20; // target loop length in ms
    public static long autoTargetLoopLength = 25; // autos generally have longer loops

    public static double stalenessReduction = 1; // how important staleness is. higher values are less important
    public static boolean softScheduling = true; // if true (or isDebugging is true): update device once max staleness has been exceeded
}

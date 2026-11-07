package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Tunables {

    public static boolean isDebugging = false; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /** BozoTeleOp tunables (used in BozoTeleOp.java) **/

    public static double manualStartingRPM = 3150;
    public static double adjustRPM = 50;
    public static double shootHeadingMargin = Math.toRadians(5);
    public static boolean teleBrake = true;

    /** Adjustable Hood tunables (will be used in Hood.java) **/

    public static double flowerPos = 0.4;
    public static double pollenPos = 0.6;

    /** flywheel tunables (used in Flywheel.java) **/

    // PIDF tuned 9-28-2026
    public static double flywheelP = 0.008;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0.0004;
    public static double maxFlywheelBraking = 1.0;
    public static double flywheelVoltageStart = 12.0; // start voltage of compensation scale
    public static double flywheelVoltageEnd = 13.5;   // end voltage of compensation scale
    public static double flywheelCompensationMax = 1.2; // max amount to scale voltage

    /** intake tunables (used in Intake.java) **/

    public static boolean intakeUsePowerSave = true;
    public static int intakePowerSaveWaitInterval = 30; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 30; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 6;
    public static double intakePowerSaveTriggerVelocity = 0;
    public static double intakeHoldPower = 0.0; // minimum power for balls to not fall out of intake

    /** transfer tunables (used in Transfer.java) **/

    public static double transferOpen = 0.41;
    public static double transferClosed = 0.108;

    public static double flowerDown = 0.6746;
    public static double flowerUp = 0.16;
    public static double flowerCollect = 0.63;

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 525;

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double tagThreshold = 2; // amount of tags we need at least
    public static double shootRPM = 2975; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long flowerRefuelDurationMillis = 400; // how long to intake while driving through the refuel line
    public static long refuelDurationMillis = 200; // how long to intake while driving through the refuel line
    public static double maxTravelToRefuelTime = 1500;

    /** vision tunables (used in subsys/Vision.java) **/

    // HIVE correction; fit with limelight/fit_hive.py (h, pivot_y, z_offset, z_per_dist)
    public static double visionHivePivotHeight = 43.95; // in
    public static double visionHivePivotY = 72; // in, pedro y
    public static double visionZOffset = 0; // in, subtracted from raw z
    public static double visionZPerDist = 0; // in/in of avg tag distance, subtracted from raw z

    public static double visionCameraErrorForward = 0.356; // in, robot frame
    public static double visionCameraErrorLeft = -3.763;  // in, robot frame
    public static double visionBlueBiasX = -3.426; // in, field frame
    public static double visionBlueBiasY = -0.260;
    public static double visionRedBiasX = -1.477;
    public static double visionRedBiasY = 0.130;

    public static double visionMaxSpeed = 81; // in/s, pedro maxAchievableForwardVelocity
    public static double visionJumpMargin = 4; // in, allowed frame noise
    public static double visionPoseGain = 0.1;
    public static double visionPoseMaxSpeed = 10; // in/s, teleop relocalizes only below this

    public static double visionTunerSpotX = 72;
    public static double visionTunerSpotY = 72;

    /** field tunables (used in Field.java) **/

    // auto RPM
    public static double autoRPMMultiplier = 1; // multiply auto RPM output by this
    public static double autoRPMAddition = 0; // add this many RPMs to auto RPM output

    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball

    /** hardware scheduler tunables (used in HWScheduler.java) **/

    public static long teleTargetLoopLength = 20; // target loop length in ms
    public static long autoTargetLoopLength = 25; // autos generally have longer loops

    public static double stalenessReduction = 1000000; // how important staleness is. higher values are less important
    public static boolean softScheduling = false;
}

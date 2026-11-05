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

    public static double flowerDown = 0.6746;
    public static double flowerUp = 0.16;

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 525;

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double tagThreshold = 2; // amount of tags we need at least
    public static double shootRPM = 2990; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long flowerRefuelDurationMillis = 400; // how long to intake while driving through the refuel line
    public static long refuelDurationMillis = 200; // how long to intake while driving through the refuel line
    public static double maxTravelToRefuelTime = 1500; // timeout for travel to picking up balls (if it gets stuck on a ball)

    /** field tunables (used in Field.java) **/

    // auto RPM
    public static double autoRPMMultiplier = 1; // multiply auto RPM output by this
    public static double autoRPMAddition = 0; // add this many RPMs to auto RPM output

    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball

    /** hardware scheduler tunables (used in HWScheduler.java) **/

    public static double targetLoopLength = 20; // target loop length in ms

    public static double stalenessReduction = 1000000; // how important staleness is. higher values are less important
    public static boolean softScheduling = false; // if true (or isDebugging is true): update device once max staleness has been exceeded
}

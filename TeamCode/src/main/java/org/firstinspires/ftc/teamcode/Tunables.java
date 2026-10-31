/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Tunables {
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = true; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /** BozoTeleOp tunables (used in BozoTeleOp.java) **/

    public static double manualStartingRPM = 3250;
    public static double adjustRPM = 50;
    public static double shootHeadingMargin = Math.toRadians(5);

    /** flywheel tunables (used in Flywheel.java) **/

    // PIDF tuned 9-28-2026
    public static double flywheelP = 0.004;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0.0004;
    public static double maxFlywheelBraking = 1.0; // max amount of breaking (negative power) to apply to launch motors

    /** intake tunables (used in Intake.java) **/

    public static boolean intakeUsePowerSave = true;
    public static int intakePowerSaveWaitInterval = 30; // how often to check for a stalled intake while running
    public static int intakePowerSaveCheckInterval = 30; // how often to update intake power/velocity check in millis
    public static double intakePowerSaveTriggerAmps = 6; // if we are drawing more amps than this, trigger power save
    public static double intakePowerSaveTriggerVelocity = 0; // if we have a tps less than this, trigger power save
    public static double intakeHoldPower = 0.0; // minimum power for balls to not fall out of intake
    public static double intakeForwardPower = 1.0;
    public static double intakeForwardLaunchingPower = .7;

    /** transfer tunables (used in Transfer.java) **/

    public static double transferOpen = 0.41;
    public static double transferClosed = 0.108;

    public static double flowerDown = 0.883;
    public static double flowerUp = 0.413;

    /** robot tunables (used in Robot.java) **/

    public static double launchIntakeStopTime = 80; // time to wait for intake to stop before opening transfer
    public static double launchTransferOpeningTime = 150;
    public static double launchShootingTime = 500;

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double tagThreshold = 2; // amount of tags we need at least
    public static double shootRPM = 3150; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long refuelDurationMillis = 500; // how long to intake while driving through the refuel line
    public static double maxTravelToRefuelTime = 3000; // timeout for travel to picking up balls (if it gets stuck on a ball)

    /** field tunables (used in Field.java) **/

    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball
}

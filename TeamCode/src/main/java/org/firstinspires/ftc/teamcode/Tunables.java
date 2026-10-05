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
    public static double intakeForwardLaunchingPower = 0.5;

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

    /** physics tunables (used in Physics.java) **/

    /* shooter, see subsys/Physics.java and doc/shooter-calibration.md
     * Outtake 1 shoots POLLEN, outtake 2 shoots NECTAR, on separate hoods and wheels.
     * SpeedPerRpm and SpeedIntercept are your measured rpm -> muzzle speed line. They start at
     * zero, which makes the solver refuse to return an RPM until you have measured them. */

    // outtake 1, POLLEN
    public static double pollenLaunchAngleDeg = 65.0; // hood angle above horizontal, degrees
    public static double pollenLaunchHeightIn = 10.704; // ball exit height above the TILES, inches
    public static double pollenLaunchOffsetIn = 0.0; // exit point ahead of the odo centre, inches
    public static double pollenSpeedPerRpm = 0.0; // m/s per RPM, slope of your measured line
    public static double pollenSpeedIntercept = 0.0; // m/s at zero RPM, its intercept
    public static double pollenDragCoefficient = 0.5;

    // outtake 2, NECTAR
    public static double nectarLaunchAngleDeg = 65.0;
    public static double nectarLaunchHeightIn = 10.704;
    public static double nectarLaunchOffsetIn = 0.0;
    public static double nectarSpeedPerRpm = 0.0;
    public static double nectarSpeedIntercept = 0.0;
    public static double nectarDragCoefficient = 0.5;
}

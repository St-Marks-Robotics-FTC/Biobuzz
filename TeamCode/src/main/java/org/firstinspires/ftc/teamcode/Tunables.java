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

    /** auto shooting sequence tunables (used in auto/BozoAuto.java) **/

    public static double shootRPM = 3000; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long feedDurationMillis = 1500; // how long to run the transfer + intake to feed balls through the flywheel
    public static long refuelDurationMillis = 1500; // how long to intake while driving through the refuel line

    /** field tunables (used in Field.java) **/

    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    public static Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    public static Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    public static Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    public static Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

    public static double maxScoringAngle = Math.toRadians(40); // max angle in radians goal can accept ball
}

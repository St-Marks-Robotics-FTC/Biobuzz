/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tunables {
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = true; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /** BozoTeleOp tunables (used in BozoTeleOp.java) **/
    public static double adjustRPM = 50;

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
    public static double launchOpenTime = 100;
    public static double flywheelMarginRPM = 100;
}

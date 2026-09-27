/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tunables {
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = false; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    /* shooter, see subsys/physics.java and doc/shooter-calibration.md
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

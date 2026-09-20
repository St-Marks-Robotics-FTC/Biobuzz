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
     * outtake 1 shoots POLLEN, outtake 2 shoots NECTAR; each has its own hood and wheel, so each
     * gets its own geometry and its own RPM -> muzzle speed line. speedPerRpm and speedIntercept
     * are the fit of that measured line; seed speedPerRpm with physics.seedSpeedPerRpm(). */

    // outtake 1, POLLEN
    public static double pollenLaunchAngleDeg = 65.0; // hood angle above horizontal, degrees
    public static double pollenLaunchHeightIn = 12.0; // ball exit height above the TILES, inches
    public static double pollenLaunchOffsetIn = 0.0; // exit point ahead of the odo centre, inches
    public static double pollenSpeedPerRpm = 0.00266; // m/s of muzzle speed per RPM
    public static double pollenSpeedIntercept = 0.0; // m/s at zero RPM, usually slightly negative
    public static double pollenDragCoefficient = 0.5;

    // outtake 2, NECTAR
    public static double nectarLaunchAngleDeg = 65.0;
    public static double nectarLaunchHeightIn = 12.0;
    public static double nectarLaunchOffsetIn = 0.0;
    public static double nectarSpeedPerRpm = 0.00266;
    public static double nectarSpeedIntercept = 0.0;
    public static double nectarDragCoefficient = 0.5;
}

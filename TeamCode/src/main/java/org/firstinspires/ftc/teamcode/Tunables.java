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

    // shooter, see subsys/physics.java
    public static double launchAngleDeg = 65.0; // fixed hood angle above horizontal, degrees
    public static double launchHeightIn = 12.0; // ball exit height above the TILES, inches
    public static double launchOffsetIn = 0.0; // ball exit point ahead of the odo centre, inches
    public static double flywheelDiameterIn = 4.0; // inches
    public static double flywheelTransfer = 0.5; // ball speed / wheel surface speed, measure this
    public static double ballDragCoefficient = 0.5;
}

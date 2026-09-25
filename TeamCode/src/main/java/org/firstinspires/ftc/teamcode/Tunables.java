/** class that holds all of our constants that we want to be able to tune quickly
 * all values in are milliseconds unless otherwise stated
 */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsys.PIDF;

@Configurable
public class Tunables {
    // all members must be initialized with the keywords `public static`... in order to be usable

    public static boolean isDebugging = false; // whether to print additional debug info (slow)
    public static double turnRateMultiplier = 0.75; // reduce turn speed in TeleOp to 75%

    // PIDF values for subsys/Flywheel.java
    public static double flywheelP = 0;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0;

    // Transfer Servo open/closed positions
    public static double transferOpen = 1.0;
    public static double transferClosed = 0.0;
}

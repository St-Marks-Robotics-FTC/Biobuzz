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
    public static double flywheelP = 1;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelF = 0;

    // shooting sequence, used in auto/BozoAuto.java
    public static double shootRPM = 3000; // target flywheel RPM while lined up to shoot
    public static double flywheelRPMMargin = 75; // how close to shootRPM we must be before feeding balls
    public static long feedDurationMillis = 1500; // how long to run the transfer + intake to feed balls through the flywheel
    public static long refuelDurationMillis = 1500; // how long to intake while driving through the refuel line

    // Transfer Servo open/closed positions
    public static double transferOpen = 1.0;
    public static double transferClosed = 0.0;
}

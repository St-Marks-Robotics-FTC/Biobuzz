package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Tunables {
    public static boolean isDebugging = false;
    public static double turnRateMultiplier = 0.75;
    public static double slowModeMin = 0.2;
    public static double minShootDistance = 30.0;
    public static double maxShootDistance = 45.0;
    public static double maxAngleErrorDeg = 35.0;
    public static double aimP = 2.0; //PIDF can be tuned
    public static double aimI = 0.0;
    public static double aimD = 0.05;
    public static double aimF = 0.0;
    public static double aimMaxPower = 0.8;
    public static double aimToleranceDeg = 2.0; //Gap between target and acceptable actual
    public static double aimIntegralLimit = 1.0;
    public static double shooterTargetRpm = 3000.0;
    public static double shooterRampRpmPerSec = 2500.0;
    public static double shooterToleranceRpm = 120.0; //Gap between target and acceptable actual
    public static double shooterFireIntervalSec = 0.4; //Minimum time passing between shots
    public static long shootTimeMs = 1000;
    public static long intakeTimeMs = 1000;
    public static long pathTimeoutMs = 10000;
}

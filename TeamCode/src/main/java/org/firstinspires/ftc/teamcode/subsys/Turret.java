package org.firstinspires.ftc.teamcode.subsys;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

@Configurable
public class Turret {
    /** configurables **/

    // servo amounts
    public static double SERVO_MAX_LEFT = 0;
    public static double SERVO_MAX_RIGHT  = 0;

    // turret angles (radians)
    public static double TURRET_MAX_LEFT = Math.toRadians(0);
    public static double TURRET_MAX_RIGHT = Math.toRadians(0);

    /** end configurables **/

    private final ServoDevice servo;
    private double lastAngle;
    private boolean isLocked;

    public Turret(IoThread io) {
        servo = new ServoDevice(io, "turretServo", 0.5, WritePolicy.DEFAULT); // centered until the first updateAngle
        lastAngle = 0;
    }

    public void updateAngle(double angle) {
        double pos = Range.scale(angle , TURRET_MAX_RIGHT, TURRET_MAX_LEFT, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);

        servo.setPosition(pos);

        lastAngle = angle;
    }

    public void updateRawPos(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        servo.setPosition(pos);
    }

    public double getLastAngle() {
        return lastAngle;
    }

    public void lock() {
        isLocked = true;
    }

    public void unlock() {
        isLocked = false;
    }

    public boolean isLocked() {
        return isLocked;
    }
}

package org.firstinspires.ftc.teamcode.subsys;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

@Configurable
public class Turret {
    /** configurables **/

    // servo amounts
    public static double SERVO_MAX_LEFT = 0;
    public static double SERVO_MAX_RIGHT  = 0;

    // turret angles (radians)
    public static double TURRET_MAX_LEFT = 0;
    public static double TURRET_MAX_RIGHT = 0;

    /** end configurables **/

    private SServo servo;
    private double lastAngle;
    private boolean isLocked;

    public Turret(HWScheduler hwScheduler) {
        servo = new SServo("turretServo", 1, hwScheduler.hw());
        hwScheduler.addDevice(servo);
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

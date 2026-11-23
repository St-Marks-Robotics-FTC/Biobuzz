package org.firstinspires.ftc.teamcode.subsys;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

@Configurable
public class Turret {
    // servo amounts
    public static double servoMaxLeft = 0;
    public static double servoMaxRight = 0;

    // turret angles (radians)
    public static double turretMaxLeft = 0;
    public static double turretMaxRight = 0;

    private SServo servo;
    private double lastAngle;
    private boolean isLocked;

    public Turret(HWScheduler hwScheduler) {
        servo = new SServo("turretServo", 1, hwScheduler.hw());
        hwScheduler.addDevice(servo);
        lastAngle = 0;
    }

    public void updateAngle(double angle) {
        double pos = Range.scale(angle , turretMaxRight, turretMaxLeft, servoMaxRight, servoMaxLeft);

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

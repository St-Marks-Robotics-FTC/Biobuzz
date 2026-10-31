/** hardware-scheduled servo wrapper **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SServo extends Device {
    public Servo s;

    private double lastPos = 0; // last position we wrote to the servo
    private double setPos = 0; // position we want the servo to be set to

    public SServo(String name, double priorityMultiplier, HardwareMap hw) {
        super(name, priorityMultiplier);

        estimatedUpdateLength = 3; // estimated update time

        s = hw.get(Servo.class, name);
    }

    public void setPosition(double newPos) {
        setPos = newPos;
    }

    public double getPosition() {
        return lastPos;
    }

    protected double getChange() {
        return Math.abs(lastPos - setPos);
    }

    protected void write() {
        s.setPosition(setPos);
        lastPos = setPos;
    }
}

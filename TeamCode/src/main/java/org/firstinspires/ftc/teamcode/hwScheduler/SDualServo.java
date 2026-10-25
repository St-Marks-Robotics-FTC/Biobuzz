/** hardware-scheduled servo wrapper **/

package org.firstinspires.ftc.teamcode.hwScheduler;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SDualServo extends Device {
    public Servo servo1, servo2;

    private double lastPos = 0; // last position we wrote to the servo
    private double setPos = 0; // position we want the servo to be set to

    public SDualServo(String name1, String name2, double priorityMultiplier, HardwareMap hw) {
        super(name1 + name2, priorityMultiplier);

        lastUpdateLength = 6; // estimated update time

        servo1 = hw.get(Servo.class, name1);
        servo2 = hw.get(Servo.class, name2);
    }

    public void setPosition(double newPos) {
        setPos = newPos;
    }

    public double getPosition() {
        return lastPos;
    }

    public double getPriority() {
        double priority = Math.abs(lastPos - setPos) * priorityMultiplier;
        return priority;
    }

    public void write() {
        servo1.setPosition(setPos);
        servo2.setPosition(setPos);
        lastPos = setPos;
    }
}

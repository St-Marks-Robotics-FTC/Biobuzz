package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;
public class Flower {
    private Servo flowerServo;

    private enum State {
        UP,
        DOWN
    }

    private Flower.State state = Flower.State.DOWN;

    public Flower(HardwareMap hw) {
        flowerServo = hw.get(Servo.class, "flowerServo");

        flowerServo.setPosition(Tunables.flowerDown); // start down
    }

    public void toggle() {
        if (isUp()) goDown();
        else goUp();
    }

    public void goUp() {
        if (state != Flower.State.UP) {
            flowerServo.setPosition(Tunables.flowerUp);
            state = Flower.State.UP;
        }
    }

    public void goDown() {
        if (state != Flower.State.DOWN) {
            flowerServo.setPosition(Tunables.flowerDown);
            state = Flower.State.DOWN;
        }
    }

    public boolean isUp() {
        return state == Flower.State.UP;
    }

    public void setServoRaw(double pos) {
        flowerServo.setPosition(pos);
    }
}

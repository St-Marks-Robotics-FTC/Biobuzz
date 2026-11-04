package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;

public class Flower {
    private final Servo flowerServo;

    private enum State {
        UP,
        DOWN
    }

    // the constructor raises the servo, so the state must start UP or goDown() would skip the move
    private State state = State.UP;

    public Flower(HardwareMap hw) {
        flowerServo = hw.get(Servo.class, "flowerServo");

        flowerServo.setPosition(Tunables.flowerUp); // start up
    }

    public void goUp() {
        if (state != State.UP) {
            flowerServo.setPosition(Tunables.flowerUp);
            state = State.UP;
        }
    }

    public void goDown() {
        if (state != State.DOWN) {
            flowerServo.setPosition(Tunables.flowerDown);
            state = State.DOWN;
        }
    }

    public void setServoRaw(double pos) { // ServoTuner only; bypasses state tracking
        flowerServo.setPosition(pos);
    }
}

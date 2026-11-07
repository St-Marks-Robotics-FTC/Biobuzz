package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

public class Flower {
    private SServo flowerServo;

    private enum State {
        UP,
        COLLECTING,
        DOWN
    }

    /** Must match the position commanded in the constructor; goUp/goDown skip the write when the state already matches. */
    private Flower.State state = Flower.State.UP;

    public Flower(HWScheduler scheduler) {
        flowerServo = new SServo("flowerServo", 1, scheduler.hw());
        scheduler.addDevice(flowerServo);

        flowerServo.setPosition(Tunables.flowerUp); // start up
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

    public void goCollect() {
        if (state != State.COLLECTING) {
            flowerServo.setPosition(Tunables.flowerCollect);
            state = Flower.State.COLLECTING;
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

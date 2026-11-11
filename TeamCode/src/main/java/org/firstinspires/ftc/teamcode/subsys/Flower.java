package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

public class Flower {
    private SServo flowerServo;

    private enum State {
        UP,
        SHAKE_HIGH,
        SHAKE_LOW,
        DOWN
    }

    /** Must match the position commanded in the constructor; goUp/goDown skip the write when the state already matches. */
    private Flower.State state = Flower.State.UP;
    private Timer shakeTimer;

    public Flower(HWScheduler scheduler) {
        flowerServo = new SServo("flowerServo", 1, scheduler.hw());
        scheduler.addDevice(flowerServo);

        shakeTimer = new Timer();

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

    public void startShake() {
        if (state != State.SHAKE_HIGH && state != State.SHAKE_LOW) {
            flowerServo.setPosition(Tunables.flowerDown + (2 * Tunables.flowerShake));
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
        }
    }

    public void update() { // shake n bake
        if (state == State.SHAKE_HIGH && shakeTimer.milliseconds() > Tunables.flowerShake) {
            flowerServo.setPosition(Tunables.flowerDown + Tunables.flowerShake);
            state = State.SHAKE_LOW;

            shakeTimer.reset();
        } else if (state == State.SHAKE_LOW && shakeTimer.milliseconds() > Tunables.flowerShake) {
            flowerServo.setPosition(Tunables.flowerDown + (2 * Tunables.flowerShake));
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
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

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

public class Flower {
    private final ServoDevice flowerServo;

    private enum State {
        UP,
        SHAKE_HIGH,
        SHAKE_LOW,
        DOWN
    }

    /** Must match the position commanded in the constructor; goUp/goDown skip the write when the state already matches. */
    private Flower.State state = Flower.State.UP;
    private Timer shakeTimer;

    public Flower(IoThread io) {
        // start up; the I/O thread writes it when it starts, so the servo moves during init
        flowerServo = new ServoDevice(io, "flowerServo", Tunables.flowerUp, WritePolicy.DEFAULT);

        shakeTimer = new Timer();
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
            flowerServo.setPosition(shakeHighPosition());
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
        }
    }

    public void update() { // shake n bake
        if (state == State.SHAKE_HIGH && shakeTimer.milliseconds() > Tunables.shakeMillis) {
            flowerServo.setPosition(Tunables.flowerDown);
            state = State.SHAKE_LOW;

            shakeTimer.reset();
        } else if (state == State.SHAKE_LOW && shakeTimer.milliseconds() > Tunables.shakeMillis) {
            flowerServo.setPosition(shakeHighPosition());
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
        }
    }

    /** servo position of the far end of the shake; {@link Tunables#flowerShake} is a servo position, not a time **/
    private static double shakeHighPosition() {
        return Tunables.flowerDown - (Tunables.flowerShake);
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

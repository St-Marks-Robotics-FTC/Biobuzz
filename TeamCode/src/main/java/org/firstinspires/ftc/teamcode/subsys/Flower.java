package org.firstinspires.ftc.teamcode.subsys;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

@Configurable
public class Flower {
    /** configurables **/

    public static double PRIORITY_MULTIPLIER = 1.0;
    public static int    SHAKE_MILLIS = 80; // flower shake: time between flips, in ms (real period is rounded up to the loop length)
    public static double POS_DOWN = 0.52;
    public static double POS_UP = 0.008;
    public static double SHAKE_AMOUNT = 0.08; // flower shake: servo position amplitude (NOT a time) around flowerDown

    /** end configurables **/

    private final ServoDevice flowerServo;

    private enum State {
        UP,
        SHAKE_HIGH,
        SHAKE_LOW,
        DOWN
    }

    /** Must match the position commanded in the constructor; goUp/goDown skip the write when the state already matches. */
    private Flower.State state = Flower.State.UP;
    private final Timer shakeTimer;

    public Flower(IoThread io) {
        // start up; the I/O thread writes it when it starts, so the servo moves during init
        flowerServo = new ServoDevice(io, "flowerServo", POS_UP, new WritePolicy(PRIORITY_MULTIPLIER, 100));

        shakeTimer = new Timer();
    }

    public void goUp() {
        if (state != Flower.State.UP) {
            flowerServo.setPosition(POS_UP);
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
        if (state == State.SHAKE_HIGH && shakeTimer.milliseconds() > SHAKE_MILLIS) {
            flowerServo.setPosition(POS_DOWN);
            state = State.SHAKE_LOW;

            shakeTimer.reset();
        } else if (state == State.SHAKE_LOW && shakeTimer.milliseconds() > SHAKE_MILLIS) {
            flowerServo.setPosition(shakeHighPosition());
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
        }
    }

    private static double shakeHighPosition() {
        return POS_DOWN - (SHAKE_AMOUNT);
    }

    public void goDown() {
        if (state != Flower.State.DOWN) {
            flowerServo.setPosition(POS_DOWN);
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

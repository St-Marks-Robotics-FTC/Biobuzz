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

    /**
     * Starts oscillating the flower around {@link Tunables#flowerDown}; {@link #update()} must be called every loop
     * to keep it going. No-op if already shaking. Call {@link #goDown()} or {@link #goUp()} to stop.
     */
    public void startShake() {
        if (state != State.SHAKE_HIGH && state != State.SHAKE_LOW) {
            flowerServo.setPosition(shakeHighPosition());
            state = State.SHAKE_HIGH;

            shakeTimer.reset();
        }
    }

    /**
     * Advances the shake: flips between the high and low shake position once {@link Tunables#shakeMillis} has passed
     * since the last flip. No-op unless shaking. A flip happens at most once per call, so the real half-period is
     * {@code shakeMillis} rounded up to the next loop.
     */
    public void update() { // shake n bake
        if (state == State.SHAKE_HIGH && shakeTimer.milliseconds() > Tunables.shakeMillis) {
            flowerServo.setPosition(shakeLowPosition());
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
        return Tunables.flowerDown + (2 * Tunables.flowerShake);
    }

    /** servo position of the near end of the shake **/
    private static double shakeLowPosition() {
        return Tunables.flowerDown + (0.5 * Tunables.flowerShake);
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

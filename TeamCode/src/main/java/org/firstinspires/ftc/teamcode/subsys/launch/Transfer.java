package org.firstinspires.ftc.teamcode.subsys.launch;

import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

public class Transfer {
    /** configurables **/

    public static double PRIORITY_MULTIPLIER = 1; // open/close is a small position change, but VITAL
    public static double POS_OPEN = 0.41;
    public static double POS_CLOSED = 0.108;

    /** end configurables **/

    private SServo servo;

    private enum State {
        OPEN,
        CLOSED
    }

    private State state = State.CLOSED;

    public Transfer(HWScheduler scheduler, String ballType) {
        servo = new SServo(ballType + "TransferServo", PRIORITY_MULTIPLIER, scheduler.hw());
        scheduler.addDevice(servo);

        servo.setPosition(POS_CLOSED); // start closed
        servo.forceUpdate(); // write now so the servo moves during init instead of waiting for the first loop
    }

    public void toggle() {
        if (isOpen()) close();
        else open();
    }

    public void open() {
        if (state != State.OPEN) {
            servo.setPosition(POS_OPEN);
            state = State.OPEN;
        }
    }

    public void close() {
        if (state != State.CLOSED) {
            servo.setPosition(POS_CLOSED);
            state = State.CLOSED;
        }
    }

    public boolean isOpen() {
        return state == State.OPEN;
    }

    public void setServoRaw(double pos) {
        servo.setPosition(pos);
    }
}

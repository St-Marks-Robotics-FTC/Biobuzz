package org.firstinspires.ftc.teamcode.subsys.launch;

import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

public class Transfer {
    /** configurables **/

    public static double PRIORITY_MULTIPLIER = 1; // open/close is a small position change, but VITAL
    public static double POS_OPEN = 0.41;
    public static double POS_CLOSED = 0.108;

    /** end configurables **/

    private final ServoDevice servo;

    private enum State {
        OPEN,
        CLOSED
    }

    private State state = State.CLOSED;

    public Transfer(IoThread io, String ballType) {
        // starts closed; the I/O thread writes it when it starts, so the servo moves during init
        servo = new ServoDevice(io, ballType + "TransferServo", POS_CLOSED, new WritePolicy(PRIORITY_MULTIPLIER, 100));
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

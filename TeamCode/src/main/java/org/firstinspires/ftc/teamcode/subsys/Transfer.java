package org.firstinspires.ftc.teamcode.subsys;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

public class Transfer {
    private static final double PRIORITY_MULTIPLIER = 1; // open/close is a small position change, but VITAL

    private final ServoDevice servo;

    private enum State {
        OPEN,
        CLOSED
    }

    private State state = State.CLOSED;

    public Transfer(IoThread io) {
        // starts closed; the I/O thread writes it when it starts, so the servo moves during init
        servo = new ServoDevice(io, "transferServo", Tunables.transferClosed, new WritePolicy(PRIORITY_MULTIPLIER, 100));
    }

    public void toggle() {
        if (isOpen()) close();
        else open();
    }

    public void open() {
        if (state != State.OPEN) {
            servo.setPosition(Tunables.transferOpen);
            state = State.OPEN;
        }
    }

    public void close() {
        if (state != State.CLOSED) {
            servo.setPosition(Tunables.transferClosed);
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

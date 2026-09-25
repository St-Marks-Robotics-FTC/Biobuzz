package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public enum State {OPEN, CLOSED}

    // placeholder positions, tune once the servo is mounted
    private static final double OPEN_POSITION = 1.0;
    private static final double CLOSED_POSITION = 0.0;

    private final Servo servo;
    private State state = State.CLOSED;

    public Transfer(HardwareMap hw) {
        servo = hw.get(Servo.class, "transfer");
        close();
    }

    public State getState() {
        return state;
    }

    public void open() {
        state = State.OPEN;
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        state = State.CLOSED;
        servo.setPosition(CLOSED_POSITION);
    }
}

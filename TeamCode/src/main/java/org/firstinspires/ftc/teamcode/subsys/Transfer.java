package org.firstinspires.ftc.teamcode.subsys;

import static org.firstinspires.ftc.teamcode.Tunables.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public enum State {OPEN, CLOSED}

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
        servo.setPosition(transferOpen);
    }

    public void close() {
        state = State.CLOSED;
        servo.setPosition(transferClosed);
    }
}

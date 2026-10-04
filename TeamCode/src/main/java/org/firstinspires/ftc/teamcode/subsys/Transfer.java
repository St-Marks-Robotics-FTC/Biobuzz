package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tunables;

public class Transfer {
    private Servo servo;

    private enum State {
        OPEN,
        CLOSED
    }

    private State state = State.CLOSED;

    public Transfer(HardwareMap hw) {
        servo = hw.get(Servo.class, "transferServo");

        servo.setPosition(Tunables.transferClosed); // start closed
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

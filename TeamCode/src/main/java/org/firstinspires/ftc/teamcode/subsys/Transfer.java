package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

public class Transfer {
    private static final double PRIORITY_MULTIPLIER = 4; // open/close is a small position change, but VITAL

    private SServo servo;

    private enum State {
        OPEN,
        CLOSED
    }

    private State state = State.CLOSED;

    public Transfer(HardwareMap hw, HWScheduler scheduler) {
        servo = new SServo("transferServo", PRIORITY_MULTIPLIER, hw);
        scheduler.addDevice(servo);

        servo.setPosition(Tunables.transferClosed); // start closed
        servo.forceUpdate(); // write now so the servo moves during init instead of waiting for the first loop
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

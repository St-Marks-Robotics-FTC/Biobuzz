package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public enum State {OFF, FORWARD, REVERSE}

    private final DcMotor motor;
    private State state = State.OFF;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "intake");
    }

    public State getState() {
        return state;
    }

    public void off() {
        state = State.OFF;
        motor.setPower(0);
    }

    public void forward() {
        state = State.FORWARD;
        motor.setPower(1.0);
    }

    public void reverse() {
        state = State.REVERSE;
        motor.setPower(-1.0);
    }
}

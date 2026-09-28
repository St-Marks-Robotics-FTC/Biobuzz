package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;

public class Intake {
    private enum State {
        OFF,
        FORWARD,
        REVERSE
    }

    private State state = State.OFF;

    private DcMotorEx motor;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "intakeMotor");

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder
    }

    public void off() {
        if (state != State.OFF) {
            motor.setPower(0);
            state = State.OFF;
        }
    }

    public void forward() {
        if (state != State.FORWARD) {
            motor.setPower(Tunables.intakeForwardPower);
            state = State.FORWARD;
        }
    }

    public void toggle() {
        if (state != State.FORWARD) {
            forward();
        } else {
            off();
        }
    }

    public void reverse() {
        if (state != State.REVERSE) {
            motor.setPower(-1);
            state = State.REVERSE;
        }
    }

    public void toggleReverse() {
        if (state == State.REVERSE) {
            forward();
        } else {
            reverse();
        }
    }

    /** getter methods **/

    public boolean isOff() {
        return state == State.OFF;
    }

    public boolean isForward() {
        return state == State.FORWARD;
    }
    public boolean isReverse() {
        return state == State.REVERSE;
    }

    public double getCurrent() { return motor.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public double getVelocity() { return motor.getVelocity(); } // get velocity in tps
}

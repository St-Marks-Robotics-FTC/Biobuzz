package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SMotor;

public class Intake {
    private static final double PRIORITY_MULTIPLIER = 1;

    public enum State {
        OFF,
        FORWARD,
        FORWARD_LAUNCHING,
        REVERSE
    }

    private State state = State.OFF;

    private SMotor motor; // writes go through the scheduler, reads go straight to motor.m

    public Intake(HardwareMap hw, HWScheduler scheduler) {
        motor = new SMotor("intakeMotor", PRIORITY_MULTIPLIER, hw);
        scheduler.addDevice(motor);

        // one-time config, so it doesn't need to be scheduled
        motor.m.setDirection(DcMotorEx.Direction.FORWARD);
        motor.m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        motor.m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // we're just running our intake at 100% speed all the time, so we don't need the encoder
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

    public void forwardLaunching() {
        if (state != State.FORWARD_LAUNCHING) {
            motor.setPower(Tunables.intakeForwardLaunchingPower);
            state = State.FORWARD_LAUNCHING;
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

    public State getState() { return state; }
    public double getCurrent() { return motor.m.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public double getVelocity() { return motor.m.getVelocity(); } // get velocity in tps
}

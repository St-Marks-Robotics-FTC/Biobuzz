package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SMotor;

public class Intake {
    private static final double PRIORITY_MULTIPLIER = 1;

    public enum State {
        OFF, // intake completely unpowered
        FORWARD, // intake full power
        POWER_SAVE, // saving intake power
        HOLD,
        REVERSE, // intake full power reverse
    }

    public enum PowerSaveTrigger {
        CURRENT,
        VELOCITY,
        NOT_TRIGGERED
    }

    private State state = State.OFF;
    private Timer powerSaveWaitTimer;
    private Timer powerSaveCheckTimer;
    private PowerSaveTrigger powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
    double lastIntakeHoldPower = Tunables.intakeHoldPower;

    private SMotor motor; // writes go through the scheduler, reads go straight to motor.m

    public Intake(HWScheduler scheduler) {
        motor = new SMotor("intakeMotor", PRIORITY_MULTIPLIER, scheduler.hw());
        scheduler.addDevice(motor);

        // one-time config, so it doesn't need to be scheduled
        motor.m.setDirection(DcMotorEx.Direction.FORWARD);
        motor.m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); // don't brake when we turn off the motor
        motor.m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        powerSaveWaitTimer = new Timer();
        powerSaveCheckTimer = new Timer();
    }
    public void update(boolean overridePowerSave) { // need to continuously update for power save function and caching
        switch (state) {
            case OFF:
                off();
                break;
            case FORWARD:
                forward();
                if (Tunables.intakeUsePowerSave && !overridePowerSave) {
                    if (powerSaveCheckTimer.milliseconds() >= Tunables.intakePowerSaveCheckInterval) {
                        if (isCurrentTrigger()) {
                            powerSaveTrigger = PowerSaveTrigger.CURRENT;
                            enablePowerSave();
                            return;
                        } else if (isVelocityTrigger()) {
                            powerSaveTrigger = PowerSaveTrigger.VELOCITY;
                            enablePowerSave();
                            return;
                        } else {
                            powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
                            powerSaveCheckTimer.reset(); // wait Tunables.intakePowerSaveUpdateInterval before next reading
                            return;
                        }
                    }
                    // else: don't waste loop time
                }
                break;
            case POWER_SAVE:
                if (powerSaveWaitTimer.milliseconds() >= Tunables.intakePowerSaveWaitInterval || overridePowerSave) {
                    // we've waiting long enough, turn intake back on
                    forward();
                }
                break;
            case HOLD:
                hold();
                if (Tunables.intakeHoldPower != lastIntakeHoldPower) {
                    motor.setPower(Tunables.intakeHoldPower);
                    lastIntakeHoldPower = Tunables.intakeHoldPower;
                }
                break;
            case REVERSE:
                reverse();
                break;
        }
    }

    // we check states to prevent unnecessary writes

    public void off() {
        if (state != State.OFF) {
            motor.setPower(0);
            state = State.OFF;
        }
    }

    public void forward() {
        if (state != State.FORWARD) {
            motor.setPower(1);
            powerSaveTrigger = PowerSaveTrigger.NOT_TRIGGERED;
            state = State.FORWARD;
            powerSaveCheckTimer.reset();
        }
    }

    public void toggle() {
        if (state != State.FORWARD) {
            forward();
        } else {
            off();
        }
    }

    public void hold() {
        if (state != State.HOLD) {
            motor.setPower(Tunables.intakeHoldPower);
            state = State.HOLD;
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

    public boolean isCurrentTrigger() { // whether our current amps is over our trigger amps
        return getCurrent() >= Tunables.intakePowerSaveTriggerAmps;
    }
    public boolean isVelocityTrigger() { // whether our current velocity is under our trigger velocity
        return getVelocity() <= Tunables.intakePowerSaveTriggerVelocity;
    }
    private void enablePowerSave() {
        state = State.POWER_SAVE;
        powerSaveWaitTimer.reset();
        motor.setPower(Tunables.intakeHoldPower); // save power
    }

    /** getter methods **/

    public boolean isOff() {
        return state == State.OFF;
    }

    public boolean isForward() {
        return state == State.FORWARD;
    }

    public boolean isHold() {
        return state == State.HOLD;
    }

    public boolean isReverse() {
        return state == State.REVERSE;
    }

    public PowerSaveTrigger getPowerSaveTrigger() { return powerSaveTrigger; }
    public State getState() { return state; }
    public double getCurrent() { return motor.m.getCurrent(CurrentUnit.AMPS); } // return intake current in amps
    public double getVelocity() { return motor.m.getVelocity(); } // get velocity in tps
}

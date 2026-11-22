package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.MotorDevice;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.io.TimedValue;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

public class Intake {
    private static final double PRIORITY_MULTIPLIER = 1;

    public enum State {
        OFF, // intake completely unpowered
        FORWARD, // intake full power
        POWER_SAVE, // saving intake power
        HOLD, // intake partially powered, just enough to keep balls in but save power
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

    private long forwardSinceNanos = 0; // when FORWARD began; power save ignores current read before this

    private final MotorDevice motor; // writes and reads run on the I/O thread

    public Intake(IoThread io) {
        motor = new MotorDevice(io, "intakeMotor", new MotorDevice.Config()
                .direction(DcMotorSimple.Direction.FORWARD)
                .zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT) // don't brake when we turn off the motor
                .readEncoder() // velocity power-save trigger; free from the bulk read
                .readCurrent(ReadPeriod.millis(() -> Tunables.intakePowerSaveCheckInterval)) // one read per power-save check
                .writePolicy(new WritePolicy(PRIORITY_MULTIPLIER, 100)));

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
                        // now we check which of our power save triggers hit
                        // we should ideally do this in order of what reads are the quickest / most accurate
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
                // waiting with intake off and then after a while going back to on
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
            forwardSinceNanos = System.nanoTime();
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
        TimedValue current = motor.current();
        if (current.nanoTime() < forwardSinceNanos) return false; // read before we went forward: says nothing about a stall now
        return current.value() >= Tunables.intakePowerSaveTriggerAmps;
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
    public double getCurrent() { return motor.current().value(); } // newest intake current in amps
    public double getVelocity() { return motor.encoder().velocityTps(); } // newest velocity in tps
}

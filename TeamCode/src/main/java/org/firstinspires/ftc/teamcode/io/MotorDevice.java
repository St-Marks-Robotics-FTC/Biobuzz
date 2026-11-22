package org.firstinspires.ftc.teamcode.io;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A DC motor whose power writes and encoder/current reads run on the I/O thread.
 * Loop thread: {@link #setPower} and {@link #setZeroPowerBehavior} record commands; {@link #encoder()} and
 * {@link #current()} return the newest published readings.
 */
public final class MotorDevice extends IoDevice {
    private static final double INITIAL_WRITE_ESTIMATE_MILLIS = 2; // one Lynx command over the Control Hub UART

    /** One-time setup, applied in the constructor before the I/O thread starts. */
    public static final class Config {
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private boolean readEncoder = false;
        private ReadPeriod currentReads = ReadPeriod.never();
        private WritePolicy writePolicy = WritePolicy.DEFAULT;

        public Config direction(DcMotorSimple.Direction direction) { this.direction = direction; return this; }

        /** initial zero power behavior; change it later with {@link MotorDevice#setZeroPowerBehavior} */
        public Config zeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) { this.zeroPowerBehavior = behavior; return this; }

        /** read position and velocity every cycle; nearly free because they come from the hub's bulk read */
        public Config readEncoder() { this.readEncoder = true; return this; }

        /** read current draw; every read is its own Lynx command (~2 ms), so keep the period as slow as possible */
        public Config readCurrent(ReadPeriod period) { this.currentReads = period; return this; }

        public Config writePolicy(WritePolicy policy) { this.writePolicy = policy; return this; }
    }

    /** Encoder position and velocity from one bulk read. Immutable. */
    public static final class EncoderReading {
        private final int position;
        private final double velocityTps;
        private final long nanoTime;

        EncoderReading(int position, double velocityTps, long nanoTime) {
            this.position = position;
            this.velocityTps = velocityTps;
            this.nanoTime = nanoTime;
        }

        /** encoder position in ticks */
        public int position() { return position; }

        /** encoder velocity in ticks per second */
        public double velocityTps() { return velocityTps; }

        /** {@link System#nanoTime()} right after the read */
        public long nanoTime() { return nanoTime; }
    }

    private final DcMotorEx motor; // I/O thread only after start
    private final boolean readsEncoder;

    // loop thread -> I/O thread
    private volatile double commandedPower = 0;
    private volatile DcMotor.ZeroPowerBehavior commandedZeroPower;
    private volatile ReadPeriod currentReads;

    // I/O thread only
    private double ioWrittenPower = Double.NaN; // NaN until the first write, so the first write always happens
    private DcMotor.ZeroPowerBehavior ioWrittenZeroPower = null;
    private boolean hasReadCurrent = false;
    private long lastCurrentReadNanos = 0;

    // I/O thread -> loop thread
    private volatile double writtenPower = 0;
    private volatile EncoderReading encoder = null;
    private volatile TimedValue current = null;

    /** looks up {@code name} as a DcMotorEx, applies direction and RUN_WITHOUT_ENCODER (we run our own control loops) */
    public MotorDevice(IoThread io, String name, Config config) {
        super(io, name, config.writePolicy, INITIAL_WRITE_ESTIMATE_MILLIS);
        motor = io.hw().get(DcMotorEx.class, name);
        motor.setDirection(config.direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        readsEncoder = config.readEncoder;
        commandedZeroPower = config.zeroPowerBehavior;
        currentReads = config.currentReads;
    }

    /**
     * @param power in [-1, 1]
     * @throws IllegalArgumentException if power is NaN or outside [-1, 1]
     */
    public void setPower(double power) {
        if (Double.isNaN(power) || power < -1 || power > 1) throw new IllegalArgumentException(name + ": power " + power + " is outside [-1, 1]");
        commandedPower = power;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior == null) throw new IllegalArgumentException(name + ": zero power behavior is null");
        commandedZeroPower = behavior;
    }

    /** change how often current is read; takes effect next I/O cycle */
    public void setCurrentReads(ReadPeriod period) {
        if (period == null) throw new IllegalArgumentException(name + ": current read period is null");
        currentReads = period;
    }

    /** newest power passed to {@link #setPower} */
    public double commandedPower() { return commandedPower; }

    /** power the I/O thread last sent to the hub */
    public double writtenPower() { return writtenPower; }

    /** @throws IllegalStateException if encoder reads are off ({@link Config#readEncoder()}) */
    public EncoderReading encoder() {
        EncoderReading reading = encoder;
        if (reading == null) throw new IllegalStateException(name + ": no encoder reading; " + (readsEncoder ? "start the I/O thread first" : "enable Config.readEncoder()"));
        return reading;
    }

    /** current draw in amps @throws IllegalStateException if current has not been read yet (see {@link #setCurrentReads}) */
    public TimedValue current() {
        TimedValue reading = current;
        if (reading == null) throw new IllegalStateException(name + ": no current reading; enable it with Config.readCurrent() or setCurrentReads()");
        return reading;
    }

    @Override
    void readIfDue(long nowNanos) {
        if (readsEncoder) {
            int position = motor.getCurrentPosition();
            double velocity = motor.getVelocity();
            encoder = new EncoderReading(position, velocity, System.nanoTime());
        }
        if (currentReads.isDue(hasReadCurrent, lastCurrentReadNanos, nowNanos)) {
            double amps = motor.getCurrent(CurrentUnit.AMPS);
            current = new TimedValue(amps, System.nanoTime());
            hasReadCurrent = true;
            lastCurrentReadNanos = nowNanos;
        }
    }

    @Override
    double pendingChange() {
        if (commandedZeroPower != ioWrittenZeroPower || Double.isNaN(ioWrittenPower)) return 1;
        return Math.abs(commandedPower - ioWrittenPower) / 2; // power spans [-1, 1]; halve so a full reversal scores 1
    }

    @Override
    void write() {
        DcMotor.ZeroPowerBehavior behavior = commandedZeroPower;
        double power = commandedPower;
        if (behavior != ioWrittenZeroPower) {
            motor.setZeroPowerBehavior(behavior);
            ioWrittenZeroPower = behavior;
        }
        if (power != ioWrittenPower) {
            motor.setPower(power);
            ioWrittenPower = power;
            writtenPower = power;
        }
    }
}

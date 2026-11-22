package org.firstinspires.ftc.teamcode.io;

import com.qualcomm.robotcore.hardware.Servo;

/** A position servo whose writes run on the I/O thread. Loop thread: {@link #setPosition} records the command. */
public final class ServoDevice extends IoDevice {
    private static final double INITIAL_WRITE_ESTIMATE_MILLIS = 2; // one Lynx command over the Control Hub UART

    private final Servo servo; // I/O thread only after start

    // loop thread -> I/O thread
    private volatile double commandedPosition;

    // I/O thread only
    private double ioWrittenPosition = Double.NaN; // NaN until the first write, so the first write always happens

    // I/O thread -> loop thread
    private volatile double writtenPosition = Double.NaN;

    /**
     * @param initialPosition written in {@link IoThread#start()}, so the servo moves during init
     * @throws IllegalArgumentException if initialPosition is outside [0, 1]
     */
    public ServoDevice(IoThread io, String name, double initialPosition, WritePolicy writePolicy) {
        super(io, name, writePolicy, INITIAL_WRITE_ESTIMATE_MILLIS);
        servo = io.hw().get(Servo.class, name);
        setPosition(initialPosition);
    }

    /**
     * @param position in [0, 1]
     * @throws IllegalArgumentException if position is NaN or outside [0, 1]
     */
    public void setPosition(double position) {
        if (Double.isNaN(position) || position < 0 || position > 1) throw new IllegalArgumentException(name + ": position " + position + " is outside [0, 1]");
        commandedPosition = position;
    }

    /** newest position passed to {@link #setPosition} */
    public double commandedPosition() { return commandedPosition; }

    /** position the I/O thread last sent to the hub; NaN before the first write */
    public double writtenPosition() { return writtenPosition; }

    @Override
    double pendingChange() {
        if (Double.isNaN(ioWrittenPosition)) return 1;
        return Math.abs(commandedPosition - ioWrittenPosition);
    }

    @Override
    void write() {
        double position = commandedPosition;
        servo.setPosition(position);
        ioWrittenPosition = position;
        writtenPosition = position;
    }
}

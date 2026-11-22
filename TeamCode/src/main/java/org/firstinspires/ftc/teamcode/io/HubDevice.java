package org.firstinspires.ftc.teamcode.io;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/** A Control or Expansion Hub's battery input voltage, read on the I/O thread. */
public final class HubDevice extends IoDevice {
    private final LynxModule hub; // I/O thread only after start
    private final ReadPeriod voltageReads;

    // I/O thread only
    private boolean hasRead = false;
    private long lastReadNanos = 0;

    // I/O thread -> loop thread
    private volatile TimedValue voltage = null;

    /**
     * @param name         hub name in the robot configuration, e.g. "Expansion Hub 2"
     * @param voltageReads every read is its own Lynx command (~2 ms)
     * @throws IllegalArgumentException if voltageReads is {@link ReadPeriod#never()}
     */
    public HubDevice(IoThread io, String name, ReadPeriod voltageReads) {
        super(io, name);
        if (!voltageReads.isEnabled()) throw new IllegalArgumentException(name + ": a hub device that never reads does nothing");
        hub = io.hw().get(LynxModule.class, name);
        this.voltageReads = voltageReads;
    }

    /** input voltage in volts @throws IllegalStateException before the I/O thread's first cycle */
    public TimedValue inputVoltage() {
        TimedValue reading = voltage;
        if (reading == null) throw new IllegalStateException(name + ": no voltage reading; start the I/O thread first");
        return reading;
    }

    @Override
    void readIfDue(long nowNanos) {
        if (!voltageReads.isDue(hasRead, lastReadNanos, nowNanos)) return;
        double volts = hub.getInputVoltage(VoltageUnit.VOLTS);
        voltage = new TimedValue(volts, System.nanoTime());
        hasRead = true;
        lastReadNanos = nowNanos;
    }
}

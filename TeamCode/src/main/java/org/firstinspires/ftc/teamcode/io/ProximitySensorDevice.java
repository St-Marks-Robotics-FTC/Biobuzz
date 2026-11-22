package org.firstinspires.ftc.teamcode.io;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * A Pololu Digital Distance Sensor, 5cm (item 4050) on a hub digital port, read on the I/O thread.
 *
 * <p>The sensor has one open-drain-style output that drives LOW while an object is within ~5 cm and HIGH otherwise.
 * Digital inputs come from the Lynx bulk read, so reading every cycle adds no extra hub command.
 */
public final class ProximitySensorDevice extends IoDevice {
    /** One reading: whether an object was in range when read. Immutable. */
    public static final class Detection {
        private final boolean objectDetected;
        private final long nanoTime;

        Detection(boolean objectDetected, long nanoTime) {
            this.objectDetected = objectDetected;
            this.nanoTime = nanoTime;
        }

        /** true if an object was within range */
        public boolean objectDetected() { return objectDetected; }

        /** {@link System#nanoTime()} right after the read */
        public long nanoTime() { return nanoTime; }

        /** milliseconds since the read */
        public double ageMillis() { return (System.nanoTime() - nanoTime) / 1e6; }
    }

    private final DigitalChannel pin; // I/O thread only after start
    private final ReadPeriod reads;

    // I/O thread only
    private boolean hasRead = false;
    private long lastReadNanos = 0;

    // I/O thread -> loop thread
    private volatile Detection detection = null;

    /**
     * @param name  hardware map name of the digital port the sensor's OUT wire is on
     * @param reads how often to read; must be enabled
     * @throws IllegalArgumentException if reads are off
     */
    public ProximitySensorDevice(IoThread io, String name, ReadPeriod reads) {
        super(io, name);
        if (!reads.isEnabled()) throw new IllegalArgumentException(name + ": a proximity sensor that never reads does nothing");
        pin = io.hw().get(DigitalChannel.class, name);
        pin.setMode(DigitalChannel.Mode.INPUT);
        this.reads = reads;
    }

    /** the newest reading @throws IllegalStateException if the I/O thread has not started */
    public Detection detection() {
        Detection reading = detection;
        if (reading == null) throw new IllegalStateException(name + ": no reading; start the I/O thread first");
        return reading;
    }

    @Override
    void readIfDue(long nowNanos) {
        if (!reads.isDue(hasRead, lastReadNanos, nowNanos)) return;
        boolean outputLow = !pin.getState(); // output drives low on detection
        detection = new Detection(outputLow, System.nanoTime());
        hasRead = true;
        lastReadNanos = nowNanos;
    }
}

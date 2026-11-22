package org.firstinspires.ftc.teamcode.io;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A REV Color Sensor V3 read on the I/O thread. Color and proximity are separate I2C transactions (each several
 * Lynx commands), so each has its own {@link ReadPeriod}.
 */
public final class ColorSensorDevice extends IoDevice {
    /** One-time setup, applied in the constructor before the I/O thread starts. */
    public static final class Config {
        private float gain = Float.NaN; // NaN keeps the sensor's default gain
        private ReadPeriod colorReads = ReadPeriod.everyCycle();
        private ReadPeriod distanceReads = ReadPeriod.never();

        /** multiplies normalized color values; raise it if readings are dim */
        public Config gain(float gain) {
            if (!(gain > 0)) throw new IllegalArgumentException("color sensor gain must be positive, got " + gain);
            this.gain = gain;
            return this;
        }

        public Config colorReads(ReadPeriod period) { this.colorReads = period; return this; }

        public Config distanceReads(ReadPeriod period) { this.distanceReads = period; return this; }
    }

    /** Normalized color channels from one read, each roughly 0-1 after gain. Immutable. */
    public static final class ColorReading {
        private final float red, green, blue, alpha;
        private final long nanoTime;

        ColorReading(NormalizedRGBA rgba, long nanoTime) {
            this.red = rgba.red;
            this.green = rgba.green;
            this.blue = rgba.blue;
            this.alpha = rgba.alpha;
            this.nanoTime = nanoTime;
        }

        public float red() { return red; }
        public float green() { return green; }
        public float blue() { return blue; }
        public float alpha() { return alpha; }

        /** {@link System#nanoTime()} right after the read */
        public long nanoTime() { return nanoTime; }
    }

    private final RevColorSensorV3 sensor; // I/O thread only after start
    private final ReadPeriod colorReads;
    private final ReadPeriod distanceReads;

    // I/O thread only
    private boolean hasReadColor = false, hasReadDistance = false;
    private long lastColorNanos = 0, lastDistanceNanos = 0;

    // I/O thread -> loop thread
    private volatile ColorReading color = null;
    private volatile TimedValue distanceMm = null;

    /** @throws IllegalArgumentException if both color and distance reads are off */
    public ColorSensorDevice(IoThread io, String name, Config config) {
        super(io, name);
        if (!config.colorReads.isEnabled() && !config.distanceReads.isEnabled()) throw new IllegalArgumentException(name + ": a color sensor that never reads does nothing");
        sensor = io.hw().get(RevColorSensorV3.class, name);
        if (!Float.isNaN(config.gain)) sensor.setGain(config.gain);
        colorReads = config.colorReads;
        distanceReads = config.distanceReads;
    }

    /** @throws IllegalStateException if color reads are off */
    public ColorReading color() {
        ColorReading reading = color;
        if (reading == null) throw new IllegalStateException(name + ": no color reading; " + (colorReads.isEnabled() ? "start the I/O thread first" : "enable Config.colorReads()"));
        return reading;
    }

    /** proximity in millimeters @throws IllegalStateException if distance reads are off */
    public TimedValue distanceMm() {
        TimedValue reading = distanceMm;
        if (reading == null) throw new IllegalStateException(name + ": no distance reading; " + (distanceReads.isEnabled() ? "start the I/O thread first" : "enable Config.distanceReads()"));
        return reading;
    }

    @Override
    void readIfDue(long nowNanos) {
        if (colorReads.isDue(hasReadColor, lastColorNanos, nowNanos)) {
            NormalizedRGBA rgba = sensor.getNormalizedColors();
            color = new ColorReading(rgba, System.nanoTime());
            hasReadColor = true;
            lastColorNanos = nowNanos;
        }
        if (distanceReads.isDue(hasReadDistance, lastDistanceNanos, nowNanos)) {
            double mm = sensor.getDistance(DistanceUnit.MM);
            distanceMm = new TimedValue(mm, System.nanoTime());
            hasReadDistance = true;
            lastDistanceNanos = nowNanos;
        }
    }
}

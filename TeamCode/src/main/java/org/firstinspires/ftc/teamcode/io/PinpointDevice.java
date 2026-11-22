package org.firstinspires.ftc.teamcode.io;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

/**
 * Pedro localizer whose goBILDA Pinpoint reads and pose writes run on the I/O thread.
 *
 * <p>The I/O thread drives Pedro's own {@link PinpointLocalizer} (so pod offsets, directions and units come from the
 * same {@link PinpointConfig}) and publishes a {@link Sample} every cycle. On the loop thread, {@link #update()}
 * latches the newest sample, so one follower update sees one consistent pose.
 */
public final class PinpointDevice extends IoDevice implements Localizer {
    private static final double INITIAL_WRITE_ESTIMATE_MILLIS = 3; // one I2C register write

    /** One Pinpoint read, tagged with the newest pose request it already includes. Immutable. */
    public static final class Sample {
        private final MotionState state;
        private final long nanoTime;
        private final long appliedPoseRequest;

        Sample(MotionState state, long nanoTime, long appliedPoseRequest) {
            this.state = state;
            this.nanoTime = nanoTime;
            this.appliedPoseRequest = appliedPoseRequest;
        }

        public MotionState state() { return state; }

        /** {@link System#nanoTime()} right after the read */
        public long nanoTime() { return nanoTime; }

        /** milliseconds since the read */
        public double ageMillis() { return (System.nanoTime() - nanoTime) / 1e6; }
    }

    /** a {@link #setPose} call waiting for the I/O thread; numbered so readings can tell whether they include it */
    private static final class PoseRequest {
        final Pose pose;
        final long number;

        PoseRequest(Pose pose, long number) {
            this.pose = pose;
            this.number = number;
        }
    }

    private final PinpointLocalizer pinpoint; // I/O thread only after start

    // loop thread -> I/O thread
    private volatile PoseRequest poseRequest = null;

    // I/O thread only
    private long ioAppliedPoseRequest = 0;

    // I/O thread -> loop thread
    private volatile Sample sample;

    // loop thread only
    private MotionState state;
    private long requestedPoseNumber = 0;

    /**
     * Builds Pedro's Pinpoint localizer, which configures the Pinpoint, recalibrates its IMU and sleeps 500 ms.
     * The robot must be still.
     */
    public PinpointDevice(IoThread io, PinpointConfig config) {
        super(io, config.name.get(), WritePolicy.IMMEDIATE, INITIAL_WRITE_ESTIMATE_MILLIS);
        pinpoint = new PinpointLocalizer(io.hw(), config); // reads once, so a state exists before the thread starts
        sample = new Sample(pinpoint.state(), System.nanoTime(), 0);
        state = sample.state;
    }

    /** newest published Pinpoint read (age telemetry); the follower uses {@link #state()} */
    public Sample latestSample() { return sample; }

    /** latches the newest Pinpoint read; until the I/O thread applies the latest {@link #setPose}, keeps the requested pose */
    @Override
    public void update() {
        Sample newest = sample;
        if (newest.appliedPoseRequest >= requestedPoseNumber) state = newest.state;
    }

    @Override
    public MotionState state() { return state; }

    /** queues a pose overwrite for the I/O thread; {@link #state()} reports the new pose immediately */
    @Override
    public void setPose(Pose pose) {
        if (pose == null) throw new IllegalArgumentException("pose is null");
        requestedPoseNumber++;
        poseRequest = new PoseRequest(pose, requestedPoseNumber);
        state = state.withPose(pose);
    }

    /**
     * Not supported: an IMU recalibration blocks the hardware link for 500 ms. Construct a new device during init
     * instead. Pedro's Follower never calls this.
     */
    @Override
    public void reset() {
        throw new UnsupportedOperationException("PinpointDevice cannot reset while running; it recalibrates when constructed");
    }

    @Override
    void readIfDue(long nowNanos) {
        long applied = ioAppliedPoseRequest; // read before the hardware so the tag never claims more than the read includes
        pinpoint.update();
        sample = new Sample(pinpoint.state(), System.nanoTime(), applied);
    }

    @Override
    double pendingChange() {
        PoseRequest request = poseRequest;
        return request != null && request.number > ioAppliedPoseRequest ? 1 : 0;
    }

    @Override
    void write() {
        PoseRequest request = poseRequest;
        pinpoint.setPose(request.pose);
        ioAppliedPoseRequest = request.number;
    }
}

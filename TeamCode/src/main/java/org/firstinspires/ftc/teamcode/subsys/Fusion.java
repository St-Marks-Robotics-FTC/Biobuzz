/**
 * Fuses odometry (fast, smooth, but slowly drifts) with Limelight vision (no drift, but noisy and delayed).
 *
 * This is a Kalman filter: one filter for x and one for y (see KalmanFilter.java).
 * The state is the robot's real field position in inches.
 *
 *   PREDICT (every loop, using TeleOp odometry):
 *     x = x + odometryDelta
 *     P = P + Q grows with distance driven
 *
 *   UPDATE (new vision frame):
 *     K = P / (P + R)    R is the vision uncertainty for this frame
 *     x = x + K * (vision - x)
 *     P = (1 - K) * P
 *
 * Heading always comes straight from the Pinpoint IMU; only x/y are filtered.
 *
 * Vision is delayed, so the frame is first moved forward by the odometry displacement
 * that happened between the picture and now. This lets the filter update the current pose
 * without keeping a full delayed-state history.
 *
 * USAGE: Fusion doesn't own the follower or the vision; every loop, the OpMode updates both
 * and hands their data in:
 *   follower.update();
 *   vision.update(follower.pose().heading());
 *   fusion.update(follower.pose(), follower.velocity(), vision.getResult());
 *   Pose fused = fusion.getPose();
 */

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.Tunables;

import java.util.ArrayDeque;

public class Fusion {
    private static final long HISTORY_MS = 1500; // how much odometry history to keep; must be longer than the vision delay
    private static final long STARTUP_HISTORY_SLACK_MS = 250; // allow the first frame to be slightly older than our history

    // one odometry reading and when it was taken
    private static class OdoSample {
        final long timeMs;
        final double x, y;
        final double vx, vy, omega;

        OdoSample(long timeMs, double x, double y, double vx, double vy, double omega) {
            this.timeMs = timeMs;
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }

    private final ArrayDeque<OdoSample> history = new ArrayDeque<>(); // oldest first, newest last

    /** filter state **/

    private Pose odoPose; // newest odometry pose we were given
    private Pose lastOdoPose; // odometry pose at the previous update (for the predict step)
    private KalmanFilter filterX; // real field x position, inches
    private KalmanFilter filterY; // real field y position, inches
    private long lastAcceptedMs = -1;
    private int rejectedInARow = 0; // vision frames in a row that disagreed too much with us
    private int odoRejectsInARow = 0; // odometry frames in a row that teleported too far
    private long lastFusedTime = Long.MIN_VALUE; // acquisition time of the last picture we fused

    /** debug info **/

    private double lastVisionStdX, lastVisionStdY;
    private int acceptedCount = 0;
    private int rejectedCount = 0;
    private int odoRejectedCount = 0;
    private String lastResult = "no vision yet";

    // startPose = the pose the follower starts at
    public Fusion(Pose startPose) {
        reset(startPose);
    }

    // set to pose that is known to be correct
    public void reset(Pose pose) {
        odoPose = pose;
        lastOdoPose = pose;
        history.clear();
        lastAcceptedMs = -1;

        double startVariance = Tunables.fusionStartStd * Tunables.fusionStartStd;

        filterX = new KalmanFilter(pose.x(), startVariance);
        filterY = new KalmanFilter(pose.y(), startVariance);

        rejectedInARow = 0;
        odoRejectsInARow = 0;
        lastFusedTime = Long.MIN_VALUE;
        lastVisionStdX = 0;
        lastVisionStdY = 0;
        acceptedCount = 0;
        rejectedCount = 0;
        odoRejectedCount = 0;
        lastResult = "no vision yet";
    }

    // call once per loop, AFTER follower.update() and vision.update()
    // visionResult may be null (no valid frame this loop)
    public void update(Pose odo, Velocity velocity, Vision.VisionResult visionResult) {
        if (odo == null) return;

        long now = System.currentTimeMillis();
        boolean firstUpdate = lastAcceptedMs < 0;
        double dx = 0;
        double dy = 0;

        if (!firstUpdate) {
            dx = odo.x() - lastOdoPose.x();
            dy = odo.y() - lastOdoPose.y();
            double distance = Math.hypot(dx, dy);
            double seconds = Math.max(0, now - lastAcceptedMs) / 1000.0;
            double maxTravel = Tunables.fusionOdoMaxSpeed * seconds + Tunables.fusionOdoJumpMargin;

            if (distance > maxTravel) {  //Prevent teleportation pedro affecting the filter
                odoRejectsInARow++;
                odoRejectedCount++;
                lastResult = "odometry dropped: teleport";

                if (odoRejectsInARow < Tunables.fusionOdoRejectsBeforeReset) return;
                odoRejectsInARow = 0;
                history.clear();
            } else {
                odoRejectsInARow = 0;
            }
        }

        odoPose = odo;

        if (firstUpdate) {
            filterX.setState(odo.x());
            filterY.setState(odo.y());
        } else {
            double distance = Math.hypot(dx, dy);
            double processNoise = Tunables.fusionOdoVariancePerInch * distance;

            filterX.predict(dx, processNoise);
            filterY.predict(dy, processNoise);
        }
        lastOdoPose = odo;
        lastAcceptedMs = now;

        // If it is too old, then throw it away
        recordOdometry(now, odo, velocity);
        while (!history.isEmpty() && history.peekFirst().timeMs < now - HISTORY_MS) history.removeFirst();

        // if there's a vision result we haven't fused yet, use it to correct the pose.
        // Don't send the same picture twice.
        if (visionResult != null && visionResult.acquisitionTime() != lastFusedTime) {
            lastFusedTime = visionResult.acquisitionTime();
            useVision(visionResult);
        }
    }

    private void recordOdometry(long now, Pose odo, Velocity velocity) {
        double vx = 0, vy = 0, omega = 0;
        if (velocity != null) {
            vx = velocity.vx;
            vy = velocity.vy;
            omega = velocity.omega;
        }

        OdoSample newest = history.peekLast();
        if (newest != null && newest.timeMs == now) {
            history.removeLast(); // keep one sample per millisecond so interpolation never divides by zero
        }
        history.addLast(new OdoSample(now, odo.x(), odo.y(), vx, vy, omega));
    }

    private void useVision(Vision.VisionResult result) {
        // where did odometry think we were when the picture was taken?
        long captureTimeMs = Math.min(lastAcceptedMs,
                result.acquisitionTime() - Math.round(Tunables.fusionExtraLatencyMs));
        OdoSample odoThen = odometryAt(captureTimeMs);
        if (odoThen == null) {
            reject("picture older than odometry history");
            return;
        }

        // Move delayed vision to the current odo capture
        double visionEstimateX = result.pose().x() + (odoPose.x() - odoThen.x);
        double visionEstimateY = result.pose().y() + (odoPose.y() - odoThen.y);

        measureVisionStd(result, odoThen); // sets lastVisionStdX and lastVisionStdY
        double visionVarianceX = lastVisionStdX * lastVisionStdX;
        double visionVarianceY = lastVisionStdY * lastVisionStdY;

        // if the two estimates disagree way more than their uncertainties explain, the frame is probably bad
        boolean tooFarX = filterX.isOutlier(visionEstimateX, visionVarianceX, Tunables.fusionGateSigmas);
        boolean tooFarY = filterY.isOutlier(visionEstimateY, visionVarianceY, Tunables.fusionGateSigmas);
        if (tooFarX || tooFarY) {
            rejectedInARow++;
            reject("too far from current estimate");

            // if vision keeps disagreeing, odometry is probably the one that's wrong (ex. we got hit),
            // so admit we're less sure. This lets vision get back in after a collision.
            if (rejectedInARow >= Tunables.fusionRejectsBeforeDoubt) {
                filterX.inflateVariance(4);
                filterY.inflateVariance(4);
                rejectedInARow = 0;
            }
            return;
        }
        rejectedInARow = 0;
        filterX.update(visionEstimateX, visionVarianceX);
        filterY.update(visionEstimateY, visionVarianceY);

        acceptedCount++;
        lastResult = "accepted";
    }

    // how uncertain is this vision frame?
    private void measureVisionStd(Vision.VisionResult result, OdoSample odoThen) {
        // tags: error grows with distance squared (far tags are tiny in the image), and shrinks with more tags
        double tagCount = Math.max(1, result.tagCount());
        double avgDistance = isFinite(result.avgTagDistance()) ? Math.max(0, result.avgTagDistance()) : 0;
        double distanceRatio = avgDistance / Math.max(1e-6, Tunables.fusionVisionRefDistance);
        double tagStd = Tunables.fusionVisionBaseStd * distanceRatio * distanceRatio / Math.sqrt(tagCount);

        // Limelight's own std dev: take whichever of the two is more pessimistic
        double limelightStdX = isFinite(result.stdDevX()) ? Math.abs(result.stdDevX()) : 0;
        double limelightStdY = isFinite(result.stdDevY()) ? Math.abs(result.stdDevY()) : 0;
        double stdX = Math.max(tagStd, limelightStdX);
        double stdY = Math.max(tagStd, limelightStdY);

        // turning blurs the image.
        // Use the velocity from the moment the picture was taken, not our current velocity.
        double speedStd = Math.hypot(odoThen.vx, odoThen.vy) * Tunables.fusionTimingErrorMs / 1000.0;
        double turnStd = Math.abs(odoThen.omega) * Tunables.fusionTurnStd;
        double motionStd = Math.hypot(speedStd, turnStd);

        // independent errors add like hypotenuse
        stdX = Math.hypot(stdX, motionStd);
        stdY = Math.hypot(stdY, motionStd);

        // never let a frame claim to be perfect
        lastVisionStdX = Math.max(stdX, Tunables.fusionMinVisionStd);
        lastVisionStdY = Math.max(stdY, Tunables.fusionMinVisionStd);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private void reject(String reason) {
        rejectedCount++;
        lastResult = "rejected: " + reason;
    }

    // odometry position at a past time, linearly interpolated between the two samples around it
    // returns null if that time is older than our history
    private OdoSample odometryAt(long timeMs) {
        OdoSample before = null;
        for (OdoSample after : history) { // oldest -> newest
            if (after.timeMs >= timeMs) {
                if (before == null) {
                    // Frame is from just before we started keeping history. At startup this is
                    // normal, so use the oldest sample instead of rejecting for a whole second.
                    if (after.timeMs - timeMs <= STARTUP_HISTORY_SLACK_MS) return after;
                    return null;
                }

                double t = (double) (timeMs - before.timeMs) / (after.timeMs - before.timeMs); // 0 = before, 1 = after
                return new OdoSample(timeMs,
                        before.x + t * (after.x - before.x),
                        before.y + t * (after.y - before.y),
                        before.vx + t * (after.vx - before.vx),
                        before.vy + t * (after.vy - before.vy),
                        before.omega + t * (after.omega - before.omega));
            }
            before = after;
        }
        return before; // picture is newer than our newest sample, so the newest sample is the closest we have
    }

    // our best guess of where the robot actually is
    public Pose getPose() {
        double heading = odoPose != null ? odoPose.heading() : 0;
        return new Pose(filterX.getState(), filterY.getState(), heading);
    }

    public double getOffsetX() { return odoPose != null ? filterX.getState() - odoPose.x() : 0; } // how far odometry has drifted
    public double getOffsetY() { return odoPose != null ? filterY.getState() - odoPose.y() : 0; }
    public double getStdX() { return filterX.getStdDev(); } // uncertainty of the fused pose, in inches
    public double getStdY() { return filterY.getStdDev(); } // uncertainty of the fused pose, in inches
    public double getLastVisionStdX() { return lastVisionStdX; } // uncertainty of the last vision frame, in inches
    public double getLastVisionStdY() { return lastVisionStdY; } // uncertainty of the last vision frame, in inches
    public int getAcceptedCount() { return acceptedCount; }
    public int getRejectedCount() { return rejectedCount; }
    public int getOdoRejectedCount() { return odoRejectedCount; } // odometry samples dropped as teleports
    public String getLastResult() { return lastResult; }
}

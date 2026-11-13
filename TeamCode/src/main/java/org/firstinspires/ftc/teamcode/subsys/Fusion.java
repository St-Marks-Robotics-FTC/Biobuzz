/**
 * Fuses odometry (fast, smooth, but slowly drifts) with Limelight vision (no drift, but noisy and delayed).
 *
 * IDEA:
 * - we never touch the odometry itself; instead we learn an `offset` = how far odometry has drifted
 * - fused pose = odometry pose + offset
 * - heading always comes straight from the Pinpoint IMU, which is already very good
 *
 * EVERY LOOP (odometry):
 * - odometry drifts the further we drive, so our uncertainty (variance) grows with distance driven
 *
 * EVERY NEW VISION FRAME:
 * 1. look up where odometry thought we were when the picture was taken (vision is delayed)
 * 2. measure how uncertain this frame is (tag count, tag distance, Limelight std dev, robot motion)
 * 3. throw it out if it's clearly wrong
 * 4. combine the two estimates with a weighted average, weight = 1 / variance (this is a Kalman filter update)
 *
 * USAGE: Fusion doesn't own the follower or the vision; every loop, the OpMode updates both and hands their data in:
 *   follower.update();
 *   vision.update();
 *   fusion.update(follower.pose(), follower.velocity(), vision.getResult());
 *   Pose fused = fusion.getPose();
 */

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.Tunables;

import java.util.ArrayDeque;

public class Fusion {
    private static final long HISTORY_MS = 1000; // how much odometry history to keep; must be longer than the vision delay

    // one odometry reading and when it was taken
    private static class OdoSample {
        final long timeMs;
        final double x, y;

        OdoSample(long timeMs, double x, double y) {
            this.timeMs = timeMs;
            this.x = x;
            this.y = y;
        }
    }

    private final ArrayDeque<OdoSample> history = new ArrayDeque<>(); // oldest first, newest last

    /** filter state **/

    private Pose odoPose; // newest odometry pose we were given
    private Velocity odoVelocity; // newest odometry velocity we were given
    private double offsetX, offsetY; // how far odometry has drifted in inches (true = odometry + offset)
    private double varianceX, varianceY; // how unsure we are about our estimate, in inches squared
    private int rejectedInARow = 0; // vision frames in a row that disagreed too much with us
    private long lastFusedTime = -1; // acquisition time of the last picture we fused

    /** debug info **/

    private double lastVisionStdX, lastVisionStdY;
    private int acceptedCount = 0;
    private int rejectedCount = 0;
    private String lastResult = "no vision yet";

    // startPose = the pose the follower starts at
    public Fusion(Pose startPose) {
        reset(startPose);
    }

    // forget all corrections, use this after setting the follower to a pose you know is correct
    public void reset(Pose pose) {
        odoPose = pose;
        odoVelocity = null;
        history.clear();
        lastFusedTime = -1;
        offsetX = 0;
        offsetY = 0;
        varianceX = Tunables.fusionStartStd * Tunables.fusionStartStd;
        varianceY = varianceX;
        rejectedInARow = 0;
    }

    // call once per loop, AFTER follower.update() and vision.update()
    // visionResult may be null (no valid frame this loop)
    public void update(Pose odo, Velocity velocity, Vision.VisionResult visionResult) {
        odoPose = odo;
        odoVelocity = velocity;
        long now = System.currentTimeMillis();

        // 1. odometry drifts more the further we drive, so grow our uncertainty by distance driven
        OdoSample newest = history.peekLast();
        if (newest != null) {
            double distance = Math.hypot(odo.x() - newest.x, odo.y() - newest.y);
            varianceX += Tunables.fusionOdoVariancePerInch * distance;
            varianceY += Tunables.fusionOdoVariancePerInch * distance;
        }

        // 2. remember where odometry is right now, and throw away history that's too old to be useful
        history.addLast(new OdoSample(now, odo.x(), odo.y()));
        while (history.peekFirst().timeMs < now - HISTORY_MS) history.removeFirst();

        // 3. if there's a vision result we haven't fused yet, use it to correct the offset
        // Don't send the same picture twice
        if (visionResult != null && visionResult.acquisitionTime() != lastFusedTime) {
            lastFusedTime = visionResult.acquisitionTime();
            useVision(visionResult);
        }
    }

    private void useVision(Vision.VisionResult result) {
        // STEP 1: where did odometry think we were when the picture was taken?
        long captureTimeMs = result.acquisitionTime() - Math.round(Tunables.fusionExtraLatencyMs);
        OdoSample odoThen = odometryAt(captureTimeMs);
        if (odoThen == null) {
            reject("picture older than odometry history");
            return;
        }

        // STEP 2: we now have two estimates of where the robot was at that moment, each with an uncertainty
        double odoEstimateX = odoThen.x + offsetX; // uncertainty: varianceX
        double odoEstimateY = odoThen.y + offsetY; // uncertainty: varianceY

        double visionEstimateX = result.pose().x();
        double visionEstimateY = result.pose().y();
        measureVisionStd(result); // sets lastVisionStdX and lastVisionStdY
        double visionVarianceX = lastVisionStdX * lastVisionStdX;
        double visionVarianceY = lastVisionStdY * lastVisionStdY;

        // STEP 3: if the two estimates disagree way more than their uncertainties explain, the frame is probably bad
        boolean tooFarX = Math.abs(visionEstimateX - odoEstimateX) > Tunables.fusionGateSigmas * Math.sqrt(varianceX + visionVarianceX);
        boolean tooFarY = Math.abs(visionEstimateY - odoEstimateY) > Tunables.fusionGateSigmas * Math.sqrt(varianceY + visionVarianceY);
        if (tooFarX || tooFarY) {
            reject("too far from current estimate");
            rejectedInARow++;

            // if vision keeps disagreeing, odometry is probably the one that's wrong (ex. we got hit),
            // so admit we're less sure, allows vision to get back in
            if (rejectedInARow >= Tunables.fusionRejectsBeforeDoubt) {
                varianceX *= 4;
                varianceY *= 4;
                rejectedInARow = 0;
            }
            return;
        }
        rejectedInARow = 0;

        // STEP 4: combine the two estimates, trusting each one according to how certain it is
        double fusedX = weightedAverage(odoEstimateX, varianceX, visionEstimateX, visionVarianceX);
        double fusedY = weightedAverage(odoEstimateY, varianceY, visionEstimateY, visionVarianceY);

        // after combining, we're more certain than either estimate was alone
        varianceX = combinedVariance(varianceX, visionVarianceX);
        varianceY = combinedVariance(varianceY, visionVarianceY);

        // shift the offset so that odometry (at that moment) + offset lands on the fused estimate
        offsetX += fusedX - odoEstimateX;
        offsetY += fusedY - odoEstimateY;

        acceptedCount++;
        lastResult = "accepted";
    }

    // how uncertain is this vision frame? (std dev in inches, for x and y)
    private void measureVisionStd(Vision.VisionResult result) {
        // tags: error grows with distance squared (far tags are tiny in the image), and shrinks with more tags
        // (each tag is another independent measurement, so error shrinks by sqrt(number of tags))
        double distanceRatio = result.avgTagDistance() / Tunables.fusionVisionRefDistance;
        double tagStd = Tunables.fusionVisionBaseStd * distanceRatio * distanceRatio / Math.sqrt(result.tagCount());

        // Limelight's own std dev: take whichever of the two is more pessimistic
        double stdX = Math.max(tagStd, result.stdDev()[0]);
        double stdY = Math.max(tagStd, result.stdDev()[1]);

        // motion: if our timing is a little off, driving turns that into position error, and turning blurs the image
        double motionStd = 0;
        if (odoVelocity != null) {
            double speedStd = Math.hypot(odoVelocity.vx, odoVelocity.vy) * Tunables.fusionTimingErrorMs / 1000.0;
            double turnStd = Math.abs(odoVelocity.omega) * Tunables.fusionTurnStd;
            motionStd = Math.hypot(speedStd, turnStd);
        }

        // independent errors add like the sides of a right triangle: total = sqrt(a^2 + b^2)
        stdX = Math.hypot(stdX, motionStd);
        stdY = Math.hypot(stdY, motionStd);

        // never let a frame claim to be perfect
        lastVisionStdX = Math.max(stdX, Tunables.fusionMinVisionStd);
        lastVisionStdY = Math.max(stdY, Tunables.fusionMinVisionStd);
    }

    // average of two estimates, where each one's weight is 1 / its variance (more certain = more weight)
    private static double weightedAverage(double a, double varianceA, double b, double varianceB) {
        double weightA = 1 / varianceA;
        double weightB = 1 / varianceB;
        return (a * weightA + b * weightB) / (weightA + weightB);
    }

    // variance of the weighted average above; always smaller than both inputs
    private static double combinedVariance(double varianceA, double varianceB) {
        return 1 / (1 / varianceA + 1 / varianceB);
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
                if (before == null) return null; // older than everything we have

                double t = (double) (timeMs - before.timeMs) / (after.timeMs - before.timeMs); // 0 = before, 1 = after
                return new OdoSample(timeMs,
                        before.x + t * (after.x - before.x),
                        before.y + t * (after.y - before.y));
            }
            before = after;
        }
        return before; // picture is newer than our newest sample, so the newest sample is the closest we have
    }

    // our best guess of where the robot actually is
    public Pose getPose() {
        return new Pose(odoPose.x() + offsetX, odoPose.y() + offsetY, odoPose.heading());
    }

    public double getOffsetX() { return offsetX; }
    public double getOffsetY() { return offsetY; }
    public double getStdX() { return Math.sqrt(varianceX); } // uncertainty of the fused pose, in inches
    public double getStdY() { return Math.sqrt(varianceY); } // uncertainty of the fused pose, in inches
    public double getLastVisionStdX() { return lastVisionStdX; } // uncertainty of the last vision frame, in inches
    public double getLastVisionStdY() { return lastVisionStdY; } // uncertainty of the last vision frame, in inches
    public int getAcceptedCount() { return acceptedCount; }
    public int getRejectedCount() { return rejectedCount; }
    public String getLastResult() { return lastResult; }
}

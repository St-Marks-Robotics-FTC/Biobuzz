/**
 * RESOURCES
 * red tag IDs are 30-37
 * blue tag IDs are 38-45
 * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
 * https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
 *
 */

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Tunables;

import java.util.concurrent.TimeUnit;

public class Vision {
    private static final double METERS_TO_INCHES = 39.37;
    private static final double HIVE_MAX_TILT = Math.toRadians(35); // max tilt of hive in radians

    public static Limelight3A limelight;

    public enum Pipeline { // expression of our limelight pipelines; order and elements must match exactly with pipeline indices on camera
        BLUE, // pipeline index: 0
        RED   // pipeline index: 1
    }

    public enum HiveState {
        UNKNOWN,
        AUDIENCE_UP,
        SCORING_UP
    }

    /**
     * One accepted Limelight frame. Fusion consumes this instead of the raw LLResult so that
     * timestamps and uncertainties are already in Pedro units (inches / milliseconds).
     */
    public static class VisionResult {
        private final Pose pose;
        private final long acquisitionTimeMs;
        private final int tagCount;
        private final double avgTagDistance;
        private final double stdDevX;
        private final double stdDevY;
        private final double heading;

        public VisionResult(Pose pose, long acquisitionTimeMs, int tagCount, double avgTagDistance,
                            double stdDevX, double stdDevY, double heading) {
            this.pose = pose;
            this.acquisitionTimeMs = acquisitionTimeMs;
            this.tagCount = tagCount;
            this.avgTagDistance = avgTagDistance;
            this.stdDevX = stdDevX;
            this.stdDevY = stdDevY;
            this.heading = heading;
        }

        public Pose pose() { return pose; }
        public long acquisitionTime() { return acquisitionTimeMs; }
        public int tagCount() { return tagCount; }
        public double avgTagDistance() { return avgTagDistance; }
        public double stdDevX() { return stdDevX; }
        public double stdDevY() { return stdDevY; }
        public double[] stdDev() { return new double[]{stdDevX, stdDevY}; }
        public double heading() { return heading; }
    }

    private boolean started = false;
    private final boolean isBlueTeam;
    private Pose lastBotPose = new Pose(0, 0);
    private boolean hasBotPose = false; // lastBotPose is a real reading (enables the jump check)
    private HiveState lastHiveState = HiveState.UNKNOWN;
    private VisionResult lastResult;
    private int jumpRejects = 0;

    private final Timer staleTimer; // how stale lastBotPose is

    private double lastFrameTimestamp = Double.NaN;
    private long lastFrameHubTime = Long.MIN_VALUE;

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
        if (isBlueTeam) startPipeline(Pipeline.BLUE);
        else startPipeline(Pipeline.RED);
        staleTimer = new Timer();
    }

    public void startPipeline(Pipeline pipeline) {
        if (!started) {
            limelight.start();
            started = true;
        }

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    public LLStatus getStatus() { return limelight.getStatus(); }

    public Pipeline getPipeline() { // convert from limelight pipeline ordinal/index to Pipeline enum
        int index = getStatus().getPipelineIndex();
        Pipeline[] pipelines = Pipeline.values();
        return index >= 0 && index < pipelines.length ? pipelines[index] : Pipeline.BLUE;
    }

    // return staleness of the last accepted botpose in milliseconds
    public double getStaleness() { return staleTimer.get(TimeUnit.MILLISECONDS); }

    /**
     * Processes the latest Limelight frame.
     *
     * @param heading current robot heading in radians (used to rotate the camera offset)
     * @return the new accepted frame, or null if the frame was a duplicate or was rejected
     */
    public VisionResult update(double heading) {
        /**
         *         pivot ●
         *               |\
         *               | \
         *             h |  \  dToHive  (distance from pivot to robot)
         *               |   \
         *               |    \
         *         floor ●-----● robot
         *               horizontal
         */
        LLResult result = limelight.getLatestResult();

        if (!isReasonable(result)) return null;

        // Only process each picture once. Mark it seen even if it is later rejected, otherwise
        // a bad frame would be counted as a new rejection every loop.
        if (!isNewFrame(result)) return null;

        if (result.getBotposeTagCount() < Tunables.minTagCount) return null;

        Pose3D botpose3D = result.getBotpose();
        if (botpose3D == null) return null;

        Position botpose = botpose3D.getPosition().toUnit(DistanceUnit.INCH);
        boolean isRed = getPipeline() == Pipeline.RED;
        double x = botpose.y + 72; // convert to pedro units
        double y = 72 - botpose.x; // convert to pedro units
        double avgDist = result.getBotposeAvgDist() * METERS_TO_INCHES; // m -> in
        double z = botpose.z - Tunables.visionZOffset - Tunables.visionZPerDist * avgDist;

        double pivotHeight = Tunables.visionHivePivotHeight;
        double pivotY = Tunables.visionHivePivotY;

        double relativeY = y - pivotY;
        double relativeZ = z - pivotHeight;

        double dToHive = Math.sqrt(Math.pow(relativeY, 2) + Math.pow(relativeZ, 2));

        if (dToHive < pivotHeight) return null; // impossible state -> reject data

        double horizontalSquared = dToHive * dToHive - pivotHeight * pivotHeight;

        double trueRelativeY = Math.sqrt(Math.abs(horizontalSquared)) * Math.signum(relativeY);

        double reportedAngle = Math.atan2(relativeZ, relativeY);
        double trueAngle = Math.atan2(-pivotHeight, trueRelativeY);

        double hiveTilt = AngleUnit.normalizeRadians(trueAngle - reportedAngle);

        HiveState hiveState;
        if (-HIVE_MAX_TILT < hiveTilt && hiveTilt < 0) {
            hiveState = HiveState.AUDIENCE_UP;
        } else if (0 < hiveTilt && hiveTilt < HIVE_MAX_TILT) {
            hiveState = HiveState.SCORING_UP;
        } else {
            return null; // hive is at an impossible tilt
        }

        double realY = pivotY + trueRelativeY;

        // subtract fitted limelight - pedro error (see Tunables)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double forward = Tunables.visionCameraErrorForward;
        double left = Tunables.visionCameraErrorLeft;
        double errorX = forward * cos - left * sin + (isRed ? Tunables.visionRedBiasX : Tunables.visionBlueBiasX);
        double errorY = forward * sin + left * cos + (isRed ? Tunables.visionRedBiasY : Tunables.visionBlueBiasY);
        double newX = x - errorX;
        double newY = realY - errorY;

        // reject jumps farther than the robot could drive at top speed since the last accepted pose.
        // If vision keeps disagreeing, the robot was probably carried/hit, so eventually re-acquire.
        if (hasBotPose) {
            double seconds = staleTimer.get(TimeUnit.MILLISECONDS) / 1000.0;
            double maxTravel = Tunables.visionMaxSpeed * seconds + Tunables.visionJumpMargin;
            if (Math.hypot(newX - lastBotPose.x(), newY - lastBotPose.y()) > maxTravel) {
                if (++jumpRejects < Tunables.visionJumpRejectsBeforeReset) return null;
            }
        }
        jumpRejects = 0;

        lastHiveState = hiveState;
        lastBotPose = new Pose(newX, newY);
        hasBotPose = true;
        staleTimer.reset();

        long latencyMs = Math.round(result.getCaptureLatency() + result.getTargetingLatency());
        long acquisitionTimeMs = result.getControlHubTimeStamp() - latencyMs;

        double[] stdDev = result.getStddevMt1();
        double stdX = stdDev != null && stdDev.length > 0 ? Math.abs(stdDev[0]) * METERS_TO_INCHES : 0;
        double stdY = stdDev != null && stdDev.length > 1 ? Math.abs(stdDev[1]) * METERS_TO_INCHES : 0;

        double visionHeading = AngleUnit.normalizeRadians(
                botpose3D.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI / 2);

        // DON'T SCREW WITH X
        lastResult = new VisionResult(new Pose(x, newY), acquisitionTimeMs,
                result.getBotposeTagCount(), avgDist, stdX, stdY, visionHeading);
        return lastResult;
    }

    public boolean isReasonable(LLResult result) { // check if result is reasonable
        return result != null && result.isValid() && result.getBotpose() != null;
    }

    private boolean isNewFrame(LLResult result) {
        double timestamp = result.getTimestamp();
        long hubTime = result.getControlHubTimeStamp();
        boolean isNew = (timestamp != 0 && timestamp != lastFrameTimestamp) || hubTime != lastFrameHubTime;
        lastFrameTimestamp = timestamp;
        lastFrameHubTime = hubTime;
        return isNew;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public Pose getLastBotPose () {
        return lastBotPose;
    }

    public HiveState getLastHiveState() {
        return lastHiveState;
    }

    // last accepted frame (may be a few milliseconds old); null until the first accepted frame
    public VisionResult getResult() {
        return lastResult;
    }
}

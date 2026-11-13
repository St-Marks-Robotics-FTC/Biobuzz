/**
 * RESOURCES
 * red tag IDs are 30-37
 * blue tag IDs are 38-45
 * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
 * https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
 *
 *
 */

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Tunables;

import java.util.Arrays;

public class Vision {
    private static final double HIVE_MAX_TILT = Math.toRadians(35); // tilt past this is impossible
    private static final double HIVE_CENTER_X = 72; // hive center, pedro x
    private static final double METERS_TO_INCHES = 39.3701;

    private static final double[] DIST_EDGES = {50, 65}; // in, std dev bin edges
    private static final double[] SIGMA_X = {5.7, 4.1, 3.3}; // in, per distance group
    private static final double[] SIGMA_Y = {7.6, 7.6, 4.8}; // in, per distance group, diff groups have diff biases
    private static final int JUMP_WINDOW = 8; // frames in rolling median
    private static final int JUMP_MIN_FRAMES = 3; // need this many before judging
    private final double[] recentX = new double[JUMP_WINDOW];
    private final double[] recentY = new double[JUMP_WINDOW];
    private int recentCount = 0;
    private int recentNext = 0;

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

    public class VisionResult {
        private Pose pose;
        private int tagCount;
        private double[] stdDev;
        private double avgTagDistance; // average distance from the camera to the tags it used, in inches
        private long acquisitionTime; // acquisition time from System.currentTimeMillis()

        public VisionResult(Pose pose, int tagCount, double[] stdDev, double avgTagDistance, long acquisitionTime) {
            this.pose = pose;
            this.tagCount = tagCount;
            this.stdDev = stdDev;
            this.avgTagDistance = avgTagDistance;
            this.acquisitionTime = acquisitionTime;
        }

        public Pose pose() {
            return pose;
        }

        public int tagCount() {
            return tagCount;
        }

        public double[] stdDev() {
            return stdDev;
        }

        public double avgTagDistance() {
            return avgTagDistance;
        }

        public long acquisitionTime() {
            return acquisitionTime;
        }

        public int staleness() {
            return Math.toIntExact(System.currentTimeMillis() - acquisitionTime);
        }
    }

    private boolean started = false;
    private boolean isBlueTeam;

    /** vars that change from update() **/

    private Pose lastBotPose = new Pose(0, 0);
    private HiveState lastHiveState = HiveState.UNKNOWN;
    private int lastTagCount = 0; // number of tags seen in the last update
    private long lastCaptureTimeMs = 0; // when the image behind lastBotPose was taken (System.currentTimeMillis() clock)
    private VisionResult latestResult = null; // result from the newest frame, null if the newest frame wasn't valid
    private Position lastRawPosition = null; // newest valid Limelight pose in Pedro units, before any correction
    private double lastRawFrameStamp = Double.NaN; // Control Hub timestamp of the frame behind lastRawPosition; changes once per new frame

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
        if (isBlueTeam) startPipeline(Pipeline.BLUE);
        else startPipeline(Pipeline.RED);
    }

    public void startPipeline(Pipeline pipeline) {
        if (!started) {
            limelight.start();
            started = true;
        }

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    public LLStatus getStatus() { return limelight.getStatus(); }
    public Pipeline getPipeline() { return Pipeline.values()[getStatus().getPipelineIndex()]; } // convert from limelight pipeline ordinal/index to Pipeline enum

    // return staleness of botpose in milliseconds (time since the image was taken, not since we last read it)
    public double getStaleness() { return System.currentTimeMillis() - lastCaptureTimeMs; }

    public void update() {
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

        if (!isReasonable(result)) {
            latestResult = null; // newest frame is unusable
            return;
        }
        if (result.getControlHubTimeStamp() == lastRawFrameStamp) return; // same frame, keep last verdict

        latestResult = null; // stays null unless this frame passes every check

        Pose3D rawPose = result.getBotpose();
        Position botpose = convertToPedro(rawPose);
        lastRawPosition = botpose;
        lastRawFrameStamp = result.getControlHubTimeStamp();

        double avgTagDistance = result.getBotposeAvgDist() * METERS_TO_INCHES;
        double pivotHeight = Tunables.visionHivePivotHeight;

        double x = botpose.x;
        double z = botpose.z - Tunables.visionZOffset - Tunables.visionZPerDist * avgTagDistance;

        double relativeY = botpose.y - Tunables.visionHivePivotY;
        double relativeZ = z - pivotHeight;

        double dToHive = Math.sqrt(Math.pow(relativeY, 2) + Math.pow(relativeZ, 2));

        if (dToHive < pivotHeight) return; // impossible state -> reject data

        double horizontalSquared = dToHive * dToHive - pivotHeight * pivotHeight;

        double trueRelativeY = Math.sqrt(Math.abs(horizontalSquared)) * Math.signum(relativeY);

        double reportedAngle = Math.atan2(relativeZ, relativeY);
        double trueAngle = Math.atan2(-pivotHeight, trueRelativeY);

        double hiveTilt = AngleUnit.normalizeRadians(trueAngle - reportedAngle);

        if (-HIVE_MAX_TILT < hiveTilt && hiveTilt < 0) {
            lastHiveState = HiveState.AUDIENCE_UP;
        } else if (0 < hiveTilt && hiveTilt < HIVE_MAX_TILT) {
            lastHiveState = HiveState.SCORING_UP;
        } else {
            return; // hive is at an impossible tilt
        }

        if (trueRelativeY == 0) return; // directly under the pivot; the std dev math below would divide by zero

        double realY = Tunables.visionHivePivotY + trueRelativeY;

        // limelight heading, same frame as the pose, in pedro convention
        double heading = AngleUnit.normalizeRadians(rawPose.getOrientation().getYaw(AngleUnit.RADIANS) - Math.PI / 2);

        // camera sits off robot center, so the error rotates with heading
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double forward = Tunables.visionCameraErrorForward;
        double left = Tunables.visionCameraErrorLeft;

        double biasX = isBlueTeam ? Tunables.visionBlueBiasX : Tunables.visionRedBiasX;
        double biasY = isBlueTeam ? Tunables.visionBlueBiasY : Tunables.visionRedBiasY;

        double correctedX = x - (forward * cos - left * sin + biasX); //Equation claude came up with
        double correctedY = realY - (forward * sin + left * cos + biasY);

        lastBotPose = new Pose(correctedX, correctedY);
        // time the image was taken = time the Control Hub received it - camera capture latency - processing latency
        lastCaptureTimeMs = Math.round(result.getControlHubTimeStamp() - result.getCaptureLatency() - result.getTargetingLatency());

        // hive math multiplies z error by this: high means unusable => bad data
        double zSensitivity = Math.abs(relativeZ / trueRelativeY);
        if (zSensitivity > Tunables.visionMaxZSensitivity) return;

        // too close to the hive, position error makes robot teleport
        double goalDist = Math.hypot(correctedX - HIVE_CENTER_X, correctedY - Tunables.visionHivePivotY);

        if (goalDist < Tunables.visionMinGoalDist) return;

        if (isJump(correctedX, correctedY)) return; // far from recent frames

        double[] stdDev = getPedroStdDev(result, relativeY, relativeZ, trueRelativeY);
        int bin = goalDist < DIST_EDGES[0] ? 0 : goalDist < DIST_EDGES[1] ? 1 : 2;
        double inflate = Math.max(1, zSensitivity / Tunables.visionSensitivityRef); // trust shaky z less
        stdDev[0] = Math.max(stdDev[0], SIGMA_X[bin]);
        stdDev[1] = Math.max(stdDev[1], SIGMA_Y[bin] * inflate);

        latestResult = new VisionResult(lastBotPose, lastTagCount, stdDev, avgTagDistance, lastCaptureTimeMs);
    }

    // true if frame is far from the rolling median; always records the frame
    private boolean isJump(double x, double y) {
        boolean jump = recentCount >= JUMP_MIN_FRAMES
                && Math.hypot(x - median(recentX), y - median(recentY)) > Tunables.visionJumpMax;

        recentX[recentNext] = x;
        recentY[recentNext] = y;
        recentNext = (recentNext + 1) % JUMP_WINDOW;
        recentCount = Math.min(recentCount + 1, JUMP_WINDOW);
        return jump;
    }

    // median of the filled part of a recent-frame buffer
    private double median(double[] buffer) {
        double[] sorted = Arrays.copyOf(buffer, recentCount);
        Arrays.sort(sorted);
        int mid = recentCount / 2;
        return recentCount % 2 == 1 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
    }

    // returns {std dev of x, std dev of y} in inches, Pedro coordinates
    private double[] getPedroStdDev(LLResult result, double relativeY, double relativeZ, double trueRelativeY) {
        double[] llStdDev = result.getStddevMt1(); // limelight coordinates: {x, y, z, roll, pitch, yaw}, meters and degrees

        double stdX = llStdDev[1] * METERS_TO_INCHES; // pedro x comes from limelight y
        double stdRawY = llStdDev[0] * METERS_TO_INCHES; // pedro y comes from limelight x
        double stdZ = llStdDev[2] * METERS_TO_INCHES;

        // realY goes through the hive math, which uses both y and z, so both of their errors end up in realY:
        // d(trueRelativeY)/d(relativeY) = relativeY / trueRelativeY, d(trueRelativeY)/d(relativeZ) = relativeZ / trueRelativeY
        double yFromY = (relativeY / trueRelativeY) * stdRawY;
        double yFromZ = (relativeZ / trueRelativeY) * stdZ;
        double stdY = Math.sqrt(yFromY * yFromY + yFromZ * yFromZ);

        return new double[] {stdX, stdY};
    }

    // result from the newest frame, or null if the newest frame wasn't valid
    // the same frame is returned every loop until the Limelight sends a new one
    public VisionResult getResult() {
        return latestResult;
    }

    public Position convertToPedro(Pose3D rawPos) { // convert from Limelight coordinates to Pedro
        Position botpose = rawPos.getPosition().toUnit(DistanceUnit.INCH);
        double x = botpose.y + 72; // convert to pedro units
        double y = 72 - botpose.x; // convert to pedro units
        double z = botpose.z;

        return new Position(DistanceUnit.INCH, x, y, z, botpose.acquisitionTime);
    }

    public boolean isReasonable(LLResult result) { // check if result is reasonable
        if (result == null) return false; // check for null before touching result

        lastTagCount = result.getBotposeTagCount();

        return result.isValid() && lastTagCount >= Tunables.minTagCount;
    }


    /** DO NOT USE THESE FOR FUSION. THEY ARE FOR TESTING  **/
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public Pose getLastBotPose () {
        return lastBotPose;
    }

    // newest valid Limelight pose in Pedro units, unfixed (null until the first valid frame)
    public Position getLastRawPosition() {
        return lastRawPosition;
    }

    // changes every time update() sees a new valid Limelight frame, including frames the hive fix later rejects
    public double getLastRawFrameStamp() {
        return lastRawFrameStamp;
    }

    public HiveState getLastHiveState() {
        return lastHiveState;
    }

    public int getLastTagCount() {
        return lastTagCount;
    }

    public long getLastCaptureTimeMs() {
        return lastCaptureTimeMs;
    }
}

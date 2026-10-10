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
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Tunables;

import java.nio.channels.Pipe;
import java.util.List;

public class Vision {
    private double HIVE_PIVOT_HEIGHT = 43.95; // hive pivot height in inches
    private double HIVE_PIVOT_Y = 72; // y-coordinate of hive pivot
    private double HIVE_MAX_TILT = Math.toRadians(35); // max tilt of hive in radians

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
        latestResult = null; // stays null unless this frame passes every check

        LLResult result = limelight.getLatestResult();

        if (isReasonable(result)) {
            Position botpose = convertToPedro(result.getBotpose());

            double x = botpose.x;
            double y = botpose.y;
            double z = botpose.z;

            double relativeY = y - HIVE_PIVOT_Y;
            double relativeZ = z - HIVE_PIVOT_HEIGHT;

            double dToHive = Math.sqrt(Math.pow(relativeY, 2) + Math.pow(relativeZ, 2));

            if (dToHive < HIVE_PIVOT_HEIGHT) return; // impossible state -> reject data

            double horizontalSquared = dToHive * dToHive - HIVE_PIVOT_HEIGHT * HIVE_PIVOT_HEIGHT;

            double trueRelativeY = Math.sqrt(Math.abs(horizontalSquared)) * Math.signum(relativeY);

            double reportedAngle = Math.atan2(relativeZ, relativeY);
            double trueAngle = Math.atan2(-HIVE_PIVOT_HEIGHT, trueRelativeY);

            double hiveTilt = AngleUnit.normalizeRadians(trueAngle - reportedAngle);

            if (-HIVE_MAX_TILT < hiveTilt && hiveTilt < 0) {
                lastHiveState = HiveState.AUDIENCE_UP;
            } else if (0 < hiveTilt && hiveTilt < HIVE_MAX_TILT) {
                lastHiveState = HiveState.SCORING_UP;
            } else {
                return; // hive is at an impossible tilt
            }

            if (trueRelativeY == 0) return; // directly under the pivot; the std dev math below would divide by zero

            double realY = HIVE_PIVOT_Y + trueRelativeY;

            lastBotPose = new Pose(x, realY);

            // time the image was taken = time the Control Hub received it - camera capture latency - processing latency
            lastCaptureTimeMs = Math.round(result.getControlHubTimeStamp() - result.getCaptureLatency() - result.getTargetingLatency());

            double avgTagDistance = result.getBotposeAvgDist() * 39.3701; // meters -> inches

            latestResult = new VisionResult(lastBotPose, lastTagCount, getPedroStdDev(result, relativeY, relativeZ, trueRelativeY), avgTagDistance, lastCaptureTimeMs);
        }
    }

    // returns {std dev of x, std dev of y} in inches, Pedro coordinates
    private double[] getPedroStdDev(LLResult result, double relativeY, double relativeZ, double trueRelativeY) {
        double[] llStdDev = result.getStddevMt1(); // limelight coordinates: {x, y, z, roll, pitch, yaw}, meters and degrees
        double metersToInches = 39.3701;

        double stdX = llStdDev[1] * metersToInches; // pedro x comes from limelight y
        double stdRawY = llStdDev[0] * metersToInches; // pedro y comes from limelight x
        double stdZ = llStdDev[2] * metersToInches;

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

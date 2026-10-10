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
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.nio.channels.Pipe;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private static final double HIVE_PIVOT_HEIGHT = 43.95; // hive pivot height in inches
    private static final double HIVE_PIVOT_Y = 72; // y-coordinate of hive pivot
    private static final double HIVE_MAX_TILT = Math.toRadians(35); // max tilt of hive in radians
    private static final double FIELD_MARGIN = 6; // inches a fix may land outside the field before it's rejected

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

    private boolean started = false;
    private boolean isBlueTeam;
    private Pose lastBotPose = new Pose(0, 0);
    private HiveState lastHiveState = HiveState.UNKNOWN;
    private double lastHiveTilt = 0; // tilt in radians from the last accepted frame
    private int lastTagCount = 0;
    private boolean newPose = false; // an accepted frame nobody has taken yet
    private double lastFrameTimestamp = Double.NaN; // Limelight timestamp of the last frame we looked at
    private HiveState pendingHiveState = HiveState.UNKNOWN; // state being confirmed
    private int pendingHiveCount = 0; // consecutive frames that agreed with pendingHiveState

    private Timer staleTimer; // how stale lastBotPose is
    private Timer hiveTimer; // how long ago lastHiveState was last confirmed

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
        if (isBlueTeam) startPipeline(Pipeline.BLUE);
        else startPipeline(Pipeline.RED);
        staleTimer = new Timer();
        hiveTimer = new Timer();
    }

    public void startPipeline(Pipeline pipeline) {
        if (!started) limelight.start();

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    public LLStatus getStatus() { return limelight.getStatus(); }
    public Pipeline getPipeline() { return Pipeline.values()[getStatus().getPipelineIndex()]; } // convert from limelight pipeline ordinal/index to Pipeline enum

    // return staleness of botpose in milliseconds
    public double getStaleness() { return staleTimer.get(TimeUnit.MILLISECONDS);}

    // Call once per loop: reads the newest Limelight frame and refreshes the bot pose and hive state.
    public void update() {
        LLResult result = limelight.getLatestResult();
        if (!isReasonable(result)) return;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;
        Position pos = botpose.getPosition().toUnit(DistanceUnit.INCH);

        processFrame(result.getTimestamp(), result.getBotposeTagCount(), result.getStaleness(), pos.x, pos.y, pos.z,
                sideFromTags(result.getFiducialResults()));
    }

    // Handles one frame (botpose in inches, Limelight/FTC field frame). Split out of update() so it can run without a camera.
    void processFrame(double timestamp, int tagCount, double stalenessMs, double ftcX, double ftcY, double ftcZ, int side) {
        if (timestamp == lastFrameTimestamp) return; // same frame we already handled
        lastFrameTimestamp = timestamp;

        if (tagCount < Tunables.visionMinTags || stalenessMs > Tunables.visionMaxStalenessMs) return; // too weak or too old to trust

        HiveFix fix = solveHive(ftcX, ftcY, ftcZ, side);
        if (fix == null) return; // impossible geometry -> reject data
        if (fix.x < -FIELD_MARGIN || fix.x > 144 + FIELD_MARGIN || fix.y < -FIELD_MARGIN || fix.y > 144 + FIELD_MARGIN) return; // off the field -> reject data

        lastBotPose = new Pose(fix.x, fix.y);
        lastTagCount = tagCount;
        lastHiveTilt = fix.tilt;
        newPose = true;
        staleTimer.reset();

        confirmHiveState(fix.tilt);
    }

    // The hive has a tag face on each side (limelight/fmap): a robot only sees the face on the side it is standing on.
    private static boolean isAudienceFaceTag(int id) { return (id >= 38 && id <= 41) || (id >= 34 && id <= 37); } // blue, red
    private static boolean isScoringFaceTag(int id) { return (id >= 42 && id <= 45) || (id >= 30 && id <= 33); } // blue, red

    // +1 if most visible tags are on the scoring face (robot is on the scoring side), -1 for the audience face, 0 if unclear
    private static int sideFromTags(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return 0;
        int audience = 0, scoring = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (isAudienceFaceTag(tag.getFiducialId())) audience++;
            else if (isScoringFaceTag(tag.getFiducialId())) scoring++;
        }
        if (scoring > audience) return 1;
        if (audience > scoring) return -1;
        return 0;
    }

    /**
     * The camera solves the robot pose assuming the hive is flat, but the hive tilts about its pivot, so the reported
     * pose is the true pose rotated about the pivot. The robot is really on the floor, which gives the true position
     * and the tilt (tilt > 0 raises the scoring side, tilt < 0 the audience side).
     *
     *         pivot ●
     *               |\
     *               | \
     *             h |  \  dToHive  (distance from pivot to robot)
     *               |   \
     *               |    \
     *         floor ●-----● robot
     *               horizontal
     *
     * Takes the Limelight botpose in inches (FTC field frame) and returns Pedro units, or null if it is impossible.
     * hintSide is +1 if the robot is on the scoring side of the hive (y > pivot), -1 on the audience side, 0 if unknown.
     */
    static HiveFix solveHive(double ftcX, double ftcY, double ftcZ, int hintSide) {
        double x = ftcY + 72; // convert to pedro units
        double y = 72 - ftcX; // convert to pedro units

        double relativeY = y - HIVE_PIVOT_Y;
        double relativeZ = ftcZ - HIVE_PIVOT_HEIGHT;

        double dToHive = Math.sqrt(relativeY * relativeY + relativeZ * relativeZ);

        if (dToHive < HIVE_PIVOT_HEIGHT) return null; // impossible state -> reject data

        double horizontalSquared = dToHive * dToHive - HIVE_PIVOT_HEIGHT * HIVE_PIVOT_HEIGHT;

        // which side of the pivot the robot is on: tilting can flip the sign of the reported relativeY for a robot close
        // to the hive, so trust the visible tag faces when we have them
        double side = hintSide != 0 ? hintSide : Math.signum(relativeY);
        double trueRelativeY = Math.sqrt(horizontalSquared) * side;

        double reportedAngle = Math.atan2(relativeZ, relativeY);
        double trueAngle = Math.atan2(-HIVE_PIVOT_HEIGHT, trueRelativeY);

        double hiveTilt = AngleUnit.normalizeRadians(trueAngle - reportedAngle);

        if (Math.abs(hiveTilt) > HIVE_MAX_TILT) return null; // hive is at an impossible tilt

        return new HiveFix(x, HIVE_PIVOT_Y + trueRelativeY, hiveTilt);
    }

    // The hive only changes state when it physically tilts, so a new state must be seen on several frames in a row.
    private void confirmHiveState(double tilt) {
        double minTilt = Math.toRadians(Tunables.hiveMinTiltDeg);
        HiveState seen;
        if (tilt < -minTilt) {
            seen = HiveState.AUDIENCE_UP;
        } else if (tilt > minTilt) {
            seen = HiveState.SCORING_UP;
        } else {
            return; // looks flat: no information, keep the last state
        }

        if (seen == lastHiveState) {
            pendingHiveCount = 0;
            hiveTimer.reset();
            return;
        }

        if (seen == pendingHiveState) {
            pendingHiveCount++;
        } else {
            pendingHiveState = seen;
            pendingHiveCount = 1;
        }

        if (pendingHiveCount >= Tunables.hiveConfirmFrames) {
            lastHiveState = seen;
            pendingHiveCount = 0;
            hiveTimer.reset();
        }
    }

    // one Limelight frame converted to Pedro units; y is corrected for hive tilt
    static class HiveFix {
        final double x, y, tilt;

        HiveFix(double x, double y, double tilt) {
            this.x = x;
            this.y = y;
            this.tilt = tilt;
        }
    }

    public boolean isReasonable(LLResult result) { // check if result is reasonable
        return result != null && result.isValid();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public Pose getLastBotPose () {
        return lastBotPose;
    }

    // true when a frame has been accepted since the last takePose()
    public boolean hasNewPose() {
        return newPose;
    }

    // latest accepted pose (x/y only, y corrected for hive tilt); marks it as consumed
    public Pose takePose() {
        newPose = false;
        return lastBotPose;
    }

    public int getLastTagCount() {
        return lastTagCount;
    }

    // last hive state the camera confirmed; stays put until the camera confirms a different one
    public HiveState getLastHiveState() {
        return lastHiveState;
    }

    // how long ago the camera last confirmed lastHiveState, in milliseconds
    public double getHiveAgeMs() {
        return hiveTimer.get(TimeUnit.MILLISECONDS);
    }

    public double getLastHiveTilt() {
        return lastHiveTilt;
    }
}

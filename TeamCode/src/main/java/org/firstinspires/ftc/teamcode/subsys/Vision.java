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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.nio.channels.Pipe;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
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

    private boolean started = false;
    private boolean isBlueTeam;
    private Pose lastBotPose = new Pose(0, 0);
    private boolean hasBotPose = false; // lastBotPose is a real reading (enables the jump check)
    private boolean newBotPose = false; // accepted since last takeNewBotPose()
    private long lastFrameTime = -1; // control hub timestamp of the last processed frame
    private HiveState lastHiveState = HiveState.UNKNOWN;

    private Timer staleTimer; // how stale lastBotPose is

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
        if (isBlueTeam) startPipeline(Pipeline.BLUE);
        else startPipeline(Pipeline.RED);
        staleTimer = new Timer();
    }

    public void startPipeline(Pipeline pipeline) {
        if (!started) limelight.start();

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    public LLStatus getStatus() { return limelight.getStatus(); }
    public Pipeline getPipeline() { return Pipeline.values()[getStatus().getPipelineIndex()]; } // convert from limelight pipeline ordinal/index to Pipeline enum

    // return staleness of botpose in milliseconds
    public double getStaleness() { return staleTimer.get(TimeUnit.MILLISECONDS);}

    // heading: follower.pose().heading(), radians
    public void update(double heading) {
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

        if (isReasonable(result) && result.getControlHubTimeStamp() != lastFrameTime) {
            lastFrameTime = result.getControlHubTimeStamp(); // getLatestResult repeats frames between camera updates
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            boolean isRed = getPipeline() == Pipeline.RED;
            double x = botpose.y + 72; // convert to pedro units
            double y = 72 - botpose.x; // convert to pedro units
            double avgDist = result.getBotposeAvgDist() * 39.37; // m -> in
            double z = botpose.z - Tunables.visionZOffset - Tunables.visionZPerDist * avgDist;

            double pivotHeight = Tunables.visionHivePivotHeight;
            double pivotY = Tunables.visionHivePivotY;

            double relativeY = y - pivotY;
            double relativeZ = z - pivotHeight;

            double dToHive = Math.sqrt(Math.pow(relativeY, 2) + Math.pow(relativeZ, 2));

            if (dToHive < pivotHeight) return; // impossible state -> reject data

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
                return; // hive is at an impossible tilt
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

            // reject jumps farther than the robot could drive at top speed since the last accepted pose (to prevent the teleportation)
            if (hasBotPose) {
                double seconds = staleTimer.get(TimeUnit.MILLISECONDS) / 1000.0;
                double maxTravel = Tunables.visionMaxSpeed * seconds + Tunables.visionJumpMargin;
                if (Math.hypot(newX - lastBotPose.x(), newY - lastBotPose.y()) > maxTravel) return;
            }

            lastHiveState = hiveState;
            lastBotPose = new Pose(newX, newY);
            hasBotPose = true;
            newBotPose = true;
            staleTimer.reset();
        }
        else return;
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

    // accepted pose since the last call, else null
    public Pose takeNewBotPose() {
        if (!newBotPose) return null;
        newBotPose = false;
        return lastBotPose;
    }

    public HiveState getLastHiveState() {
        return lastHiveState;
    }
}

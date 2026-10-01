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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.nio.channels.Pipe;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    private double HIVE_PIVOT_HEIGHT = 43.95; // hive pivot height in inches
    private double HIVE_PIVOT_Y = 70; // y-coordinate of hive pivot
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

    public Pose getBotPose() {
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

        if (result != null && result.isValid()) {
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            double x = botpose.y + 72; // convert to pedro units
            double y = 72 - botpose.x; // convert to pedro units
            double z = botpose.z;

            double relativeY = y - HIVE_PIVOT_Y;
            double relativeZ = z - HIVE_PIVOT_HEIGHT;

            double dToHive = Math.sqrt(Math.pow(relativeY, 2) + Math.pow(relativeZ, 2));

            if (dToHive < HIVE_PIVOT_HEIGHT) return null; // impossible state -> reject data

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
                return null; // hive is at an impossible tilt
            }

            double realY = HIVE_PIVOT_Y + trueRelativeY;

            lastBotPose = new Pose(x, realY);
            staleTimer.reset();
            return lastBotPose;
        }
        else return null;
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
}

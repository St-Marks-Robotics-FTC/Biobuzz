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

    public static Limelight3A limelight;

    public enum Pipeline { // expression of our limelight pipelines; order and elements must match exactly with pipeline indices on camera
        BLUE, // pipeline index: 0
        RED   // pipeline index: 1
    }

    private boolean started = false;
    private boolean isBlueTeam;
    private Pose lastBotPose;
    private Timer staleTimer; // how stale lastBotPose is

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
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

            double horizontalSquared = dToHive * dToHive - HIVE_PIVOT_HEIGHT * HIVE_PIVOT_HEIGHT;

            double realY = HIVE_PIVOT_Y + Math.sqrt(Math.abs(horizontalSquared)) * Math.signum(relativeY);

            lastBotPose = new Pose(x, realY);
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

    public Pose3D translateLLPoseToPedro(Pose3D rawPose3D) {
        Position rawPos = rawPose3D.getPosition().toUnit(DistanceUnit.INCH); // ensure we are using inches
        YawPitchRollAngles rawOrientation = rawPose3D.getOrientation();
        double newX = rawPos.y + 72;
        double newY = 72 - rawPos.x;
        double newZ = rawPos.z; // shouldn't need to translate this

        Position newPos = new Position(DistanceUnit.INCH, newX, newY, newZ, rawPos.acquisitionTime);

        return new Pose3D(newPos, rawOrientation);
    }
}

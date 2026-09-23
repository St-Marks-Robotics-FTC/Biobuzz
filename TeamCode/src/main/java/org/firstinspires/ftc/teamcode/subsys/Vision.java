/**
 * RESOURCES
 * red tag IDs are 30-37
 * blue tag IDs are 38-45
 * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
 * https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
 * TODO:
 * - find whether garden side is up/down accurately
 *
 * PLAN:
 * - only look at the tags of our team
 * 4 pipelines
 * - 2 blue (all blue tag IDs allowed)
 *  - blue: audience side up field map
 *  - blue: scoring side up field map
 * - 2 red (all red tag IDs allowed)
 *  - red: audience side up field map
 *  - red: scoring side up field map
 *
 *  use fiducials to figure out which field map to use (which side is up)
 *  use limelight to get our botpose
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

import java.nio.channels.Pipe;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    public static Limelight3A limelight;

    public enum Pipeline { // expression of our limelight pipelines; order and elements must match exactly with pipeline indices on camera
        BLUE_AUDIENCE_HIGH, // pipeline index: 0
        BLUE_SCORING_HIGH, // pipeline index: 1
        RED_AUDIENCE_HIGH, // pipeline index: 2
        RED_SCORING_HIGH, // pipeline index: 3
        FAILURE // can't identify which is necessary
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

        if (pipeline == Pipeline.FAILURE) {
            throw new RuntimeException("do not set to Pipeline.FAILURE");
        }

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    public LLStatus getStatus() { return limelight.getStatus(); }
    public Pipeline getPipeline() { return Pipeline.values()[getStatus().getPipelineIndex()]; } // convert from limelight pipeline ordinal/index to Pipeline enum

    // return staleness of botpose in milliseconds
    public double getStaleness() { return staleTimer.get(TimeUnit.MILLISECONDS);}

    public Pose update() {
        LLResult result = getLatestResult();
        Pipeline currentPipeline = getPipeline();
        Pipeline neededPipeline = getNeededPipeline(result);
        if (result != null && result.isValid() && neededPipeline != Pipeline.FAILURE) {
            if (currentPipeline == neededPipeline) {
                lastBotPose = translateLLPoseToField(result.getBotpose());
                staleTimer.reset();
            } else {
                startPipeline(neededPipeline);
            }
        }
        return lastBotPose;
    }

    public Pipeline getNeededPipeline(LLResult result) {
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // compute which tag group is closer based on average distances

            AverageFinder scoringTags = new AverageFinder(); // side closer to scoring zone
            AverageFinder audienceTags = new AverageFinder(); // side closer to audience

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                Pose3D pose = fiducial.getRobotPoseFieldSpace();
                double dst = XYDstFromPose3D(pose);

                if (isBlueTeam) { // use blue team IDs
                    if (42 <= id && id <= 45) {
                        scoringTags.add(dst);
                    } else if (38 <= id && id <= 41) {
                        audienceTags.add(dst);
                    }
                } else { // use red team IDs
                    if (30 <= id && id <= 33) {
                        scoringTags.add(dst);
                    } else if (34 <= id && id <= 37) {
                        audienceTags.add(dst);
                    }
                }
            }

            double scoringAvg = scoringTags.calc();
            double audienceAvg = audienceTags.calc();

            if (scoringAvg == 0 && audienceAvg == 0) return Pipeline.FAILURE;

            if (audienceAvg < scoringAvg) {
                if (isBlueTeam)  return Pipeline.BLUE_AUDIENCE_HIGH;
                else return Pipeline.RED_AUDIENCE_HIGH;
            } else {
                if (isBlueTeam)  return Pipeline.BLUE_SCORING_HIGH;
                else return Pipeline.RED_SCORING_HIGH;
            }
        }
        return Pipeline.FAILURE;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    private double XYDstFromPose3D(Pose3D pose3D) {
        Position pos = pose3D.getPosition();
        return Math.sqrt(Math.pow(pos.x, 2) + Math.pow(pos.y, 2));
    }

    private Pose translateLLPoseToField(Pose3D rawPose3D) {
        Position rawPos = rawPose3D.getPosition().toUnit(DistanceUnit.INCH); // ensure we are using inches
        double rawX = rawPos.y + 72;
        double rawY = 72 - rawPos.x;

        return new Pose(rawX, rawY); // doesn't have heading
    }

    private class AverageFinder {
        private double total;
        private int num;
        public AverageFinder() {
            total = 0;
            num = 0;
        }

        public void add(double entry) {
            total += entry;
            num++;
        }

        public double calc() { // return the average
            if (num == 0) return 0;
            return total / num;
        }
    }
}

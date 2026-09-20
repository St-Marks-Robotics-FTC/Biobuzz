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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Vision {
    public static Limelight3A limelight;

    public enum Pipeline { // expression of our limelight pipelines; order and elements must match exactly with pipeline indices on camera
        BLUE_TEAM, // pipeline index: 0
        RED_TEAM // pipeline index: 1
    }

    private boolean started = false;
    private boolean isBlueTeam;

    public Vision(HardwareMap hw, boolean isBlueTeam) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        this.isBlueTeam = isBlueTeam;
    }

    public void startPipeline(Pipeline pipeline) {
        if (!started) limelight.start();

        limelight.pipelineSwitch(pipeline.ordinal()); // convert from Pipeline enum to ordinal/index
    }

    // return 0 to indicate failure
    public double getRelativeDistance() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // compute which tag group is closer based on average distances

            AverageFinder scoringTags = new AverageFinder(); // side closer to scoring zone
            AverageFinder audienceTags = new AverageFinder(); // side closer to audience

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                Pose3D pose = fiducial.getRobotPoseTargetSpace();
                double dst = dstFromPose3D(pose);

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

            if (scoringAvg == 0 && audienceAvg == 0) return 0;

            if (scoringAvg > audienceAvg) return scoringAvg;
            else return audienceAvg;
        }
        return 0;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    private double dstFromPose3D(Pose3D pose3D) {
        Position pos = pose3D.getPosition();
        return Math.sqrt(Math.pow(pos.x, 2) + Math.pow(pos.y, 2) + Math.pow(pos.z, 2));
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

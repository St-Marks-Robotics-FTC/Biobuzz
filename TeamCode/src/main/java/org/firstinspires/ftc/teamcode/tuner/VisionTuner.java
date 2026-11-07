package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

@TeleOp(name="VisionTuner", group="Tuner")
public class VisionTuner extends LinearOpMode {
    private enum TuneMode {
        BLUE,
        RED
    }

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, Math.toRadians(90))); // start in center of field

        TuneMode mode = TuneMode.BLUE;
        boolean isCalibrating = false;
        int calibrationSpot = 1;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.addLine("select your tuning mode using the gamepad:");
            telemetryM.addLine("right bumper: " + mode0);
            telemetryM.addLine("left bumper: " + mode1);

            if (gamepad1.rightBumperWasPressed()) {
                mode = mode0;
                vision.startPipeline(Vision.Pipeline.BLUE);
            }
            if (gamepad1.leftBumperWasPressed()) {
                mode = mode1;
                vision.startPipeline(Vision.Pipeline.RED);
            }


            if (gamepad1.startWasPressed()) isCalibrating = !isCalibrating;
            if (gamepad1.dpadUpWasPressed()) calibrationSpot++;
            if (gamepad1.dpadDownWasPressed() && calibrationSpot > 1) calibrationSpot--;
            if (gamepad1.aWasPressed()) { // set odo to the tape-measured spot (Tunables)
                follower.setPose(new Pose(Tunables.visionTunerSpotX, Tunables.visionTunerSpotY, follower.pose().heading()));
            }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);
            telemetryM.addLine("toggle calibration with START button");
            telemetryM.addLine("calibration is: " + isCalibrating);
            telemetryM.addLine("calibration spot (dpad up/down): S" + calibrationSpot);
            telemetryM.addLine("A: set pose to visionTunerSpotX/Y");

            vision.update(follower.pose().heading());

            Position rawPos = vision.getLatestResult().getBotpose().getPosition();

            telemetryM.addData("raw position", rawPos.toString());
            telemetryM.addData("last botpose", vision.getLastBotPose().toString());
            telemetryM.addData("last hive state", vision.getLastHiveState());
            telemetryM.addData("staleness", vision.getStaleness());
            telemetryM.addData("odo pos", follower.pose().toString());
            telemetryM.addData("pipeline", vision.getPipeline());

            // log for limelight/fit_hive.py: spot, true_y, raw_y, raw_z, avg_dist (inches, pedro frame, uncorrected)
            // setPose to the tape-measured pose at each spot and hold still
            if (isCalibrating) {
                LLResult result = vision.getLatestResult();
                if (vision.isReasonable(result)) {
                    Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
                    double y = 72 - botpose.x; // convert to pedro units
                    double z = botpose.z;
                    double avgDist = result.getBotposeAvgDist() * 39.37; // m -> in

                    RobotLog.d("VisionTuner: " + calibrationSpot + ", " + follower.pose().y() + ", " + y + ", " + z + ", " + avgDist);
                }
            }

            telemetryM.update(telemetry);

            idle();
        }
    }
}

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
        String calibrationSpot = "A";
        boolean isCalibrating = false;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.addLine("select your tuning mode using the gamepad:");
            telemetryM.addLine("left bumper: " + mode0);
            telemetryM.addLine("right bumper: " + mode1);

            if (gamepad1.rightBumperWasPressed()) {
                mode = mode0;
                vision.startPipeline(Vision.Pipeline.BLUE);
            }
            if (gamepad1.leftBumperWasPressed()) {
                mode = mode1;
                vision.startPipeline(Vision.Pipeline.RED);
            }

            if (gamepad1.aWasPressed()) calibrationSpot = "A";
            if (gamepad1.bWasPressed()) calibrationSpot = "B";
            if (gamepad1.xWasPressed()) calibrationSpot = "X";
            if (gamepad1.yWasPressed()) calibrationSpot = "Y";

            if (gamepad1.dpadUpWasPressed()) calibrationSpot = "U";
            if (gamepad1.dpadDownWasPressed()) calibrationSpot = "D";
            if (gamepad1.dpadLeftWasPressed()) calibrationSpot = "L";
            if (gamepad1.dpadRightWasPressed()) calibrationSpot = "R";

            if (gamepad1.startWasPressed()) isCalibrating = !isCalibrating;

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);
            telemetryM.addLine("select calibration spot using A/B/X/Y");
            telemetryM.addLine("toggle calibration with START button");
            telemetryM.addLine("calibration is: " + isCalibrating);
            telemetryM.addLine("current calibration spot: " + calibrationSpot);

            vision.update();

            Position rawPos = vision.getLatestResult().getBotpose().getPosition();

            telemetryM.addData("raw position", rawPos.toString());
            telemetryM.addData("last botpose", vision.getLastBotPose().toString());
            telemetryM.addData("last hive state", vision.getLastHiveState());
            telemetryM.addData("staleness", vision.getStaleness());
            telemetryM.addData("odo pos", follower.pose().toString());
            telemetryM.addData("pipeline", vision.getPipeline());

            // update calibration data

            if (isCalibrating) {
                LLResult result = vision.getLatestResult();
                Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
                double y = 72 - botpose.x; // convert to pedro units
                double z = botpose.z;

                double avgDist = result.getBotposeAvgDist() * 39.37; // convert to inches

                // CSV rows: spot, true_y, raw_y, raw_z, avg_dist
                RobotLog.d("VisionTuner: " + calibrationSpot + ", " + follower.pose().y() + ", " + y + ", " + z + ", " + avgDist);
            }

            telemetryM.update(telemetry);

            idle();
        }
    }
}

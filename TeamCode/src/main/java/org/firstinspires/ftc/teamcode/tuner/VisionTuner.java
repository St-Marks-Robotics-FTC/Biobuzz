package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.addLine("select your tuning mode using the gampead:");
            telemetryM.addLine("(A): " + mode0);
            telemetryM.addLine("(B): " + mode1);

            if (gamepad1.aWasReleased()) {
                mode = mode0;
                vision.startPipeline(Vision.Pipeline.BLUE);
            }
            if (gamepad1.bWasReleased()) {
                mode = mode1;
                vision.startPipeline(Vision.Pipeline.RED);
            }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);

            vision.update();

            Position rawPos = vision.getLatestResult().getBotpose().getPosition();

            telemetryM.addData("raw position", rawPos.toString());
            telemetryM.addData("last botpose", vision.getLastBotPose().toString());
            telemetryM.addData("last hive state", vision.getLastHiveState());
            telemetryM.addData("staleness", vision.getStaleness());
            telemetryM.addData("odo pos", follower.pose().toString());
            telemetryM.addData("pipeline", vision.getPipeline());

            telemetryM.update(telemetry);

            idle();
        }
    }
}

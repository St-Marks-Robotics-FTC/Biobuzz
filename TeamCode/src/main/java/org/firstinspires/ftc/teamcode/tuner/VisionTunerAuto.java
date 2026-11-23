package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

/**
 * Drives the automatic vision-data sweep. Press A to start, B to stop.
 * The heavy lifting lives in VisionDataCollector.
 */
@TeleOp(name = "VisionTunerAuto", group = "Tuner")
public class VisionTunerAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true);
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(Tunables.visionTunerSpotX, Tunables.visionTunerSpotY, Math.toRadians(90)));

        VisionDataCollector collector = new VisionDataCollector(follower, vision);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addLine("A: start automatic data collection");
        telemetryM.addLine("B: stop and close the CSV");
        telemetryM.addLine("start pose: " + follower.pose());
        telemetryM.update(telemetry);

        waitForStart();

        boolean running = false;
        while (opModeIsActive()) {
            follower.update();

            if (!running && gamepad1.aWasPressed()) {
                collector.start();
                running = collector.isRunning();
            } else if (running && gamepad1.bWasPressed()) {
                collector.stop();
                running = false;
            }

            if (running) {
                collector.update();
                running = collector.isRunning();
            }

            LLResult result = vision.getLatestResult();
            telemetryM.addLine(collector.status());
            telemetryM.addData("odo x", follower.pose().x());
            telemetryM.addData("odo y", follower.pose().y());
            telemetryM.addData("tags", result != null ? result.getBotposeTagCount() : 0);
            telemetryM.update(telemetry);
            idle();
        }

        collector.stop();
    }
}

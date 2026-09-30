package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsys.Vision;

@TeleOp(name="VisionTuner", group="Tuner")
public class VisionTuner extends LinearOpMode {
    private enum TuneMode {
        FIDUCIALS,
        BOTPOSE
    }

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        TuneMode mode = TuneMode.BOTPOSE;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        vision.startPipeline(Vision.Pipeline.BLUE);

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.addLine("select your tuning mode using the gampead:");
            telemetryM.addLine("(A): " + mode0);
            telemetryM.addLine("(B): " + mode1);

            if (gamepad1.aWasReleased()) { mode = mode0; }
            if (gamepad1.bWasReleased()) { mode = mode1; }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);

            switch (mode) {
                case FIDUCIALS:
                    LLResult result = vision.getLatestResult();

                    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                        telemetryM.addLine("id: " + fiducial.getFiducialId());
                    }
                    break;
                case BOTPOSE:
                    // print botpose
                    vision.getBotPose();
                    Position rawPos = vision.getLatestResult().getBotpose().getPosition();
                    telemetryM.addData("raw position", rawPos.toString());
                    telemetryM.addData("last botpose", vision.getLastBotPose().toString());
                    telemetryM.addData("pipeline", vision.getPipeline());
                    break;
            }

            telemetryM.update(telemetry);

            idle();
        }
    }
}

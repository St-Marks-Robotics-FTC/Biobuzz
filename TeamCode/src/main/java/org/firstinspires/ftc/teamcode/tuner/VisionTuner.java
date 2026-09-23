package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsys.Vision;

public class VisionTuner extends LinearOpMode {
    private enum TuneMode {
        FIDUCIALS,
        BOTPOSE
    }

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        TuneMode mode = TuneMode.FIDUCIALS;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        vision.startPipeline(Vision.Pipeline.BLUE_AUDIENCE_HIGH);

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

                    // print fiducials stuff
                    break;
                case BOTPOSE:
                    // print botpose
                    break;
            }

            telemetryM.update(telemetry);

            idle();
        }
    }
}

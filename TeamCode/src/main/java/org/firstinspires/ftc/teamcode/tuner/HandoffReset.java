package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HandoffState;

@TeleOp(name="HandoffReset", group="Tuner")
public class HandoffReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("start OpMode to reset HandoffState.pose");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            HandoffState.pose = new Pose(72, 72, Math.toRadians(90));
            telemetryM.addLine("HandoffState.pose reset!");
            telemetryM.update(telemetry);
            idle();
        }
    }
}

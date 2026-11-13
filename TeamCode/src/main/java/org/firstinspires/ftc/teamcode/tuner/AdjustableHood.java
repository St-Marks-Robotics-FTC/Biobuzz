package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;

@Configurable
@TeleOp(name="AdjustableHood", group="Tuner")
public class AdjustableHood extends LinearOpMode {
    public static double flowerRpm = Tunables.flowerRpm;
    public static double pollenRpm = Tunables.pollenRpm;

    private enum Ball {
        FLOWER,
        POLLEN
    }

    @Override
    public void runOpMode() {
        HWScheduler scheduler = new HWScheduler(hardwareMap);
        Flywheel flywheel = new Flywheel(scheduler);
        VoltageSensor batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        Servo hood = hardwareMap.get(Servo.class, "hoodServo");
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Ball ball = Ball.FLOWER;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) ball = Ball.FLOWER;
            if (gamepad1.rightBumperWasPressed()) ball = Ball.POLLEN;

            double targetRpm = ball == Ball.FLOWER ? flowerRpm : pollenRpm;
            hood.setPosition(ball == Ball.FLOWER ? Tunables.nectarPos : Tunables.pollenPos);
            flywheel.update(targetRpm, batteryVoltage.getVoltage());
            scheduler.update(0); // write flywheel power

            telemetryM.addLine("left bumper: FLOWER, right bumper: POLLEN");
            telemetryM.addData("mode", ball);
            telemetryM.addData("target RPM", targetRpm);
            telemetryM.addData("current RPM", flywheel.getRPM());
            telemetryM.addData("hood pos", hood.getPosition());
            telemetryM.update(telemetry);

            idle();
        }

        flywheel.update(0, batteryVoltage.getVoltage());
        scheduler.update(0);
    }
}

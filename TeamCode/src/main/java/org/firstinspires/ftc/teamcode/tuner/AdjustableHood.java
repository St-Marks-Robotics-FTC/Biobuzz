package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;

@Configurable
@TeleOp(name="AdjustableHood", group="Tuner")
public class AdjustableHood extends LinearOpMode {
    public static double flowerRpm = 3700;
    public static double pollenRpm = 3100;

    private enum Ball {
        FLOWER,
        POLLEN
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Flywheel flywheel = robot.flywheel;
        Servo hood = hardwareMap.get(Servo.class, "hoodServo");
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Ball ball = Ball.FLOWER;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.leftBumperWasPressed()) ball = Ball.FLOWER;
            if (gamepad1.rightBumperWasPressed()) ball = Ball.POLLEN;

            double targetRpm = ball == Ball.FLOWER ? flowerRpm : pollenRpm;
            hood.setPosition(ball == Ball.FLOWER ? Tunables.flowerPos : Tunables.pollenPos);
            flywheel.update(targetRpm, robot.getExpansionHubVoltage());

            telemetryM.addLine("left bumper: FLOWER, right bumper: POLLEN");
            telemetryM.addData("mode", ball);
            telemetryM.addData("target RPM", targetRpm);
            telemetryM.addData("current RPM", flywheel.getRPM());
            telemetryM.addData("hood pos", hood.getPosition());
            telemetryM.update(telemetry);

            idle();
        }

        flywheel.update(0, robot.getExpansionHubVoltage());
    }
}

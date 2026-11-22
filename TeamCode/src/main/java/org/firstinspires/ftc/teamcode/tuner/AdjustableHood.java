package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.io.HubDevice;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;
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
        IoThread io = new IoThread(hardwareMap);
        Flywheel flywheel = new Flywheel(io);
        HubDevice expansionHub = new HubDevice(io, "Expansion Hub 2", ReadPeriod.millis(100)); // battery voltage
        ServoDevice hood = new ServoDevice(io, "hoodServo", Tunables.nectarPos, WritePolicy.DEFAULT); // starts on FLOWER
        io.start();
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Ball ball = Ball.FLOWER;

        try {
            waitForStart();

            while (opModeIsActive()) {
                io.awaitNewCycle();

                if (gamepad1.leftBumperWasPressed()) ball = Ball.FLOWER;
                if (gamepad1.rightBumperWasPressed()) ball = Ball.POLLEN;

                double targetRpm = ball == Ball.FLOWER ? flowerRpm : pollenRpm;
                hood.setPosition(ball == Ball.FLOWER ? Tunables.nectarPos : Tunables.pollenPos);
                flywheel.update(targetRpm, expansionHub.inputVoltage().value());

                telemetryM.addLine("left bumper: FLOWER, right bumper: POLLEN");
                telemetryM.addData("mode", ball);
                telemetryM.addData("target RPM", targetRpm);
                telemetryM.addData("current RPM", flywheel.getRPM());
                telemetryM.addData("hood pos", hood.commandedPosition());
                telemetryM.update(telemetry);
            }

            flywheel.update(0, expansionHub.inputVoltage().value());
        } finally {
            io.stop(); // flushes the zero-RPM command above
        }
    }
}

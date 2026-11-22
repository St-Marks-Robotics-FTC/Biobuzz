/** shows what a REV Color Sensor V3 reads through the I/O thread, to pick thresholds and check wiring **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.io.ColorSensorDevice;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;

@Configurable
@TeleOp(name="ColorSensorTuner", group="Tuner")
public class ColorSensorTuner extends LinearOpMode {
    public static String sensorName = "colorSensor"; // must match the robot configuration
    public static float gain = 2; // multiplies the normalized colors; read at init

    @Override
    public void runOpMode() {
        IoThread io = new IoThread(hardwareMap);
        ColorSensorDevice sensor = new ColorSensorDevice(io, sensorName, new ColorSensorDevice.Config()
                .gain(gain)
                .colorReads(ReadPeriod.everyCycle())
                .distanceReads(ReadPeriod.everyCycle()));
        io.start();
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        try {
            waitForStart();

            while (opModeIsActive()) {
                io.awaitNewCycle();

                ColorSensorDevice.ColorReading color = sensor.color();
                telemetryM.addData("red", color.red());
                telemetryM.addData("green", color.green());
                telemetryM.addData("blue", color.blue());
                telemetryM.addData("alpha", color.alpha());
                telemetryM.addData("distance (mm)", sensor.distanceMm().value());
                telemetryM.addData("I/O cycle (ms)", io.lastCycleMillis());
                telemetryM.update(telemetry);
            }
        } finally {
            io.stop();
        }
    }
}

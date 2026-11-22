/** the purpose of OpMode is to test flywheel max speed and tune flywheel PIDF **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.subsys.launch.Flywheel;

@TeleOp(name="FlywheelTuner", group="Tuner")
public class FlywheelTuner extends LinearOpMode {
    private enum TuneMode {
        NO_POWER,
        FULL_POWER,
        PIDF
    }

    private static final double MAX_SET_RPM = 6000;

    @Override
    public void runOpMode() {
        IoThread io = new IoThread(hardwareMap);
        Robot robot = new Robot(io);
        Flywheel flywheel = robot.pollenLaunch.flywheel; // tune the pollen launcher
        flywheel.readCurrent(ReadPeriod.everyCycle()); // tuning only: costs the I/O thread a Lynx command per cycle
        io.start();

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        TuneMode mode = TuneMode.NO_POWER;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];
        TuneMode mode2 = TuneMode.values()[2];

        try {
            waitForStart();

            while (opModeIsActive()) {
                io.awaitNewCycle();

                telemetryM.addLine("select your tuning mode using the gamepad:");
                telemetryM.addLine("(A): " + mode0);
                telemetryM.addLine("(B): " + mode1);
                telemetryM.addLine("(X): " + mode2);

                if (gamepad1.aWasReleased()) { mode = mode0; }
                if (gamepad1.bWasReleased()) { mode = mode1; }
                if (gamepad1.xWasReleased()) { mode = mode2; }

                telemetryM.addLine("");
                telemetryM.addLine("current testing mode is: " + mode);

                switch (mode) {
                    case NO_POWER:
                        flywheel.powerUpdate(0.0); // apply no power
                        break;
                    case FULL_POWER:
                        flywheel.powerUpdate(1.0);
                        break;
                    case PIDF:
                        double desiredFlywheelRPM = (MAX_SET_RPM * gamepad1.right_trigger);

                        flywheel.update(desiredFlywheelRPM, robot.getExpansionHubVoltage());

                        telemetryM.addData("desired flywheel RPM", desiredFlywheelRPM);
                        telemetryM.addData("desired flywheel TPS", flywheel.RPMToTPS(desiredFlywheelRPM));
                        break;
                }

                telemetryM.addData("flywheel RPM", flywheel.getRPM());
                telemetryM.addData("flywheel TPS", flywheel.getTPS());

                telemetryM.debug("PIDF: " + Flywheel.FLYWHEEL_P + ", " + Flywheel.FLYWHEEL_I + ", " + Flywheel.FLYWHEEL_D + ", " + Flywheel.FLYWHEEL_F);
                telemetryM.addData("flywheel power", flywheel.getPower());
                telemetryM.addData("flywheel current", flywheel.getCurrent());
                telemetryM.addData("I/O cycle (ms)", io.lastCycleMillis());

                telemetryM.update(telemetry); // update our telemetry
            }
        } finally {
            io.stop();
        }
    }
}

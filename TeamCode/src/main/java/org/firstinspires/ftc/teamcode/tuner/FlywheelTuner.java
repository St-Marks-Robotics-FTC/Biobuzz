/** the purpose of OpMode is to test flywheel max speed and tune flywheel PIDF **/

package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;

@TeleOp(name="FlywheelTuner", group="Tuner")
public class FlywheelTuner extends LinearOpMode {
    private enum TuneMode {
        NO_POWER,
        FULL_POWER,
        PIDF,
        CALIBRATION
    }

    private static final double MAX_SET_RPM = 6000;
    private static final double CALIBRATION_STEP = 0.1;
    private static final double CALIBRATION_SETTLE_MILLIS = 5000; // amount of time to wait for RPM to settle

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        TuneMode mode = TuneMode.NO_POWER;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];
        TuneMode mode2 = TuneMode.values()[2];
        TuneMode mode3 = TuneMode.values()[3];

        double calibrationPower = 0;
        Timer calibrationSettleTimer = new Timer();
        String calibrationString = "";

        waitForStart();

        while (opModeIsActive()) {
            telemetryM.addLine("select your tuning mode using the gamepad:");
            telemetryM.addLine("(A): " + mode0);
            telemetryM.addLine("(B): " + mode1);
            telemetryM.addLine("(X): " + mode2);
            telemetryM.addLine("(Y): " + mode3);

            if (gamepad1.aWasReleased()) { mode = mode0; }
            if (gamepad1.bWasReleased()) { mode = mode1; }
            if (gamepad1.xWasReleased()) { mode = mode2; }
            if (gamepad1.yWasReleased()) {
                // start calibration
                mode = mode3;
                calibrationSettleTimer.reset();
                calibrationPower = 0;
                calibrationString = "";
            }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);

            switch (mode) {
                case NO_POWER:
                    robot.flywheel.powerUpdate(0.0); // apply no power
                    break;
                case FULL_POWER:
                    robot.flywheel.powerUpdate(1.0);
                    break;
                case PIDF:
                    double desiredFlywheelRPM = (MAX_SET_RPM * gamepad1.right_trigger);

                    robot.flywheel.update(desiredFlywheelRPM, robot.getExpansionHubVoltage());

                    telemetryM.addData("desired flywheel RPM", desiredFlywheelRPM);
                    telemetryM.addData("desired flywheel TPS", robot.flywheel.RPMToTPS(desiredFlywheelRPM));
                    break;
                case CALIBRATION:
                    robot.flywheel.powerUpdate(calibrationPower);

                    if (calibrationSettleTimer.milliseconds() > CALIBRATION_SETTLE_MILLIS) {
                        // next step
                        calibrationString = calibrationString + calibrationPower + ", " + robot.flywheel.getRPM() + "\n";
                        calibrationPower += CALIBRATION_STEP;
                        calibrationSettleTimer.reset();
                    }

                    telemetryM.addLine("calibration info (power, RPM)");
                    telemetryM.addLine(calibrationString);

                    break;

            }

            robot.scheduler.update(0); // write flywheel power

            telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
            telemetryM.addData("flywheel TPS", robot.flywheel.getTPS());

            telemetryM.addData("flywheel power", robot.flywheel.getPower());
            telemetryM.addData("flywheel current", robot.flywheel.getCurrent());

            telemetryM.update(telemetry); // update our telemetry

            idle();
        }
    }
}

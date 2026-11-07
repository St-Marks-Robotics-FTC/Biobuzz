package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

@TeleOp(name="ServoTuner", group="Tuner")
public class ServoTuner extends OpMode {

    private enum TuneMode { // order matters for display
        TRANSFER,
        FLOWER,
        UNSELECTED
    }

    private TuneMode mode = TuneMode.UNSELECTED;
    private boolean isLocked = false;
    private double rightStickAmount;

    private Transfer transfer;
    private Flower flower;
    private HWScheduler scheduler;

    private TelemetryManager telemetryM;
    private double JOG_AMOUNT = 0.05;

    @Override
    public void init() {
        scheduler = new HWScheduler(hardwareMap);
        transfer = new Transfer(scheduler);
        flower = new Flower(scheduler);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // set up our Panels telemetry manager

        telemetryM.addLine("init complete");

        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        telemetryM.addLine("select your tuning mode using the gamepad:");
        // display servo test modes in order of TestMode enum
        telemetryM.addLine("(A): " + mode0);
        telemetryM.addLine("(B): " + mode1);

        if (gamepad1.aWasReleased()) { mode = mode0; }
        if (gamepad1.bWasReleased()) { mode = mode1; }

        telemetryM.addLine("current testing mode is: " + mode);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) isLocked = !isLocked;

        if (!isLocked) {
            rightStickAmount = (gamepad1.right_stick_y + 1) / 2; // map from (-1)<->(1) to (0)<->(1)
        } else {
            // jog using d-pad
            if (gamepad1.dpadUpWasPressed()) rightStickAmount += JOG_AMOUNT;
            if (gamepad1.dpadDownWasPressed()) rightStickAmount -= JOG_AMOUNT;
            if (gamepad1.dpadLeftWasPressed()) rightStickAmount -= (JOG_AMOUNT / 2.0);
            if (gamepad1.dpadRightWasPressed()) rightStickAmount += (JOG_AMOUNT / 2.0);

            rightStickAmount = Range.clip(rightStickAmount, 0, 1); // make sure that it stays within acceptable range
        }

        telemetryM.addLine("current tuning mode is: " + mode);
        telemetryM.addLine("you may change the tuning mode by restarting this OpMode");
        telemetryM.addData("isLocked", isLocked);
        telemetryM.addLine("toggle isLocked with right bumper");

        telemetryM.addLine("use the right stick to tune servo position");
        telemetryM.addLine("use the d-pad to jog the position");

        switch (mode) {
            case TRANSFER:
                transfer.setServoRaw(rightStickAmount);
                telemetryM.addData("transfer pos", rightStickAmount);
                break;
            case FLOWER:
                flower.setServoRaw(rightStickAmount);
                telemetryM.addData("flower pos", rightStickAmount);
                break;
            case UNSELECTED:
                throw new RuntimeException("make sure to select a tuning mode before starting!");
        }

        scheduler.update(0); // write servo position

        telemetryM.update(telemetry);
    }
}

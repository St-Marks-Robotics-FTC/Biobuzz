package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = false; // start in field-centric mode

    @Override
    public void init() {
        loopTimer = new Timer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // must come before any telemetryM call

        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);

        follower.setPose(HandoffState.pose);

        telemetryM.debug("init time: " + loopTimer.get(TimeUnit.MILLISECONDS) + "ms");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        loopTimer.reset();

        double slowModeMultiplier = (gamepad1.left_trigger - 1) * -1; // amount to multiply for by slow mode

        if (isRobotCentric) { // robot-centric control
            follower.manual(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier // reduce speed by our turn rate
            );
        } else { // field-centric control
            double flipControl;
            if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
            else flipControl = 1; // red team doesn't need to be flipped

            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y * slowModeMultiplier * flipControl,
                    -gamepad1.left_stick_x * slowModeMultiplier * flipControl,
                    -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier, // reduce speed by our turn rate
                    follower.pose().heading()
            );

            follower.manual(powers);
        }

        follower.update();

        if (Tunables.isDebugging) updateTelemetry();

        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }

    private void updateTelemetry() {
        // odo
        telemetryM.debug("current heading: " + follower.pose().heading());
        telemetryM.addData("odo x", follower.pose().x());
        telemetryM.addData("odo y", follower.pose().y());
    }
}


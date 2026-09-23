package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.teamcode.field.FieldConstants;
import org.firstinspires.ftc.teamcode.field.GoalTargeting;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.PIDF;

import java.util.concurrent.TimeUnit;


public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Follower follower;
    private Timer loopTimer; // measures our control loop time
    private TelemetryManager telemetryM;
    private boolean isRobotCentric = false; // start in field-centric mode

    private PIDF headingPID = new PIDF(1.2, 0.0, 0.08, 0.0); // tune kP/kD on the real robot
    private boolean wasAiming = false;
    private double lastAimTurnPower = 0.0; // for slew-rate limiting (smoothing) the aim turn output

    private static final double AIM_HEADING_DEADBAND_DEG = 1.0; // stop correcting once within this many degrees of the goal
    private static final double AIM_TURN_SLEW_RATE = 0.06; // max change in turn power per loop (smooths/slows the approach)

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
        boolean isAiming = gamepad1.a; // hold a to snap toward nearest in-range goal
        FieldConstants.Goal aimGoal = isAiming ? pickAimGoal(isBlueTeam()) : null;

        if (!wasAiming && isAiming) {
            headingPID.reset();
            lastAimTurnPower = 0.0;
        }
        wasAiming = isAiming;

        if (isRobotCentric) { // robot-centric control
            double forward = -gamepad1.left_stick_y * slowModeMultiplier;
            double strafe = -gamepad1.left_stick_x * slowModeMultiplier;
            double turn = aimGoal != null
                    ? aimTurnPower(aimGoal)
                    : -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier; // reduce speed by our turn rate

            follower.manual(forward, strafe, turn);
        } else { // field-centric contarol
            double flipControl;
            if (isBlueTeam()) flipControl = -1; // blue team needs to be flipped
            else flipControl = 1; // red team doesn't need to be flipped

            double forward = -gamepad1.left_stick_y * slowModeMultiplier * flipControl;
            double strafe = -gamepad1.left_stick_x * slowModeMultiplier * flipControl;
            double turn = aimGoal != null
                    ? aimTurnPower(aimGoal)
                    : -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slowModeMultiplier; // reduce speed by our turn rate

            DrivePowers powers = ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading());
            follower.manual(powers);
        }

        follower.update();
        updateGoalTelemetry();

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

    private void updateGoalTelemetry() {
        telemetryM.addData("--- Blue Goals ---", "");
        for (FieldConstants.Goal g : FieldConstants.BLUE_GOALS) {
            reportGoal(g);
        }

        telemetryM.addData("--- Red Goals ---", "");
        for (FieldConstants.Goal g : FieldConstants.RED_GOALS) {
            reportGoal(g);
        }
    }


    private void reportGoal(FieldConstants.Goal g) {
        double dist = GoalTargeting.distanceTo(follower.pose(), g);
        boolean inRange = GoalTargeting.isInRange(follower.pose(), g);
        boolean inAngle = GoalTargeting.isInAngle(follower.pose(), g);

        telemetryM.addData(g.name + " dist (in)", String.format("%.1f", dist));
        telemetryM.addData(g.name + " can score", (inRange && inAngle) ? "YES" : (inRange ? "angle bad" : "out of range"));
    }

    private FieldConstants.Goal pickAimGoal(boolean blueTeam) {
        FieldConstants.Goal[] goals = blueTeam ? FieldConstants.BLUE_GOALS :
                FieldConstants.RED_GOALS;
        FieldConstants.Goal best = null;

        double bestDist = Double.MAX_VALUE;
        for (FieldConstants.Goal g : goals) {
            if (!GoalTargeting.isInRange(follower.pose(), g)) continue;
            double d = GoalTargeting.distanceTo(follower.pose(), g);
            if (d < bestDist) {
                bestDist = d;
                best = g;
            }
        }

        return best; // null means nothing is in range (button does nothing)
    }

    private double aimTurnPower(FieldConstants.Goal goal) {
        double targetHeading  = GoalTargeting.bearingToGoal(follower.pose(), goal);
        double error = GoalTargeting.wrapAngle(targetHeading - follower.pose().heading());
        double errorDeg = Math.toDegrees(error);

        double turnPower;
        if (Math.abs(errorDeg) <= AIM_HEADING_DEADBAND_DEG) {
            // close enough: stop correcting so we don't chatter around the setpoint
            turnPower = 0.0;
        } else {
            turnPower = headingPID.calc(0, -errorDeg);
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
        }

        // slew-rate limit: smooths the output and slows turning as we approach the target
        // instead of snapping straight to the PID output (which caused the spasming).
        double delta = turnPower - lastAimTurnPower;
        if (delta > AIM_TURN_SLEW_RATE) delta = AIM_TURN_SLEW_RATE;
        if (delta < -AIM_TURN_SLEW_RATE) delta = -AIM_TURN_SLEW_RATE;
        lastAimTurnPower += delta;

        return lastAimTurnPower;
    }
}


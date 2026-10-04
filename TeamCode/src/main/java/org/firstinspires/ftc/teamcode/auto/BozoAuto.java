package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

/**
 * Stationary autonomous: spin up the flywheel, shoot, refuel, shoot again, shut everything off.
 * The robot never drives; the follower only exists to record the start pose for the TeleOp handoff.
 *
 * Sequence: SPIN_UP_FIRST_SHOOT -> FEED_FIRST_SHOOT -> REFUEL -> SPIN_UP_SECOND_SHOOT
 *           -> FEED_SECOND_SHOOT -> DONE
 */
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    protected abstract Pose getStartPose();
    private Robot robot;
    private Follower follower;
    private Timer stateTimer, loopTimer;
    private TelemetryManager telemetryM;
    private Pose startPose;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    /** true once the flywheel is within {@link Tunables#flywheelRPMMargin} of a nonzero target RPM **/
    private boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= Tunables.flywheelRPMMargin;
    }

    private enum State {
        SPIN_UP_FIRST_SHOOT,   // flywheel spinning up; waiting to reach target RPM
        FEED_FIRST_SHOOT,      // transfer open + intake running, feeding the preload through the flywheel
        REFUEL,                // transfer closed, intake running to pick up more balls (flywheel keeps spinning)
        SPIN_UP_SECOND_SHOOT,  // waiting for flywheel to be back at target RPM after refuel
        FEED_SECOND_SHOOT,     // transfer open + intake running, feeding the refueled balls through the flywheel
        DONE                   // everything off; waiting for the OpMode to be stopped
    }

    private State state = State.SPIN_UP_FIRST_SHOOT;

    /**
     * Advances the state machine by one step.
     * @precondition {@link #start()} has run, so stateTimer is reset and targetRPM is set for the first spin-up.
     * @postcondition In DONE the flywheel target is 0, transfer is closed and intake is off.
     */
    private void autoUpdate() {
        switch (state) {
            case SPIN_UP_FIRST_SHOOT:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    startFeeding();
                    setState(State.FEED_FIRST_SHOOT);
                }
                break;
            case FEED_FIRST_SHOOT:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    robot.transfer.close();
                    robot.intake.forward(); // intake stays on to refuel; flywheel stays spinning
                    setState(State.REFUEL);
                }
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.off();
                    setState(State.SPIN_UP_SECOND_SHOOT);
                }
                break;
            case SPIN_UP_SECOND_SHOOT:
                if (isFlywheelWithinMargin()) {
                    startFeeding();
                    setState(State.FEED_SECOND_SHOOT);
                }
                break;
            case FEED_SECOND_SHOOT:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    shutOff();
                    setState(State.DONE);
                }
                break;
            case DONE:
                requestOpModeStop();
                break;
        }
    }

    private void startFeeding() {
        robot.transfer.open();
        robot.intake.forward();
    }

    private void shutOff() {
        robot.transfer.close();
        robot.intake.off();
        targetRPM = 0;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        robot.flywheel.update(targetRPM); // re-run PIDF every loop so RPM actually converges on target
        updateHandoff();
        autoUpdate();
        if (Tunables.isDebugging) {
            sendTelemetry(false);
        }
        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }

    @Override
    public void init() {
        config = buildConfig();
        loopTimer = new Timer();
        stateTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap);
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        startPose = getStartPose();
        config.startPose = startPose;
        follower.setPose(startPose);
        sendTelemetry(true);
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        sendTelemetry(true);
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        targetRPM = Tunables.shootRPM; // begin spinning up immediately
        setState(State.SPIN_UP_FIRST_SHOOT);
    }

    @Override
    public void stop() {
        if (robot != null) {
            shutOff();
            robot.flywheel.update(0);
        }
        updateHandoff();
    }

    public void updateHandoff() {
        if (follower != null) {
            HandoffState.pose = follower.pose();
        }
    }

    public void sendTelemetry(boolean sendInitTime) {
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("Init time (millis): " + loopTimer.get(TimeUnit.SECONDS));
        } else {
            if (targetRPM > 0 && !isFlywheelWithinMargin()) telemetryM.debug("WARNING: FLYWHEEL OUT OF MARGIN");
        }
        telemetryM.debug("State: " + state);
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", targetRPM);
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

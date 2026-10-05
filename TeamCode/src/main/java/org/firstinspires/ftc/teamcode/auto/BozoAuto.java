package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

/**
 * Autonomous: spin up the flywheel, shoot, drive linearly to an intermediate pose and then to the refuel pose
 * to pick up balls, drive back to the start pose, shoot again, shut everything off.
 *
 * Sequence: SPIN_UP_FIRST_SHOOT -> FEED_FIRST_SHOOT -> TRAVEL_TO_INTERMEDIATE -> TRAVEL_TO_REFUEL -> REFUEL
 *           -> TRAVEL_TO_SHOOT -> SPIN_UP_SECOND_SHOOT -> FEED_SECOND_SHOOT -> DONE
 */
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    protected abstract Pose getStartPose();
    private Robot robot;
    private Follower follower;
    private Timer stateTimer, loopTimer, telemetryTimer;
    private TelemetryManager telemetryM;
    private Pose startPose;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    /** true once the flywheel is within {@link Tunables#flywheelRPMMargin} of a nonzero target RPM **/
    private boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= Tunables.flywheelRPMMargin;
    }

    private enum State {
        SPIN_UP_FIRST_SHOOT,   // at the start pose; flywheel spinning up, waiting to reach target RPM
        FEED_FIRST_SHOOT,      // transfer open + intake running, feeding the preload through the flywheel
        TRAVEL_TO_INTERMEDIATE, // driving linearly to the intermediate pose with intake running; flywheel keeps spinning
        TRAVEL_TO_REFUEL,      // driving linearly from the intermediate pose to the refuel pose with intake running
        REFUEL,                // sitting on the refuel pose with intake running to pick up balls
        TRAVEL_TO_SHOOT,       // driving back to the start pose; intake off, flywheel keeps spinning
        SPIN_UP_SECOND_SHOOT,  // back at the start pose; waiting for flywheel to be at target RPM
        FEED_SECOND_SHOOT,     // transfer open + intake running, feeding the refueled balls through the flywheel
        DONE                   // everything off; waiting for the OpMode to be stopped
    }

    private State state = State.SPIN_UP_FIRST_SHOOT;

    /** startPose -> intermediatePose -> refuelPose and back. The shoot pose is the start pose. Built in {@link #init()}. **/
    private Path shootToIntermediatePath, intermediateToRefuelPath, refuelToShootPath;

    private void buildPaths() {
        shootToIntermediatePath = line(startPose, config.intermediatePose).linear(startPose, config.intermediatePose);
        intermediateToRefuelPath = line(config.intermediatePose, config.refuelPose).linear(config.intermediatePose, config.refuelPose)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.3)
                );
        refuelToShootPath = line(config.refuelPose, startPose).linear(config.refuelPose, startPose);
    }

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
                    robot.intake.forward(); // intake on while driving so we're collecting as we arrive
                    follower.follow(shootToIntermediatePath);
                    setState(State.TRAVEL_TO_INTERMEDIATE);
                }
                break;
            case TRAVEL_TO_INTERMEDIATE:
                if (!follower.isBusy()) {
                    follower.follow(intermediateToRefuelPath);
                    setState(State.TRAVEL_TO_REFUEL);
                }
                break;
            case TRAVEL_TO_REFUEL:
                if (!follower.isBusy()) {
                    setState(State.REFUEL);
                }
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.off();
                    follower.follow(refuelToShootPath);
                    setState(State.TRAVEL_TO_SHOOT);
                }
                break;
            case TRAVEL_TO_SHOOT:
                if (!follower.isBusy()) {
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
        robot.intake.forwardLaunching();
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
        telemetryTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap);
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        startPose = getStartPose();
        config.startPose = startPose;
        follower.setPose(startPose);
        buildPaths();
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
        telemetryTimer.reset();
        loopTimer.reset(); // so the first loop() doesn't report init time as loop time
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
        Pose pose = follower.pose(); // read once; each call may allocate
        telemetryM.debug("State: " + state);
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", targetRPM);
        telemetryM.addData("x", pose.x());
        telemetryM.addData("y", pose.y());
        telemetryM.addData("Heading", pose.heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

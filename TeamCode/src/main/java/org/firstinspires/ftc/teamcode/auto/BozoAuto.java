package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

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
        START,                  // waiting for OpMode to begin
        TRAVEL_TO_FIRST_SHOOT,  // driving from the start pose to the shoot pose; flywheel spinning up
        SPIN_UP_FIRST_SHOOT,    // holding at the shoot pose until the flywheel reaches target RPM
        FEED_FIRST_SHOOT,       // transfer open + intake running, feeding the preload through the flywheel
        TRAVEL_TO_REFUEL,       // driving to the refuel line
        REFUEL,                 // intake running while sitting on the refuel line to pick up balls
        TRAVEL_TO_SECOND_SHOOT, // driving back to the shoot pose; flywheel spinning up
        SPIN_UP_SECOND_SHOOT,   // holding at the shoot pose until the flywheel reaches target RPM
        FEED_SECOND_SHOOT,      // transfer open + intake running, feeding the refueled balls through the flywheel
        TRAVEL_TO_END,          // driving to the parking pose
        END                     // done, request OpMode stop
    }

    State state = State.START;

    private Path
            startToShootPath,
            shootToRefuelPath,
            refuelToShootPath,
            shootToEndPath;

    private void buildPaths() {
        startToShootPath = line(startPose, config.shootRightPose).linear(startPose, config.shootRightPose);
        shootToRefuelPath = line(config.shootRightPose, config.refuelPose).linear(config.shootRightPose, config.refuelPose);
        refuelToShootPath = line(config.refuelPose, config.shootRightPose).linear(config.refuelPose, config.shootRightPose);
        shootToEndPath = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
    }

    // Everything is first DO SOMETHING and then MOVE
    private void autoPathUpdate() {
        switch (state) {
            case START:
                follower.follow(startToShootPath);
                targetRPM = Tunables.shootRPM; // spin up while we drive so it's ready when we arrive
                setPathState(State.TRAVEL_TO_FIRST_SHOOT);
                break;
            case TRAVEL_TO_FIRST_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(State.SPIN_UP_FIRST_SHOOT);
                }
                break;
            case SPIN_UP_FIRST_SHOOT:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    robot.transfer.open();
                    robot.intake.forward();
                    setPathState(State.FEED_FIRST_SHOOT);
                }
                break;
            case FEED_FIRST_SHOOT:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    robot.transfer.close();
                    robot.intake.off();
                    targetRPM = 0;
                    follower.follow(shootToRefuelPath);
                    setPathState(State.TRAVEL_TO_REFUEL);
                }
                break;
            case TRAVEL_TO_REFUEL:
                if (!follower.isBusy()) {
                    robot.intake.forward();
                    setPathState(State.REFUEL);
                }
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.off();
                    follower.follow(refuelToShootPath);
                    targetRPM = Tunables.shootRPM; // spin back up on the way back to the shoot pose
                    setPathState(State.TRAVEL_TO_SECOND_SHOOT);
                }
                break;
            case TRAVEL_TO_SECOND_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(State.SPIN_UP_SECOND_SHOOT);
                }
                break;
            case SPIN_UP_SECOND_SHOOT:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    robot.transfer.open();
                    robot.intake.forward();
                    setPathState(State.FEED_SECOND_SHOOT);
                }
                break;
            case FEED_SECOND_SHOOT:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    robot.transfer.close();
                    robot.intake.off();
                    targetRPM = 0;
                    follower.follow(shootToEndPath);
                    setPathState(State.TRAVEL_TO_END);
                }
                break;
            case TRAVEL_TO_END:
                if (!follower.isBusy()) {
                    setPathState(State.END);
                }
                break;
            case END:
                if (!follower.isBusy()) { // End the process
                    requestOpModeStop();
                }
                break;
        }
    }

    private void setPathState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        robot.flywheel.update(targetRPM); // re-run PIDF every loop so RPM actually converges on target
        updateHandoff();
        autoPathUpdate();
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
        telemetryM.debug("Building paths... (this may take a while)");
        telemetryM.update(telemetry);
        buildPaths();
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
        setPathState(State.START);
    }

    @Override
    public void stop() {
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
            if (!isFlywheelWithinMargin()) telemetryM.debug("WARNING: FLYWHEEL OUT OF MARGIN");
        }
        telemetryM.debug("Path state: " + state);
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", targetRPM);
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

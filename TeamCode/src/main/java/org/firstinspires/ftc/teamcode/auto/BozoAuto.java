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

    /** how many times we have driven to the shoot pose and fed our full hopper of balls through the flywheel **/
    private int shotsCompleted = 0;
    private static final int TOTAL_SHOTS = 2; // preload, then one reload from the refuel line

    private enum State {
        START,            // waiting for OpMode to begin
        TRAVEL_TO_SHOOT,  // driving to the shoot pose; flywheel spinning up
        SPIN_UP,          // holding at the shoot pose until the flywheel reaches target RPM
        FEED,             // transfer open + intake running, feeding balls through the flywheel
        TRAVEL_TO_REFUEL, // driving to the refuel line
        REFUEL,           // intake running while sitting on the refuel line to pick up balls
        TRAVEL_TO_END,    // driving to the parking pose
        END               // done, request OpMode stop
    }

    State state = State.START;

    private Path
            path1, // start -> shoot
            path2, // shoot -> refuel
            path3, // refuel -> shoot
            path4; // shoot -> end

    private void buildPaths() {
        path1 = line(startPose, config.shootRightPose).linear(startPose, config.shootRightPose);
        path2 = line(config.shootRightPose, config.refuelPose).linear(config.shootRightPose, config.refuelPose);
        path3 = line(config.refuelPose, config.shootRightPose).linear(config.refuelPose, config.shootRightPose);
        path4 = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
    }

    // Everything is first DO SOMETHING and then MOVE
    private void autoPathUpdate() {
        switch (state) {
            case START:
                follower.follow(path1);
                robot.flywheel.setRPM(Tunables.shootRPM); // spin up while we drive so it's ready when we arrive
                setPathState(State.TRAVEL_TO_SHOOT);
                break;
            case TRAVEL_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(State.SPIN_UP);
                }
                break;
            case SPIN_UP:
                if (robot.flywheel.isWithinMargin()) { // don't feed balls until we're actually at speed
                    robot.transfer.open();
                    robot.intake.forward();
                    setPathState(State.FEED);
                }
                break;
            case FEED:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    robot.transfer.close();
                    robot.intake.off();
                    robot.flywheel.setRPM(0);
                    shotsCompleted++;

                    if (shotsCompleted >= TOTAL_SHOTS) {
                        follower.follow(path4);
                        setPathState(State.TRAVEL_TO_END);
                    } else {
                        follower.follow(path2);
                        setPathState(State.TRAVEL_TO_REFUEL);
                    }
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
                    follower.follow(path3);
                    robot.flywheel.setRPM(Tunables.shootRPM); // spin back up on the way back to the shoot pose
                    setPathState(State.TRAVEL_TO_SHOOT);
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
        robot.flywheel.update();
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
            if (!robot.flywheel.isWithinMargin()) telemetryM.debug("WARNING: FLYWHEEL OUT OF MARGIN");
        }
        telemetryM.debug("Path state: " + state);
        telemetryM.addData("shotsCompleted", shotsCompleted);
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", robot.flywheel.getTargetRPM());
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

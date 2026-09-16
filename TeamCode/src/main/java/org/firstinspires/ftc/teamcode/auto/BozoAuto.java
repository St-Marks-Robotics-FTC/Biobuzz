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
    private State pastState;
    private enum State {
        START,
        SHOOTING,
        GO_TO_LEFT_FLOWER,
        GO_TO_RIGHT_FLOWER,
        FIRST_PICKUP,
        SECOND_PICKUP,
        END
    }

    State state = State.START;

    private Path
            path1,
            path2,
            path3,
            path4,
            path5,
            path6;

    private void buildPaths() {
        path1 = line(startPose, config.shootLeftPose).linear(startPose, config.shootLeftPose);
        path2 = line(config.shootLeftPose, config.flowerLeftPose).linear(config.shootLeftPose, config.flowerLeftPose);
        path3 = line(config.flowerLeftPose, config.shootRightPose).linear(config.flowerLeftPose, config.shootRightPose);
        path4 = line(config.shootRightPose, config.flowerRightPose).linear(config.shootRightPose, config.flowerRightPose);
        path5 = line(config.flowerRightPose, config.shootRightPose).linear(config.flowerRightPose, config.shootRightPose);
        path6 = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
    }
    //Everything is first DO SOMETHING and then MOVE
    private void autoPathUpdate() {
        switch (state) {
            case START:
                follower.follow(path1);
                pastState = state;
                setPathState(State.SHOOTING);
                break;
            case SHOOTING:
                if (!follower.isBusy() && pastState == State.START) {
                    //shoot function
                    follower.follow(path2);
                    pastState = State.SHOOTING;
                    setPathState(State.GO_TO_LEFT_FLOWER);
                } else if (!follower.isBusy() && pastState == State.FIRST_PICKUP) {
                    //shoot function
                    follower.follow(path4);
                    pastState = State.SHOOTING;
                    setPathState(State.GO_TO_RIGHT_FLOWER);
                } else if (!follower.isBusy() && pastState == State.SECOND_PICKUP) {
                    //shoot function
                    follower.follow(path6);
                    pastState = State.SHOOTING;
                    setPathState(State.END);
                }
                break;
            case GO_TO_LEFT_FLOWER:
                if (!follower.isBusy() && pastState == State.SHOOTING) {
                    //Telemetry to know that we are at the flower
                    pastState = State.GO_TO_LEFT_FLOWER;
                    setPathState(State.FIRST_PICKUP);
                }
                break;
            case FIRST_PICKUP:
                if (!follower.isBusy()) {
                    //Intake the nectar
                    follower.follow(path3);
                    pastState = State.FIRST_PICKUP;
                    setPathState(State.SHOOTING);
                }
                break;
            case SECOND_PICKUP:
                if (!follower.isBusy()) {
                    //Intake the nectar
                    pastState = State.SECOND_PICKUP;
                    follower.follow(path5);
                    setPathState(State.SHOOTING);
                }
                break;
            case GO_TO_RIGHT_FLOWER:
                if (!follower.isBusy()) {
                    //Telemetry: "At the flower on the right"
                    setPathState(State.SECOND_PICKUP);
                }
                break;
            case END:
                if (!follower.isBusy()) { //End the process
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
        HandoffState.pose = follower.pose();
    }

    public void sendTelemetry(boolean sendInitTime) {
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("Init time (millis): " + loopTimer.get(TimeUnit.SECONDS));
        }
        telemetryM.debug("Path state: " + state);
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

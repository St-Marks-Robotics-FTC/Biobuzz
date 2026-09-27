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
        REFUELING,
        END
    }

    State state = State.START;

    private Path
            path1,
            path2,
            path3,
            path4;

    private void buildPaths() {
        path1 = line(startPose, config.shootRightPose).linear(startPose, config.shootRightPose);
        path2 = line(config.shootRightPose, config.refuelPose).linear(config.shootRightPose, config.refuelPose);
        path3 = line(config.refuelPose, config.shootRightPose).linear(config.refuelPose, config.shootRightPose);
        path4 = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
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
                    setPathState(State.REFUELING);
                } else if (!follower.isBusy() && pastState == State.REFUELING) {
                    //shoot function
                    follower.follow(path4);
                    pastState = State.SHOOTING;
                    setPathState(State.END);
                }
                break;
            case REFUELING:
                if (!follower.isBusy()) {
                    //Intake the nectar
                    follower.follow(path3);
                    pastState = State.REFUELING;
                    setPathState(State.SHOOTING);
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
        }
        telemetryM.debug("Path state: " + state);
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}
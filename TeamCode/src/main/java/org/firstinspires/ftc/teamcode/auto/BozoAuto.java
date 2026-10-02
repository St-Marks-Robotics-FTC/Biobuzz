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
    private double initMs;

    // Every DRIVE_* state waits for the follower to settle; every action state then dwells at that pose.
    private enum State {
        DRIVE_TO_SHOOT_LEFT,
        SHOOT_PRELOAD,
        DRIVE_TO_LEFT_FLOWER,
        LEFT_PICKUP,
        DRIVE_TO_SHOOT_RIGHT,
        SHOOT_LEFT_FLOWER,
        DRIVE_TO_RIGHT_FLOWER,
        RIGHT_PICKUP,
        DRIVE_BACK_TO_SHOOT_RIGHT,
        SHOOT_RIGHT_FLOWER,
        PARK,
        DONE
    }
    State state = State.DRIVE_TO_SHOOT_LEFT;
    private Path path1, path2, path3, path4, path5, path6;

    private void buildPaths() {
        path1 = line(startPose, config.shootLeftPose).linear(startPose, config.shootLeftPose);
        path2 = line(config.shootLeftPose, config.flowerLeftPose).linear(config.shootLeftPose, config.flowerLeftPose);
        path3 = line(config.flowerLeftPose, config.shootRightPose).linear(config.flowerLeftPose, config.shootRightPose);
        path4 = line(config.shootRightPose, config.flowerRightPose).linear(config.shootRightPose, config.flowerRightPose);
        path5 = line(config.flowerRightPose, config.shootRightPose).linear(config.flowerRightPose, config.shootRightPose);
        path6 = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
    }

    private boolean arrived() {
        return !follower.isBusy() || stateTimer.get(TimeUnit.MILLISECONDS) > Tunables.pathTimeoutMs;
    }

    private boolean dwelled(long dwellMs) {
        return stateTimer.get(TimeUnit.MILLISECONDS) >= dwellMs;
    }

    private void followThen(Path path, State next) {
        follower.follow(path);
        setPathState(next);
    }

    private void autoPathUpdate() {
        switch (state) {
            case DRIVE_TO_SHOOT_LEFT:
                if (arrived()) setPathState(State.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (dwelled(Tunables.shootTimeMs)) followThen(path2, State.DRIVE_TO_LEFT_FLOWER);
                break;
            case DRIVE_TO_LEFT_FLOWER:
                if (arrived()) setPathState(State.LEFT_PICKUP);
                break;
            case LEFT_PICKUP:
                if (dwelled(Tunables.intakeTimeMs)) followThen(path3, State.DRIVE_TO_SHOOT_RIGHT);
                break;
            case DRIVE_TO_SHOOT_RIGHT:
                if (arrived()) setPathState(State.SHOOT_LEFT_FLOWER);
                break;
            case SHOOT_LEFT_FLOWER:
                if (dwelled(Tunables.shootTimeMs)) followThen(path4, State.DRIVE_TO_RIGHT_FLOWER);
                break;
            case DRIVE_TO_RIGHT_FLOWER:
                if (arrived()) setPathState(State.RIGHT_PICKUP);
                break;
            case RIGHT_PICKUP:
                if (dwelled(Tunables.intakeTimeMs)) followThen(path5, State.DRIVE_BACK_TO_SHOOT_RIGHT);
                break;
            case DRIVE_BACK_TO_SHOOT_RIGHT:
                if (arrived()) setPathState(State.SHOOT_RIGHT_FLOWER);
                break;
            case SHOOT_RIGHT_FLOWER:
                if (dwelled(Tunables.shootTimeMs)) followThen(path6, State.PARK);
                break;
            case PARK:
                if (arrived()) {
                    setPathState(State.DONE);
                    requestOpModeStop();
                }
                break;
            case DONE:
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
        telemetryM.debug("Creating follower");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        startPose = getStartPose();
        telemetryM.debug("Building paths");
        telemetryM.update(telemetry);
        buildPaths();
        follower.setPose(startPose);
        initMs = loopTimer.get(TimeUnit.MILLISECONDS);
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
        followThen(path1, State.DRIVE_TO_SHOOT_LEFT);
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
            telemetryM.addLine("INIT COMPLETE");
            telemetryM.debug("Init " + initMs);
        }
        telemetryM.debug("State " + state);
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("Heading", follower.pose().heading());
        telemetryM.debug("Time " + getRuntime());
    }
}

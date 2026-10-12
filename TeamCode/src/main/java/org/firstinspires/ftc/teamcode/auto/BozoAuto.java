package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;

public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    private Robot robot;
    private Follower follower;
    private Timer stateTimer, loopTimer;
    private Vision vision;
    private TelemetryManager telemetryM;
    private Pose startPose;
    private double initMs;

    private enum State {
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        DRIVE_TO_GARDEN,
        GARDEN_PICKUP,
        DRIVE_TO_SHOOT_GARDEN,
        SHOOT_GARDEN,
        PARK,
        DONE
    }
    State state = State.DRIVE_TO_SHOOT_PRELOAD;
    private boolean launched;
    private Path path1, path2, path3, path4;


    private Path bezier(Pose start, Pose[] controls, Pose end) {
        if (controls.length == 0) return line(start, end);
        Pose[] points = new Pose[controls.length + 2];
        points[0] = start;
        System.arraycopy(controls, 0, points, 1, controls.length);
        points[points.length - 1] = end;
        return curve(points);
    }

    private void buildPaths() {
        path1 = bezier(startPose, config.shootPreloadControls, config.shootPreloadPose).linear(startPose, config.shootPreloadPose);
        Path garden = bezier(config.shootPreloadPose, config.gardenControls, config.gardenPose);
        if (config.gardenTangent) {
            path2 = garden.tangent();
        } else {
            Pose gardenTilt = config.gardenPose.withHeading(config.gardenTiltRad);
            path2 = garden.heading(Interpolator.piecewise()
                    .until(0.5, Interpolator.linear(config.shootPreloadPose, gardenTilt))
                    .until(0.75, Interpolator.constant(gardenTilt))
                    .until(1, Interpolator.linear(gardenTilt, config.gardenPose)));
        }
        path3 = line(config.gardenPose, config.shootGardenPose).linear(config.gardenPose, config.shootGardenPose);
        path4 = bezier(config.shootGardenPose, config.endControls, config.endPose).linear(config.shootGardenPose, config.endPose);
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

    private boolean shot() {
        if (!launched && (flywheelReady() || stateTimer.get(TimeUnit.MILLISECONDS) > Tunables.shootSpinUpTimeoutMs)) {
            robot.startLaunch();
            launched = true;
        }
        return launched && !robot.isLaunching();
    }

    private boolean flywheelReady() {
        return Math.abs(Math.abs(robot.flywheel.getRPM()) - Tunables.autoShootRpm) <= Tunables.shooterToleranceRpm;
    }

    private void autoPathUpdate() {
        switch (state) {
            case DRIVE_TO_SHOOT_PRELOAD:
                if (arrived()) setPathState(State.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (shot()) followThen(path2, State.DRIVE_TO_GARDEN);
                break;
            case DRIVE_TO_GARDEN: // intake is running, so balls swept into the corner are picked up on the way
                if (arrived()) setPathState(State.GARDEN_PICKUP);
                break;
            case GARDEN_PICKUP: // wait while the intake collects
                if (dwelled(Tunables.intakeTimeMs)) followThen(path3, State.DRIVE_TO_SHOOT_GARDEN);
                break;
            case DRIVE_TO_SHOOT_GARDEN:
                if (arrived()) setPathState(State.SHOOT_GARDEN);
                break;
            case SHOOT_GARDEN:
                if (shot()) {
                    robot.intake.off();
                    followThen(path4, State.PARK);
                }
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
        launched = false;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        updateHandoff();
        // flywheel stays spun up
        robot.flywheel.update(state == State.PARK || state == State.DONE ? 0 : Tunables.autoShootRpm);
        robot.updateLaunch(); // ends a volley
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
        startPose = config.startPose;
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
        robot.intake.forward();
        followThen(path1, State.DRIVE_TO_SHOOT_PRELOAD);
    }

    @Override
    public void stop() {
        robot.flywheel.update(0);
        robot.intake.off();
        updateHandoff();
    }

    public void updateHandoff() {
        HandoffState.pose = follower.pose();
        HandoffState.fromAuto = true;
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
        telemetryM.addData("Flywheel RPM", robot.flywheel.getRPM());
        telemetryM.debug("Time " + getRuntime());
    }
}

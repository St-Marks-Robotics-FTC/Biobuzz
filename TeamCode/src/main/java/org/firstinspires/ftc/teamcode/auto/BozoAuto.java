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
        SHOOT_PRELOAD,
        TIP_SPIN,
        DRIVING_TO_GARDEN,
        GARDEN_PICKUP,
        DRIVING_TO_SHOOT_GARDEN_BALLS,
        SHOOT_GARDEN_BALLS,
        DRIVING_TO_CLOSE_FLOWER,
        CLOSE_FLOWER_PICKUP,
        DRIVING_TO_SHOOT_CLOSE_FLOWER_BALLS,
        SHOOT_CLOSE_FLOWER_BALLS,
        DRIVING_TO_FAR_FLOWER,
        FAR_FLOWER_PICKUP,
        DRIVING_TO_SHOOT_FAR_FLOWER_BALLS,
        SHOOT_FAR_FLOWER_BALLS,
        DRIVING_TO_PARK,
        DONE
    }
    State state = State.SHOOT_PRELOAD;
    private boolean launched;
    private Vision.HiveState hiveBefore = Vision.HiveState.UNKNOWN; // hive state when the preload shot began
    private long waitedMs; // rev-up, shot, intake and tip-spin time already spent
    private int shotsLeft, intakesLeft; // fixed waits still to come, for the budget
    private Path pathToGarden, pathToShootGarden, pathToCloseFlower, pathToSecondShot, pathToFarFlower, pathToLastShot, pathPark;


    private Path bezier(Pose start, Pose[] controls, Pose end) {
        if (controls.length == 0) return line(start, end);
        Pose[] points = new Pose[controls.length + 2];
        points[0] = start;
        System.arraycopy(controls, 0, points, 1, controls.length);
        points[points.length - 1] = end;
        return curve(points);
    }

    private void buildPaths() {
        pathToGarden = tiltedHeading(bezier(config.shootPreloadPose, config.gardenControls, config.gardenPose), config.shootPreloadPose, config.gardenTiltRad, config.gardenPose);
        pathToShootGarden = bezier(config.gardenPose, config.shootGardenControls, config.shootGardenPose).linear(config.gardenPose, config.shootGardenPose);
        if (config.closeFlowerPose == null) {
            pathPark = park(config.shootGardenPose);
        } else {
            pathToCloseFlower = turnThenHold(bezier(config.shootGardenPose, config.closeFlowerControls, config.closeFlowerPose), config.shootGardenPose, config.closeFlowerPose);
            pathToSecondShot = line(config.closeFlowerPose, config.secondShotPose).linear(config.closeFlowerPose, config.secondShotPose);
            pathToFarFlower = turnThenHold(bezier(config.secondShotPose, config.farFlowerControls, config.farFlowerPose), config.secondShotPose, config.farFlowerPose);
            pathToLastShot = bezier(config.farFlowerPose, config.lastShotControls, config.lastShotPose).heading(Interpolator.piecewise()
                    .until(0.5, Interpolator.constant(config.farFlowerPose))
                    .until(1, Interpolator.linear(config.farFlowerPose, config.lastShotPose)));
            pathPark = park(config.lastShotPose);
        }
    }

    private Path park(Pose from) {
        return config.closeFlowerPose == null
                ? bezier(from, config.endControls, config.endPose).linear(from, config.endPose)
                : bezier(from, config.endControls, config.endPose).tangent();
    }
    private Path tiltedHeading(Path path, Pose from, double tiltRad, Pose to) {
        Pose tilt = to.withHeading(tiltRad);
        return path.heading(Interpolator.piecewise()
                .until(0.5, Interpolator.linear(from, tilt))
                .until(0.75, Interpolator.constant(tilt))
                .until(1, Interpolator.linear(tilt, to)));
    }

    private Path turnThenHold(Path path, Pose from, Pose to) { // heading finishes turning 70% of the way along, then holds
        return path.heading(Interpolator.piecewise()
                .until(0.7, Interpolator.linear(from, to))
                .until(1, Interpolator.constant(to)));
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

    private long stateMs() {
        return (long) stateTimer.get(TimeUnit.MILLISECONDS);
    }

    private long fixedWaitsLeftMs(int shots) { // launch sequence and intake dwells still to come; none of it can be shortened
        long launchMs = (long) (Tunables.launchIntakeStopTime + Tunables.launchTransferOpeningTime + Tunables.launchShootingTime);
        return shots * launchMs + intakesLeft * Tunables.intakeTimeMs;
    }

    // budget left for this shot's flywheel spin-up once every later wait is reserved at its fixed length
    private long spinUpCapMs() {
        return Math.max(0, Tunables.autoWaitBudgetMs - waitedMs - fixedWaitsLeftMs(shotsLeft));
    }

    // budget left after the shot in progress finishes and every later wait is reserved
    private long tipPoolMs() {
        return Tunables.autoWaitBudgetMs - waitedMs - stateMs() - fixedWaitsLeftMs(shotsLeft - 1);
    }

    // the hive tipped if Vision confirmed a different state than before the preload shot; no baseline counts as not tipped
    private boolean hiveTipped() {
        Vision.HiveState now = vision.getLastHiveState();
        return hiveBefore != Vision.HiveState.UNKNOWN && now != Vision.HiveState.UNKNOWN && now != hiveBefore;
    }

    private boolean shot() {
        if (!launched && (flywheelReady() || stateMs() >= spinUpCapMs())) {
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
            case SHOOT_PRELOAD:
                if (shot()) {
                    if (!hiveTipped() && tipPoolMs() >= Tunables.tipSpinMs) {
                        setPathState(State.TIP_SPIN);
                    } else {
                        followThen(pathToGarden, State.DRIVING_TO_GARDEN);
                    }
                }
                break;
            case TIP_SPIN: // the preload shot did not tip the hive: spin in place to shake it over
                Pose here = follower.pose();
                follower.hold(here.withHeading(here.heading() + Math.PI / 2));
                for (int i = 0; i < 20; i++) telemetryM.addLine("f*ck");
                requestOpModeStop();
                break;
            case DRIVING_TO_GARDEN: // intake is running, so balls swept into the corner are picked up on the way
                if (arrived()) setPathState(State.GARDEN_PICKUP);
                break;
            case GARDEN_PICKUP: // wait while the intake collects
                if (dwelled(Tunables.intakeTimeMs)) {
                    if (config.closeFlowerPose != null) robot.flower.goDown(); // arms drop on the way to the garden shot so they are well down before the close flowers
                    followThen(pathToShootGarden, State.DRIVING_TO_SHOOT_GARDEN_BALLS);
                }
                break;
            case DRIVING_TO_SHOOT_GARDEN_BALLS:
                if (arrived()) setPathState(State.SHOOT_GARDEN_BALLS);
                break;
            case SHOOT_GARDEN_BALLS:
                if (shot()) {
                    if (config.closeFlowerPose == null) {
                        robot.intake.off();
                        followThen(pathPark, State.DRIVING_TO_PARK);
                    } else {
                        followThen(pathToCloseFlower, State.DRIVING_TO_CLOSE_FLOWER);
                    }
                }
                break;
            case DRIVING_TO_CLOSE_FLOWER:
                if (arrived()) setPathState(State.CLOSE_FLOWER_PICKUP);
                break;
            case CLOSE_FLOWER_PICKUP: // wait while the intake collects
                if (dwelled(Tunables.intakeTimeMs)) followThen(pathToSecondShot, State.DRIVING_TO_SHOOT_CLOSE_FLOWER_BALLS);
                break;
            case DRIVING_TO_SHOOT_CLOSE_FLOWER_BALLS:
                if (arrived()) setPathState(State.SHOOT_CLOSE_FLOWER_BALLS);
                break;
            case SHOOT_CLOSE_FLOWER_BALLS:
                if (shot()) followThen(pathToFarFlower, State.DRIVING_TO_FAR_FLOWER);
                break;
            case DRIVING_TO_FAR_FLOWER:
                if (arrived()) setPathState(State.FAR_FLOWER_PICKUP);
                break;
            case FAR_FLOWER_PICKUP: // wait while the intake collects
                if (dwelled(Tunables.intakeTimeMs)) followThen(pathToLastShot, State.DRIVING_TO_SHOOT_FAR_FLOWER_BALLS);
                break;
            case DRIVING_TO_SHOOT_FAR_FLOWER_BALLS:
                if (arrived()) setPathState(State.SHOOT_FAR_FLOWER_BALLS);
                break;
            case SHOOT_FAR_FLOWER_BALLS:
                if (shot()) {
                    robot.intake.off();
                    if (config.closeFlowerPose != null) robot.flower.goUp();
                    followThen(pathPark, State.DRIVING_TO_PARK);
                }
                break;
            case DRIVING_TO_PARK:
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
        switch (state) { // waits spent in the state we are leaving come out of the shared budget
            case SHOOT_PRELOAD:
            case SHOOT_GARDEN_BALLS:
            case SHOOT_CLOSE_FLOWER_BALLS:
            case SHOOT_FAR_FLOWER_BALLS:
                waitedMs += stateMs();
                shotsLeft--;
                break;
            case GARDEN_PICKUP:
            case CLOSE_FLOWER_PICKUP:
            case FAR_FLOWER_PICKUP:
                waitedMs += stateMs();
                intakesLeft--;
                break;
            case TIP_SPIN:
                waitedMs += stateMs();
                break;
            default:
                break;
        }
        if (newState == State.SHOOT_PRELOAD) hiveBefore = vision.getLastHiveState();
        state = newState;
        launched = false;
        stateTimer.reset();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        follower.update();
        vision.update();
        updateHandoff();
        // flywheel stays spun up
        robot.flywheel.update(state == State.DRIVING_TO_PARK || state == State.DONE ? 0 : Tunables.autoShootRpm);
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
        vision = new Vision(hardwareMap, config.isBlue);
        shotsLeft = config.closeFlowerPose == null ? 2 : 4;
        intakesLeft = config.closeFlowerPose == null ? 1 : 3;
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
        telemetryM.debug("State: " + state);
        telemetryM.addData("x: ", follower.pose().x());
        telemetryM.addData("y: ", follower.pose().y());
        telemetryM.addData("Heading: ", follower.pose().heading());
        telemetryM.addData("Flywheel RPM: ", robot.flywheel.getRPM());
        telemetryM.debug("Time: " + getRuntime());
    }
}

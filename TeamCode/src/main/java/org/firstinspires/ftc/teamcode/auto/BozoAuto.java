package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.concurrent.TimeUnit;

/**
 * Autonomous: start at the start pose, drive linearly to the shoot pose (flywheel spinning up on the way), shoot,
 * drive linearly to an intermediate pose and then to the refuel pose to pick up balls, drive back to the shoot pose,
 * then (hive tipped) curve to the scoring side, shoot again, and drive to the park pose. If the hive did not tip,
 * the auto reports a failure in telemetry (TEMP_FAILED) and stops.
 *
 * Sequence: TRAVEL_TO_FIRST_SHOOT -> SPIN_UP_FIRST_SHOOT -> FEED_FIRST_SHOOT -> TRAVEL_TO_INTERMEDIATE
 *           -> TRAVEL_TO_REFUEL -> REFUEL -> TRAVEL_TO_SHOOT
 *           -> TIPPED_TRAVEL_TO_TIPPED_SHOOT -> TIPPED_SPEED_UP_SECOND_SHOOT -> TIPPED_FEED_SECOND_SHOOT
 *           -> TRAVEL_TO_PARK -> DONE
 *           (not tipped: TRAVEL_TO_SHOOT -> TEMP_FAILED -> DONE)
 */
public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();

    /** @return true if this OpMode runs on the blue alliance, false for red; selects the Limelight pipeline. **/
    protected abstract boolean isBlueTeam();
    private Robot robot;
    private Vision vision;
    private Follower follower;
    private Timer stateTimer, loopTimer, telemetryTimer;
    private TelemetryManager telemetryM;
    private Pose startPose, shootPose;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    /** true once the flywheel is within {@link Tunables#flywheelRPMMargin} of a nonzero target RPM **/
    private boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= Tunables.flywheelRPMMargin;
    }

    private enum State {
        TRAVEL_TO_FIRST_SHOOT, // driving linearly from the start pose to the shoot pose; flywheel already spinning up
        SPIN_UP_FIRST_SHOOT,   // at the shoot pose; waiting for flywheel to reach target RPM
        FEED_FIRST_SHOOT,      // Robot launch feeds the preloaded balls through the flywheel
        TRAVEL_TO_INTERMEDIATE, // driving linearly to the intermediate pose with intake running; flywheel keeps spinning
        TRAVEL_TO_REFUEL,      // driving linearly from the intermediate pose to the refuel pose with intake running
        REFUEL,                // sitting on the refuel pose with intake running to pick up balls
        TRAVEL_TO_SHOOT,       // driving back to the shoot pose; intake running, flywheel keeps spinning

        // hive tipped, go to the scoring side and shoot the second round there
        TIPPED_TRAVEL_TO_TIPPED_SHOOT, // go to scoring side to shoot second round
        TIPPED_SPEED_UP_SECOND_SHOOT, // arrived, wait for flywheel to speed up
        TIPPED_FEED_SECOND_SHOOT, // Robot launch sequence...

        TRAVEL_TO_PARK,        // everything off; driving a straight line from the tipped shoot pose to the park pose
        DONE,                  // parked; waiting for the OpMode to be stopped

        TEMP_FAILED            // TEMPORARY: hive did not flip; report failure, then DONE
    }

    private State state = State.TRAVEL_TO_FIRST_SHOOT;

    /** startPose -> shootPose, then shootPose -> intermediatePose -> refuelPose and back. Built in {@link #init()}. **/
    private Path startToShootPath, shootToIntermediatePath, intermediateToRefuelPath, refuelToShootPath;

    /** shootPose -> scoring side (bent by {@link AutoConfig#tippedControlPose1} and {@link AutoConfig#tippedControlPose2}), then a straight line to park. Built in {@link #init()}. **/
    private Path shootToTippedShootPath, tippedShootToParkPath;

    private static final double REFUEL_ROTATE_START = 0.7;

    private void buildPaths() {
        startToShootPath = line(startPose, shootPose).linear(startPose, shootPose);
        shootToIntermediatePath = line(shootPose, config.intermediatePose).linear(shootPose, config.intermediatePose);
        intermediateToRefuelPath = line(config.intermediatePose, config.refuelPose)
                .heading(Interpolator.piecewise()
                        .until(REFUEL_ROTATE_START, Interpolator.constant(config.intermediatePose))
                        .until(1.0, Interpolator.linear(config.intermediatePose, config.refuelPose)))
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.35)
                );
        refuelToShootPath = line(config.refuelPose, shootPose).linear(config.refuelPose, shootPose);
        // curve(start, control, end): the control point is not driven through, it only pulls the path toward it.
        // Toward the scoring side (bent by the two control poses), then a line to park.
        shootToTippedShootPath = curve(shootPose, config.tippedControlPose1, config.tippedControlPose2, config.tippedShootPose)
                .linear(shootPose, config.tippedShootPose);
        tippedShootToParkPath = line(config.tippedShootPose, config.parkPose)
                .linear(config.tippedShootPose, config.parkPose);
    }

    private void autoUpdate() {
        switch (state) {
            case TRAVEL_TO_FIRST_SHOOT:
                if (!follower.isBusy()) {
                    setState(State.SPIN_UP_FIRST_SHOOT);
                }
                break;
            case SPIN_UP_FIRST_SHOOT:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    robot.startLaunch();
                    setState(State.FEED_FIRST_SHOOT);
                }
                break;
            case FEED_FIRST_SHOOT:
                if (!robot.isLaunching()) {
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
                if (!follower.isBusy()
                        // make sure it times out after getting stuck on a ball
                        || stateTimer.milliseconds() >= Tunables.maxTravelToRefuelTime) {
                    setState(State.REFUEL);
                }
                break;
            case REFUEL:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    robot.intake.forward();
                    follower.follow(refuelToShootPath);
                    setState(State.TRAVEL_TO_SHOOT);
                }
                break;
            case TRAVEL_TO_SHOOT:
                if (!follower.isBusy()) {
                    if (Vision.limelight.getLatestResult().getBotposeTagCount() > Tunables.tagThreshold) {
                        // hive did not flip; that case no longer exists, so this is a failure
                        shutOff();
                        setState(State.TEMP_FAILED);
                    } else {
                        follower.follow(shootToTippedShootPath);
                        setState(State.TIPPED_TRAVEL_TO_TIPPED_SHOOT);
                    }
                }
                break;

            // hive tipped, curve to the scoring side and shoot there
            case TIPPED_TRAVEL_TO_TIPPED_SHOOT:
                if (!follower.isBusy()) {
                    setState(State.TIPPED_SPEED_UP_SECOND_SHOOT);
                }
                break;
            case TIPPED_SPEED_UP_SECOND_SHOOT:
                if (isFlywheelWithinMargin()) {
                    robot.startLaunch();
                    setState(State.TIPPED_FEED_SECOND_SHOOT);
                }
                break;
            case TIPPED_FEED_SECOND_SHOOT:
                if (!robot.isLaunching()) {
                    shutOff();
                    follower.follow(tippedShootToParkPath);
                    setState(State.TRAVEL_TO_PARK);
                }
                break;

            case TRAVEL_TO_PARK:
                if (!follower.isBusy()) {
                    setState(State.DONE);
                }
                break;
            case DONE:
                requestOpModeStop();
                break;
            case TEMP_FAILED:
                telemetryM.debug("FAILED: im a failure");
                setState(State.DONE);
                break;
        }
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
        vision.update();
        follower.update();
        robot.flywheel.update(targetRPM); // re-run PIDF every loop so RPM actually converges on target also dih
        robot.update(); // advances the shooting state machine and intake power-save
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
        vision = new Vision(hardwareMap, isBlueTeam());
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        startPose = config.startPose;
        shootPose = config.shootPose;
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
        follower.follow(startToShootPath);
        setState(State.TRAVEL_TO_FIRST_SHOOT);
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
        telemetryM.addData("tags detected", vision.getLatestResult().getBotposeTagCount());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class BozoAuto extends OpMode {
    protected AutoConfig config;
    protected abstract AutoConfig buildConfig();
    private Robot robot;
    private Follower follower;
    private Timer stateTimer, loopTimer, telemetryTimer;
    private TelemetryManager telemetryM;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    private boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= Tunables.flywheelRPMMargin;
    }

    /** what the robot does when a leg ends **/
    private enum Arrival { SHOOT, PICKUP, PARK }

    private static final class Leg {
        final Path path;
        final Arrival arrival;

        Leg(Path path, Arrival arrival) {
            this.path = path;
            this.arrival = arrival;
        }
    }

    private enum State {
        DRIVE,   // following legs.get(legIndex); intake running unless the leg parks
        SPIN_UP, // at a shoot stop; waiting for the flywheel to reach target RPM
        FEED,    // transfer open + intake running, feeding balls through the flywheel
        PICKUP,  // at a pickup stop; intake running for refuelDurationMillis
        DONE     // parked; waiting for the OpMode to be stopped
    }

    private State state = State.DRIVE;


    private List<Leg> legs;
    private int legIndex;

    private static Pose[] controlPoints(Pose start, Pose[] controls, Pose end) {
        Pose[] points = new Pose[controls.length + 2];
        points[0] = start;
        System.arraycopy(controls, 0, points, 1, controls.length);
        points[points.length - 1] = end;
        return points;
    }


    private static Path bezier(Pose start, Pose[] controls, Pose end) {
        if (controls.length == 0) return line(start, end).linear(start, end);
        return curve(controlPoints(start, controls, end)).linear(start, end);
    }

    private void buildPaths() {
        legs = new ArrayList<>();
        legs.add(new Leg(bezier(config.startPose, config.shootPreloadControls, config.shootPreloadPose), Arrival.SHOOT));

        // heading turns toward the tilt over the first half of the garden path, then holds it through the pickup
        Pose gardenTilt = config.gardenPose.withHeading(config.gardenTiltRad);
        Path garden = curve(controlPoints(config.shootPreloadPose, config.gardenControls, config.gardenPose))
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(config.shootPreloadPose, gardenTilt))
                        .until(1.0, Interpolator.constant(gardenTilt)))
                .with(Constants.foresightConfig.maxPathSpeed.at(0.35));
        legs.add(new Leg(garden, Arrival.PICKUP));

        // starts from the tilted garden pose so the heading carries through from the pickup
        legs.add(new Leg(bezier(gardenTilt, config.shootGardenControls, config.shootGardenPose), Arrival.SHOOT));

        Pose finalShotPose = config.shootGardenPose;
        if (config.closeFlowerPose != null) {
            legs.add(new Leg(bezier(config.shootGardenPose, config.closeFlowerControls, config.closeFlowerPose), Arrival.PICKUP));
            legs.add(new Leg(bezier(config.closeFlowerPose, new com.pedropathing.math.Pose[]{}, config.secondShotPose), Arrival.SHOOT));
            legs.add(new Leg(bezier(config.secondShotPose, config.farFlowerControls, config.farFlowerPose), Arrival.PICKUP));
            legs.add(new Leg(bezier(config.farFlowerPose, config.lastShotControls, config.lastShotPose), Arrival.SHOOT));
            finalShotPose = config.lastShotPose;
        }
        legs.add(new Leg(bezier(finalShotPose, config.endControls, config.endPose), Arrival.PARK));
    }

    /** the current leg is done driving; pickup legs also end on timeout in case the robot gets stuck on a ball **/
    private boolean atLegEnd() {
        if (!follower.isBusy()) return true;
        return legs.get(legIndex).arrival == Arrival.PICKUP
                && stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.maxTravelToRefuelTime;
    }

    private void advanceLeg() {
        legIndex++;
        Leg next = legs.get(legIndex);
        if (next.arrival == Arrival.PARK) {
            shutOff(); // last shot is done: flywheel and intake off for the drive to park
        } else {
            robot.intake.forward(); // intake on while driving so we're collecting as we arrive
        }
        follower.follow(next.path);
        setState(State.DRIVE);
    }

    private void autoUpdate() {
        switch (state) {
            case DRIVE:
                if (atLegEnd()) {
                    switch (legs.get(legIndex).arrival) {
                        case SHOOT:
                            setState(State.SPIN_UP);
                            break;
                        case PICKUP:
                            setState(State.PICKUP);
                            break;
                        case PARK:
                            setState(State.DONE);
                            break;
                    }
                }
                break;
            case SPIN_UP:
                if (isFlywheelWithinMargin()) { // don't feed balls until we're actually at speed
                    startShooting();
                    setState(State.FEED);
                }
                break;
            case FEED:
                // the if statement below represents AFTER we've fed the balls
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis) {
                    robot.transfer.close();
                    advanceLeg();
                }
                break;
            case PICKUP:
                if (stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.refuelDurationMillis) {
                    advanceLeg();
                }
                break;
            case DONE:
                requestOpModeStop();
                break;
        }
    }

    private void startShooting() {
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
        robot.flywheel.update(targetRPM, robot.getExpansionHubVoltage()); // re-run PIDF every loop so RPM actually converges on target
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
        follower.setPose(config.startPose);
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
        legIndex = 0;
        follower.follow(legs.get(0).path);
        setState(State.DRIVE);
    }

    @Override
    public void stop() {
        if (robot != null) {
            shutOff();
            robot.flywheel.update(0, robot.getExpansionHubVoltage());
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
        telemetryM.addData("leg", legIndex + " / " + (legs == null ? 0 : legs.size()));
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", targetRPM);
        telemetryM.addData("x", pose.x());
        telemetryM.addData("y", pose.y());
        telemetryM.addData("Heading", pose.heading());
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.HandoffState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.auto.core.Sequencer;
import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.core.Steps;
import org.firstinspires.ftc.teamcode.auto.steps.AutoContext;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * OpMode lifecycle shared by every autonomous: hardware, per-loop update order, the 30 s cutoff, telemetry, and pose
 * handoff to TeleOp. It knows nothing about any particular route; a subclass supplies the alliance, the start pose,
 * and the route as a {@link Step}. A partner-robot autonomous extends this and builds a different route.
 */
public abstract class AutoOpMode extends OpMode {
    /** past this the OpMode stops commanding motion so we never drive into the end-of-auto buzzer **/
    private static final double AUTO_CUTOFF_MILLIS = 29_750;
    private static final String LOG_TAG = "AutoOpMode";

    /** @return true if this OpMode runs on the blue alliance, false for red; selects the Limelight pipeline. **/
    protected abstract boolean isBlueTeam();

    /** @return where the robot is placed at init. Called once during {@link #init()}, after hardware exists. **/
    protected abstract Pose startPose();

    /**
     * Builds the whole routine. Called once during {@link #init()}, after hardware exists; build every path here so a
     * bad pose fails at init, not mid-match.
     */
    protected abstract Step buildRoute(AutoContext ctx);

    private Robot robot;
    private Vision vision;
    private Follower follower;
    private AutoContext ctx;
    private Sequencer sequencer;
    private List<String> routePlan;
    private Timer loopTimer, autoTimer;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        loopTimer = new Timer();
        autoTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap);
        robot.scheduler.setAutoLoopLength(); // make sure we use the longer auto loops
        vision = new Vision(hardwareMap, isBlueTeam());
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        ctx = new AutoContext(robot, follower, vision);

        Step route = buildRoute(ctx);
        sequencer = new Sequencer(Collections.singletonList(route));
        routePlan = Steps.describe(route);
        for (String line : routePlan) RobotLog.ii(LOG_TAG, line);

        sendTelemetry(true);
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        sendTelemetry(true);
        for (String line : routePlan) telemetryM.debug(line); // the plan, so a wrong branch is visible before the match
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        loopTimer.reset(); // so the first loop() doesn't report init time as loop time
        autoTimer.reset();
        sequencer.start();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        vision.update();
        follower.update();
        ctx.updateFlywheel(); // re-run PIDF every loop so RPM actually converges on target
        robot.update(); // advances the shooting state machine and intake power-save
        if (sequencer.update() == Sequencer.Status.FINISHED) requestOpModeStop();
        robot.scheduler.update(loopTimer.nanoseconds()); // write hardware first: skipping this budget starves every device
        updateHandoff();

        if (autoTimer.milliseconds() > AUTO_CUTOFF_MILLIS && sequencer.status() == Sequencer.Status.RUNNING) {
            sequencer.abort();
            follower.stop();
        }

        if (Tunables.isDebugging) {
            sendTelemetry(false);
        }
        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.addData("target RPM", ctx.targetRPM());
        telemetryM.addData("RPM", robot.flywheel.getRPM());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        if (ctx != null) {
            ctx.shutOff();
            ctx.updateFlywheel();
        }
        updateHandoff();
    }

    private void updateHandoff() {
        if (follower != null) {
            HandoffState.pose = follower.pose();
        }
    }

    private void sendTelemetry(boolean sendInitTime) {
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("Init time (millis): " + loopTimer.get(TimeUnit.SECONDS));
        } else {
            if (ctx.targetRPM() > 0 && !ctx.isFlywheelWithinMargin()) telemetryM.debug("WARNING: FLYWHEEL OUT OF MARGIN");
        }
        Pose pose = follower.pose(); // read once; each call may allocate
        telemetryM.debug("Step: " + sequencer.currentName());
        telemetryM.addData("flywheel RPM", robot.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", ctx.targetRPM());
        telemetryM.addData("x", pose.x());
        telemetryM.addData("y", pose.y());
        telemetryM.addData("Heading", pose.heading());
        telemetryM.addData("tags detected", vision.getLatestResult().getBotposeTagCount());
    }
}

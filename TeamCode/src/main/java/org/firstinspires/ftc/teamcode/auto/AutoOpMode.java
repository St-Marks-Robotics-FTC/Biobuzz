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
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.auto.core.Sequencer;
import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.core.Steps;
import org.firstinspires.ftc.teamcode.auto.steps.AutoContext;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/** Lifecycle shared by every autonomous: hardware, loop order, 30 s cutoff, telemetry, pose handoff. Subclasses supply the route. **/
public abstract class AutoOpMode extends OpMode {
    /** past this we stop commanding motion **/
    private static final double AUTO_CUTOFF_MILLIS = 29_750;
    private static final String LOG_TAG = "AutoOpMode";

    /** @return true for blue, false for red; selects the Limelight pipeline **/
    protected abstract boolean isBlueTeam();

    /** @return where the robot is placed at init **/
    protected abstract Pose startPose();

    /** Builds the whole routine once at init; build every path here so a bad pose fails early. **/
    protected abstract Step buildRoute(AutoContext ctx);

    private Robot robot;
    private Vision vision;
    private Follower follower;
    private AutoContext ctx;
    private Sequencer sequencer;
    private List<String> routePlan;
    private Timer loopTimer, autoTimer;
    private TelemetryManager telemetryM;
    private IoThread io;

    @Override
    public void init() {
        loopTimer = new Timer();
        autoTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        io = new IoThread(hardwareMap);
        robot = new Robot(io);
        vision = new Vision(hardwareMap, isBlueTeam());
        telemetryM.debug("Creating follower... (this may take a while)");
        telemetryM.update(telemetry);
        follower = Constants.create(io);
        follower.setPose(startPose());
        io.start(); // every device exists now; first cycle writes the init positions
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
        io.awaitNewCycle(); // fresh readings; the loop runs at the I/O cycle rate
        loopTimer.reset();
        vision.update();
        follower.update();
        ctx.updateFlywheel(); // re-run PIDF every loop so RPM actually converges on target
        robot.pollenLaunch.update(); // advances the shooting state machine and intake power-save
        if (sequencer.update() == Sequencer.Status.FINISHED) requestOpModeStop();
        updateHandoff();


        if (autoTimer.milliseconds() > AUTO_CUTOFF_MILLIS && sequencer.status() == Sequencer.Status.RUNNING) {
            sequencer.abort();
            follower.stop();
        }

        if (BozoAuto.IS_DEBUGGING) {
            sendTelemetry(false);
        }
        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS));
        telemetryM.addData("I/O cycle (ms)", io.lastCycleMillis());
        telemetryM.addData("target RPM", ctx.targetRPM());
        telemetryM.addData("RPM", robot.pollenLaunch.flywheel.getRPM());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        if (ctx != null) {
            ctx.shutOff();
            ctx.updateFlywheel();
        }
        if (io != null) io.stop(); // stop the I/O thread, then flush the shut-off commands above
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
        telemetryM.addData("flywheel RPM", robot.pollenLaunch.flywheel.getRPM());
        telemetryM.addData("flywheel target RPM", ctx.targetRPM());
        telemetryM.addData("x", pose.x());
        telemetryM.addData("y", pose.y());
        telemetryM.addData("Heading", pose.heading());
        telemetryM.addData("tags detected", vision.getLatestResult().getBotposeTagCount());
    }
}

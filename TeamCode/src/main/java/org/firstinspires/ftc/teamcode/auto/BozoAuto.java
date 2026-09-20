package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.curves.*;
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

    private enum State {
        START,
        RUN_PATH_1,
        RUN_PATH_2,
        RUN_PATH_3,
        END
    }


    /** these are the **only variables** that should change during runtime **/

    State state = State.START;

    /** end vars that can change **/
    private Path // some of these can probably just be Paths, but whatever
            path1,
            path2,
            path3;

    private void buildPaths() {
        // this path goes from the starting point to our scoring point
        path1 = line(startPose, config.pose1).linear(startPose, config.pose1);

        path2 = line(config.pose1, config.pose2).linear(config.pose1, config.pose2);

        path3 = line(config.pose2, config.pose3).linear(config.pose2, config.pose3);
    }

    // isn't as flexible as https://state-factory.gitbook.io/state-factory, but it should be good enough for now
    // "good enough for now" was a whole season lol

    private void autoPathUpdate() {
        switch (state) {
            case START:
                follower.follow(path1);
                setPathState(State.RUN_PATH_1);
                break;
            case RUN_PATH_1:
                if (!follower.isBusy()) {
                    follower.follow(path2);
                    setPathState(State.RUN_PATH_2);
                }
                break;
            case RUN_PATH_2:
                if (!follower.isBusy()) {
                    follower.follow(path3);
                    setPathState(State.RUN_PATH_3);
                }
                break;
            case RUN_PATH_3:
                if (!follower.isBusy()) {
                    setPathState(State.END);
                }
                break;
            case END:
                requestOpModeStop(); // request to stop our OpMode so it automatically transfers to TeleOp
                break;
        }
    }

    /** These changes our path state while also resetting its timer **/
    private void setPathState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        loopTimer.reset();

        follower.update(); // update our follower before anything else

        updateHandoff();

        autoPathUpdate(); // update our state machine and run its actions

        if (Tunables.isDebugging) {
            sendTelemetry(false); // we don't want to send our init time now that our OpMode is running
        }

        telemetryM.addData("loop time (millis)", loopTimer.get(TimeUnit.MILLISECONDS)); // we want to be able to graph this
        telemetryM.update(telemetry);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        config = buildConfig(); // get our config

        // set up our timers
        loopTimer = new Timer();
        stateTimer = new Timer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry(); // this gets our telemetryM object so we can write telemetry to Panels
        robot = new Robot(hardwareMap);

        telemetryM.debug("creating follower... (this may take a while)");
        telemetryM.update(telemetry);

        follower = Constants.create(hardwareMap);
        startPose = getStartPose(); // the getStartPose method will be included in different classes for start points

        telemetryM.debug("building paths... (this may take a while)");
        telemetryM.update(telemetry);

        buildPaths(); // this will create our paths from our predefined variables

        follower.setPose(startPose); // this will set our starting pose from our getStartPose() function

        sendTelemetry(true); // send full telemetry once, so in Panels we can set all graphs to true
        // this lets us observe our telemetry graphs from the start
        telemetryM.update(telemetry);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        sendTelemetry(true);
        telemetryM.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        setPathState(State.START);
    }

    // everything else should automatically disable, but we still want to update our handoff
    @Override
    public void stop() {
        updateHandoff(); // update our handoff when we stop
        // moving servos doesn't really work because stopping the OpMode disables servo power
    }

    public void updateHandoff() {
        HandoffState.pose = follower.pose();
    }

    public void sendTelemetry(boolean sendInitTime) {
        // sendInitTime = true; is only for init()
        if (sendInitTime) {
            telemetryM.addLine("INIT COMPLETE: READY TO START");
            telemetryM.debug("init time (millis): " + loopTimer.get(TimeUnit.SECONDS)); // i don't think addData works in init()
        }

        // state
        telemetryM.debug("path state: " + state);

        // odo
        telemetryM.addData("x", follower.pose().x());
        telemetryM.addData("y", follower.pose().y());
        telemetryM.addData("heading", follower.pose().heading());

        // timing
        telemetryM.debug("OpMode time (seconds): " + getRuntime());
    }
}
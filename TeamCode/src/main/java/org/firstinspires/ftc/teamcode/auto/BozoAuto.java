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


    /** these are the **only variables** that should change during runtime **/

    State state = State.START;

    /** end vars that can change **/
    private Path // some of these can probably just be Paths, but whatever
            startToShootLeft,
            shootLeftToFlowerLeft,
            flowerLeftToShootRight,
            shootRightToFlowerRight,
            flowerRightToShootRight,
            shootRightToEnd;

    private void buildPaths() {
        // this path goes from the starting point to our scoring point
        startToShootLeft = line(startPose, config.shootLeftPose).linear(startPose, config.shootLeftPose);

        shootLeftToFlowerLeft = line(config.shootLeftPose, config.flowerLeftPose).linear(config.shootLeftPose, config.flowerLeftPose);

        flowerLeftToShootRight = line(config.flowerLeftPose, config.shootRightPose).linear(config.flowerLeftPose, config.shootRightPose);

        shootRightToFlowerRight = line(config.shootRightPose, config.flowerRightPose).linear(config.shootRightPose, config.flowerRightPose);

        flowerRightToShootRight = line(config.flowerRightPose, config.shootRightPose).linear(config.flowerRightPose, config.shootRightPose);

        shootRightToEnd = line(config.shootRightPose, config.endPose).linear(config.shootRightPose, config.endPose);
    }

    // isn't as flexible as https://state-factory.gitbook.io/state-factory, but it should be good enough for now
    // "good enough for now" was a whole season lol

    private void autoPathUpdate() {
        switch (state) {
            case START:
                follower.follow(startToShootLeft); // Start -> Shoot Left
                pastState = state; // Remember last state
                setPathState(State.SHOOTING);
                break;
            case SHOOTING:
                if (!follower.isBusy() && pastState == State.START ) {
                    follower.follow(shootLeftToFlowerLeft);
                    pastState = State.SHOOTING;
                    setPathState(State.GO_TO_LEFT_FLOWER); // reloading
                } else if (!follower.isBusy() && pastState == State.FIRST_PICKUP){
                    follower.follow(shootRightToFlowerRight);
                    pastState = State.SHOOTING;
                    setPathState(State.GO_TO_RIGHT_FLOWER);
                } else if (!follower.isBusy() && pastState == State.SECOND_PICKUP){
                    follower.follow(shootRightToEnd);
                    pastState = State.SHOOTING;
                    setPathState(State.END);
                }
            case GO_TO_LEFT_FLOWER:
                if (!follower.isBusy() && pastState == State.SHOOTING) {
                    pastState = State.GO_TO_LEFT_FLOWER;
                    setPathState(State.FIRST_PICKUP);
                }
            case FIRST_PICKUP:
                if (!follower.isBusy()) {
                    follower.follow(flowerLeftToShootRight);
                    pastState = State.FIRST_PICKUP;
                    setPathState(State.SHOOTING);
                }
            case SECOND_PICKUP:
                if (!follower.isBusy()) {
                    pastState = State.SECOND_PICKUP;
                    follower.follow(flowerRightToShootRight);
                    setPathState(State.SHOOTING);
                }
            case GO_TO_RIGHT_FLOWER:
                if (!follower.isBusy()) {
                    setPathState(State.SECOND_PICKUP);
                }
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
            telemetryM.debug("init time (millis): " + loopTimer.get(TimeUnit.SECONDS)); // I don't think addData works in init()
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
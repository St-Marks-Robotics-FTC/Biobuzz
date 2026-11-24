package org.firstinspires.ftc.teamcode.subsys.launch;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.io.IoThread;

public class Launch {
    /** configurables **/

    public static double INTAKE_STOP_TIME = 80; // time to wait for intake to stop before opening transfer
    public static double TRANSFER_OPENING_TIME = 75;
    public static double SHOOTING_TIME = 700;

    /** end configurables **/

    private enum LaunchState {
        LOCKED,
        START_LAUNCH,
        STOPPING_INTAKE,
        OPENING_TRANSFER,
        SHOOTING
    }

    private LaunchState launchState = LaunchState.LOCKED;
    private final Timer launchTimer;

    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    public Launch(IoThread io, String ballType) {
        flywheel = new Flywheel(io, ballType);
        intake = new Intake(io, ballType);
        transfer = new Transfer(io, ballType);

        launchTimer = new Timer();
    }

    public void startLaunch() {
        launchState = LaunchState.START_LAUNCH;
    }

    public void update() {
        switch (launchState) {
            case LOCKED:
                // do nothing
                break;
            case START_LAUNCH:
                launchTimer.reset();
                intake.off();
                launchState = LaunchState.STOPPING_INTAKE;
            case STOPPING_INTAKE:
                if (launchTimer.milliseconds() >= INTAKE_STOP_TIME) {
                    // we've waited for intake to slow down so we can now open transfer without killing the servo
                    transfer.open();
                    launchState = LaunchState.OPENING_TRANSFER;
                    launchTimer.reset();
                }
                break;
            case OPENING_TRANSFER:
                if (launchTimer.milliseconds() >= TRANSFER_OPENING_TIME) {
                    // we've waited for transfer to open so we can now start our intake at launching power
                    intake.forward();
                    launchState = LaunchState.SHOOTING;
                    launchTimer.reset();
                }
                break;
            case SHOOTING:
                if (launchTimer.milliseconds() >= SHOOTING_TIME) {
                    // we've waited for all the balls to come through
                    stopLaunch();
                }
                break;
        }

        intake.update(isLaunching());
    }

    public void stopLaunch() {
        intake.forward();
        transfer.close();
        launchState = LaunchState.LOCKED;
    }

    public boolean isLaunching() {
        return launchState != LaunchState.LOCKED;
    }
}

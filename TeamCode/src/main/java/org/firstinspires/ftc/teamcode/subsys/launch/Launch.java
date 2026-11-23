package org.firstinspires.ftc.teamcode.subsys.launch;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;

public class Launch {

    private enum LaunchState {
        LOCKED,
        START_LAUNCH,
        STOPPING_INTAKE,
        OPENING_TRANSFER,
        SHOOTING
    }

    private LaunchState launchState = LaunchState.LOCKED;
    private Timer launchTimer;

    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    public Launch(HWScheduler hwScheduler, String ballType) {
        flywheel = new Flywheel(hwScheduler, ballType);
        intake = new Intake(hwScheduler, ballType);
        transfer = new Transfer(hwScheduler, ballType);

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
                if (launchTimer.milliseconds() >= Tunables.launchIntakeStopTime) {
                    // we've waited for intake to slow down so we can now open transfer without killing the servo
                    transfer.open();
                    launchState = LaunchState.OPENING_TRANSFER;
                    launchTimer.reset();
                }
                break;
            case OPENING_TRANSFER:
                if (launchTimer.milliseconds() >= Tunables.launchTransferOpeningTime) {
                    // we've waited for transfer to open so we can now start our intake at launching power
                    intake.forward();
                    launchState = LaunchState.SHOOTING;
                    launchTimer.reset();
                }
                break;
            case SHOOTING:
                if (launchTimer.milliseconds() >= Tunables.launchShootingTime) {
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

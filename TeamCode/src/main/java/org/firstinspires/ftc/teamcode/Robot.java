/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;

import org.firstinspires.ftc.teamcode.io.HubDevice;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class Robot {
    private static final long VOLTAGE_READ_PERIOD_MS = 100; // battery voltage moves slowly; each read costs a Lynx command

    private final HubDevice expansionHub;
    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    public Flower flower;

    private Timer launchTimer;

    private enum LaunchState {
        LOCKED,
        START_LAUNCH,
        STOPPING_INTAKE,
        OPENING_TRANSFER,
        SHOOTING
    }

    private LaunchState launchState = LaunchState.LOCKED;

    /** builds every subsystem's devices on {@code io}; call before {@code io.start()} **/
    public Robot(IoThread io) {
        expansionHub = new HubDevice(io, "Expansion Hub 2", ReadPeriod.millis(VOLTAGE_READ_PERIOD_MS)); // I believe this starts at 2

        flywheel = new Flywheel(io);
        intake = new Intake(io);
        transfer = new Transfer(io);
        flower = new Flower(io);

        launchTimer = new Timer();
    }

    /** newest battery voltage read by the I/O thread (at most 100 ms old) **/
    public double getExpansionHubVoltage() {
        return expansionHub.inputVoltage().value();
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
                    intake.forward(); // go forward full power
                    transfer.close(); // keep any more balls from falling through
                    launchState = LaunchState.LOCKED;
                }
                break;
        }

        intake.update(isLaunching());
        flower.update();
    }

    public boolean isLaunching() {
        return launchState != LaunchState.LOCKED;
    }
}

/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class Robot {
    public LynxModule controlHub;
    public LynxModule expansionHub;
    public HWScheduler scheduler;
    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    private boolean isLaunching = false;
    private Timer launchTimer;

    private enum LaunchState {
        LOCKED,
        START_LAUNCH,
        STOPPING_INTAKE,
        OPENING_TRANSFER,
        SHOOTING
    }

    private LaunchState launchState = LaunchState.LOCKED;

    public Robot(HardwareMap hw) {
        // initialize everything here

        controlHub = hw.get(LynxModule.class, "Control Hub");
        expansionHub = hw.get(LynxModule.class, "Expansion Hub 2"); // I believe this starts at 2

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        scheduler = new HWScheduler(); // subsystems register their devices with this

        flywheel = new Flywheel(hw, scheduler);
        intake = new Intake(hw, scheduler);
        transfer = new Transfer(hw, scheduler);

        launchTimer = new Timer();
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    /** writes queued hardware changes - call this once at the end of every loop **/
    public double update(double elapsedTime) { // take in time we have already spent this loop and return time the scheduler took
        return scheduler.update(elapsedTime);
    }

    public void startLaunch() {
        launchState = LaunchState.START_LAUNCH;
    }

    public void updateLaunch() { // returns true when we're done launching
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
                    intake.forwardLaunching();
                    launchState = LaunchState.SHOOTING;
                    launchTimer.reset();
                }
                break;
            case SHOOTING:
                // TODO: add a staggered shooting system where we go from full to zero power for each ball to fix inconsistency of forwardLaunchingPower
                if (launchTimer.milliseconds() >= Tunables.launchShootingTime) {
                    // we've waited for all the balls to come through
                    intake.forward(); // go forward full power
                    transfer.close(); // keep any more balls from falling through
                    launchState = LaunchState.LOCKED;
                }
                break;
        }
    }

    public boolean isLaunching() {
        return launchState != LaunchState.LOCKED;
    }
}

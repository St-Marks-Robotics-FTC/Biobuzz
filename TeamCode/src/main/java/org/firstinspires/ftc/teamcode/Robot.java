/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class Robot {
    public LynxModule controlHub;
    public LynxModule expansionHub;
    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    public Flower flower;

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

        flywheel = new Flywheel(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);
        flower = new Flower(hw);

        launchTimer = new Timer();
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    public double getExpansionHubVoltage() {
        return expansionHub.getInputVoltage(VoltageUnit.VOLTS);
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
                // TODO: add a staggered shooting system where we go from full to zero power for each ball to fix inconsistency of forwardLaunchingPower
                if (launchTimer.milliseconds() >= Tunables.launchShootingTime) {
                    // we've waited for all the balls to come through
                    intake.forward(); // go forward full power
                    transfer.close(); // keep any more balls from falling through
                    launchState = LaunchState.LOCKED;
                }
                break;
        }

        intake.update(isLaunching);
    }

    public boolean isLaunching() {
        return launchState != LaunchState.LOCKED;
    }
}

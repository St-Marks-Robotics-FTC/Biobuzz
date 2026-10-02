/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsys.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.Intake;
import org.firstinspires.ftc.teamcode.subsys.Transfer;

public class Robot {
    public LynxModule controlHub;
    public LynxModule expansionHub;
    public Flywheel flywheel;
    public Intake intake;
    public Transfer transfer;

    private boolean isLaunching = false;
    private Timer launchTimer;

    public Robot(HardwareMap hw) {
        // initialize everything here

        controlHub = hw.get(LynxModule.class, "Control Hub");
        expansionHub = hw.get(LynxModule.class, "Expansion Hub 2"); // I believe this starts at 2

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        flywheel = new Flywheel(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);

        launchTimer = new Timer();
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    public void startLaunch() {
        transfer.open();
        intake.forwardLaunching();
        launchTimer.reset();
        isLaunching = true;
    }

    public void updateLaunch() { // returns true when we're done launching
        if (isLaunching) {
            if (launchTimer.milliseconds() >= Tunables.launchOpenTime) {
                // end launch
                transfer.close();
                intake.forward();
                isLaunching = false;
            }
        } else {
            if (intake.getState() == Intake.State.FORWARD_LAUNCHING) {
                intake.forward();
            }
        }
    }

    public boolean isLaunching() {
        return isLaunching;
    }
}

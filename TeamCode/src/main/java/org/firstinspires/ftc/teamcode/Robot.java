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

    private enum LaunchState {
        IDLE,
        WAIT_FOR_LAUNCH,
        WAIT_FOR_RPM
    }

    private LaunchState launchState = LaunchState.IDLE;
    private Timer launchTimer;
    private int ballsRemaining;

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

    public void launchBalls(int balls) {
        ballsRemaining = balls;
        launchState = LaunchState.WAIT_FOR_LAUNCH;
        launchTimer.reset();
        transfer.open();
        intake.forwardLaunching();
    }

    public boolean updateLaunch(double desiredRPM) { // return true if launch complete
        switch (launchState) {
            case IDLE:
                transfer.close();
                break;
            case WAIT_FOR_LAUNCH:
                if (launchTimer.milliseconds() > Tunables.launchOpenTime) {
                    transfer.close();
                    intake.off();
                    ballsRemaining--;
                    if (ballsRemaining > 0) {
                        launchState = LaunchState.WAIT_FOR_RPM;
                        launchTimer.reset();
                    } else {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
            case WAIT_FOR_RPM:
                if (Math.abs(flywheel.getRPM() - desiredRPM) <= Tunables.flywheelMarginRPM) {
                    launchState = LaunchState.WAIT_FOR_LAUNCH;
                    launchTimer.reset();
                    transfer.open();
                    intake.forwardLaunching();
                }
                break;
        }
        return false;
    }
}

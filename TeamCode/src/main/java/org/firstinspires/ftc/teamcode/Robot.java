/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.subsys.Field;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Turret;
import org.firstinspires.ftc.teamcode.subsys.launch.Flywheel;
import org.firstinspires.ftc.teamcode.subsys.launch.Intake;
import org.firstinspires.ftc.teamcode.subsys.launch.Launch;
import org.firstinspires.ftc.teamcode.subsys.launch.Transfer;

public class Robot {
    public LynxModule controlHub;
    public LynxModule expansionHub;
    public HWScheduler scheduler;

    public Launch pollenLaunch;
    public Launch nectarLaunch;
    public Turret pollenTurret;

    public Flower flower;


    public Robot(HardwareMap hw) {
        // initialize everything here

        controlHub = hw.get(LynxModule.class, "Control Hub");
        expansionHub = hw.get(LynxModule.class, "Expansion Hub 2"); // I believe this starts at 2

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        scheduler = new HWScheduler(hw); // subsystems register their devices with this

        pollenLaunch = new Launch(scheduler, "pollen");
        nectarLaunch = new Launch(scheduler, "nectar");

        pollenTurret = new Turret(scheduler);
        flower = new Flower(scheduler);
    }

    public void updateAutoShoot(Field.ScoringData scoringData) {
        double voltage = getExpansionHubVoltage();

        pollenLaunch.flywheel.update(scoringData.pollenIdealRPM(), voltage);
        nectarLaunch.flywheel.update(scoringData.nectarIdealRPM(), voltage);

        pollenLaunch.update();
        nectarLaunch.update();
        pollenTurret.updateAngle(scoringData.turretAngle());

        flower.update();
    }

    public double getSystemCurrent() { // return system current in amps
        return (controlHub.getCurrent(CurrentUnit.AMPS) + expansionHub.getCurrent(CurrentUnit.AMPS));
    }

    public double getExpansionHubVoltage() {
        return expansionHub.getInputVoltage(VoltageUnit.VOLTS);
    }
}

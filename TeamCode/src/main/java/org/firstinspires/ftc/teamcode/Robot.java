/** this is our mega-class that holds all robot functions that are shared between auto and teleop **/

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.io.HubDevice;
import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ReadPeriod;
import org.firstinspires.ftc.teamcode.subsys.Field;
import org.firstinspires.ftc.teamcode.subsys.Flower;
import org.firstinspires.ftc.teamcode.subsys.Light;
import org.firstinspires.ftc.teamcode.subsys.Turret;
import org.firstinspires.ftc.teamcode.subsys.launch.Launch;

public class Robot {
    private static final long VOLTAGE_READ_PERIOD_MS = 100; // battery voltage moves slowly; each read costs a Lynx command

    private final HubDevice expansionHub;

    public Launch pollenLaunch;
    public Launch nectarLaunch;
    public Turret pollenTurret;
    public Light light;

    public Flower flower;

    /** builds every subsystem's devices on {@code io}; call before {@code io.start()} **/
    public Robot(IoThread io) {
        expansionHub = new HubDevice(io, "Expansion Hub 2", ReadPeriod.millis(VOLTAGE_READ_PERIOD_MS)); // I believe this starts at 2

        pollenLaunch = new Launch(io, "pollen");
        nectarLaunch = new Launch(io, "nectar");

        pollenTurret = new Turret(io);
        flower = new Flower(io);
        light = new Light(io);
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

    /** newest battery voltage read by the I/O thread (at most 100 ms old) **/
    public double getExpansionHubVoltage() {
        return expansionHub.inputVoltage().value();
    }
}

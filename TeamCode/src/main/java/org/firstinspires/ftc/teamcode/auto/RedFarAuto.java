package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Blue is the source of truth: this is BlueFarAuto's route rotated 180 degrees about field center.
@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends BozoAuto {
    @Override
    protected AutoConfig buildConfig() {
        return BlueFarAuto.route().rotated();
    }
}

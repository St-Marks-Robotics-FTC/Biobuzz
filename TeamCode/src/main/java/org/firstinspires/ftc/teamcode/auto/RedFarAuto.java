package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends BozoAuto {
    @Override
    protected AutoConfig buildConfig() {
        return BlueFarAuto.route().rotated();
    }
}

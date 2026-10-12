package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Based off of the bluecloseauto rotated
@Autonomous(name = "RedCloseAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedCloseAuto extends BozoAuto {
    @Override
    protected AutoConfig buildConfig() {
        return BlueCloseAuto.route().rotated();
    }
}

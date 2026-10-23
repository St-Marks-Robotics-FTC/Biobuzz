package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueCloseAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueCloseAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(9, 87, Math.toRadians(90));
    }
}
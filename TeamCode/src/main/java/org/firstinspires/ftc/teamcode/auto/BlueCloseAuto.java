package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueCloseAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueCloseAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return new Pose(84, 6, Math.toRadians(90));
    }
}
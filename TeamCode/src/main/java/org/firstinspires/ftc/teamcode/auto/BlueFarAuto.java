package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends BlueAuto {
    @Override
    public Pose getStartPose() {
        return startPose;
    }
}

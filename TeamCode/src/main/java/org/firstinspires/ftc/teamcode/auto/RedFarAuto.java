package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedFarAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends RedAuto {
    @Override
    public Pose getStartPose() {
        return startPose;
    }
}

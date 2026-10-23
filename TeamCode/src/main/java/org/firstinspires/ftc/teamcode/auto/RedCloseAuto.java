package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.math.Pose;

@Autonomous(name = "RedCloseAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedCloseAuto extends RedAuto {
    @Override
    public Pose getStartPose() {
        // Start pose of the robot for this variant.
        return new Pose(135, 57, Math.toRadians(270));
    }
}
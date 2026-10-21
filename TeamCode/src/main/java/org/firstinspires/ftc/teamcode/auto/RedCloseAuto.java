package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.math.Pose;

@Autonomous(name = "RedCloseAuto", group = "Red", preselectTeleOp = "RedTeleOp")
public class RedCloseAuto extends RedAuto {
    @Override
    public Pose getStartPose() {
        // Pose the robot is placed at for this variant; it drives to the shoot pose first.
        return new Pose(60, 15, Math.toRadians(90));
    }
}
package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class HandoffState {
    // this will be overwritten once the first auto is run, but we can start it in the middle for easy TeleOp testing
    public static Pose pose = new Pose(72, 72, Math.toRadians(90)); // pose to hand off from auto->TeleOp
}

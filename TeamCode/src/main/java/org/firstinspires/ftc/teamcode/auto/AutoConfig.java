package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose startPose,
            refuelPose;

    public AutoConfig(
            Pose startPose,
            Pose refuelPose
    ) {
        this.startPose = startPose;
        this.refuelPose = refuelPose;
    }
}
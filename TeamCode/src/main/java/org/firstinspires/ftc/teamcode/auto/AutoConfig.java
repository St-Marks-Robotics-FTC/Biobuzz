package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose shootPose,
            intermediatePose,
            refuelPose;

    public AutoConfig(
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose
    ) {
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
    }
}
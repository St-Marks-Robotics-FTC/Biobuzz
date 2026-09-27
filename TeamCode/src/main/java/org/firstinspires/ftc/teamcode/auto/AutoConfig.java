package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose startPose,
            shootRightPose,
            refuelPose,
            endPose;

    public AutoConfig(
            Pose startPose,
            Pose shootRightPose,
            Pose refuelPose,
            Pose endPose
    ) {
        this.startPose = startPose;
        this.shootRightPose = shootRightPose;
        this.refuelPose = refuelPose;
        this.endPose = endPose;
    }
}
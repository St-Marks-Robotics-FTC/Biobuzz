package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose startPose,
            shootLeftPose,
            shootRightPose,
            flowerLeftPose,
            flowerRightPose,
            endPose;

    public AutoConfig(
            Pose startPose,
            Pose shootLeftPose,
            Pose shootRightPose,
            Pose flowerLeftPose,
            Pose flowerRightPose,
            Pose endPose
    ) {
        this.startPose = startPose;
        this.shootLeftPose = shootLeftPose;
        this.shootRightPose = shootRightPose;
        this.flowerLeftPose = flowerLeftPose;
        this.flowerRightPose = flowerRightPose;
        this.endPose = endPose;
    }
}

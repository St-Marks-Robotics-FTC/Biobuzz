package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose startPose,
            intermediatePose,
            refuelPose;

    /**
     * @param intermediatePose waypoint the robot drives to (linear heading interpolation) after the first shoot
     *                         and before continuing to {@code refuelPose}. Its heading is the angle the robot holds
     *                         while approaching {@code refuelPose}; it then turns to {@code refuelPose}'s heading near the end.
     */
    public AutoConfig(
            Pose startPose,
            Pose intermediatePose,
            Pose refuelPose
    ) {
        this.startPose = startPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
    }
}
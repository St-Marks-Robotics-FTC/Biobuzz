package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * Poses the autonomous drives through. {@link #startPose} is where the robot is placed at init; it drives from there
 * to {@link #shootPose} before the first shot. {@link #parkControlPose} is the bezier control point that bends the
 * path from {@link #shootPose} to {@link #parkPose}; its heading is ignored (only x/y shape the curve).
 *
 * If the hive tipped over, the robot instead curves from {@link #shootPose} to {@link #tippedShootPose} (the scoring
 * side), bent by {@link #tippedControlPose} (heading ignored), shoots there, then drives a straight line to
 * {@link #parkPose}.
 */
public class AutoConfig {
    public Pose startPose,
            shootPose,
            intermediatePose,
            refuelPose,
            parkControlPose,
            parkPose,
            tippedControlPose,
            tippedShootPose;

    public AutoConfig(
            Pose startPose,
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose,
            Pose parkControlPose,
            Pose parkPose,
            Pose tippedControlPose,
            Pose tippedShootPose
    ) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
        this.parkControlPose = parkControlPose;
        this.parkPose = parkPose;
        this.tippedControlPose = tippedControlPose;
        this.tippedShootPose = tippedShootPose;
    }
}
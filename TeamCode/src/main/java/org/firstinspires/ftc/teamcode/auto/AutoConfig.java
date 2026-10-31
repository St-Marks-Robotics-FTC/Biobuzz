package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * Poses the autonomous drives through. {@link #startPose} is where the robot is placed at init; it drives from there
 * to {@link #shootPose} before the first shot. {@link #parkControlPose} is the bezier control point that bends the
 * path from {@link #shootPose} to {@link #parkPose}; its heading is ignored (only x/y shape the curve).
 *
 * If the hive tipped over, the robot instead curves from {@link #shootPose} to {@link #tippedShootPose} (the scoring
 * side), bent by {@link #tippedControlPose1} then {@link #tippedControlPose2} (cubic bezier; headings ignored), shoots
 * there, then drives a straight line to {@link #parkPose}.
 */
public class AutoConfig {
    public Pose startPose,
            shootPose,
            intermediatePose,
            refuelPose,
            parkControlPose,
            parkPose,
            tippedControlPose1,
            tippedControlPose2,
            tippedShootPose;

    public AutoConfig(
            Pose startPose,
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose,
            Pose parkControlPose,
            Pose parkPose,
            Pose tippedControlPose1,
            Pose tippedControlPose2,
            Pose tippedShootPose
    ) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
        this.parkControlPose = parkControlPose;
        this.parkPose = parkPose;
        this.tippedControlPose1 = tippedControlPose1;
        this.tippedControlPose2 = tippedControlPose2;
        this.tippedShootPose = tippedShootPose;
    }
}
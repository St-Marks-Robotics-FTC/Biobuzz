package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

/**
 * Poses the autonomous drives through, in path order. {@link #startPose} is where the robot is placed at init; it
 * drives from there to {@link #shootPose} before the first shot.
 *
 * If the hive tipped over, the robot curves from {@link #shootPose} to {@link #tippedShootPose} (the scoring side),
 * bent by {@link #tippedControlPose1} then {@link #tippedControlPose2} (cubic bezier), and shoots there. It then
 * collects each flower: a full-speed approach to the flower's intermediate pose, then a straight line at reduced
 * speed into the flower; after collecting flower 1 it drives a straight line back to {@link #tippedShootPose} to shoot
 * again. Flower 1's approach ({@link #flowerIntermediatePose}) is a straight line; flower 2's is a curve bent by
 * {@link #flower2ControlPose} to {@link #flower2IntermediatePose}, then into {@link #flower2Pose}. After flower 2 it
 * drives a straight line back to the original {@link #shootPose} (not the tipped one) for the third shot, then curves
 * (bent by {@link #parkControlPose}) from {@link #shootPose} to {@link #parkPose}.
 *
 * Headings of control poses are ignored (only x/y shape the curve).
 */
public class AutoConfig {
    public Pose startPose,
            shootPose,
            intermediatePose,
            refuelPose,
            tippedControlPose1,
            tippedControlPose2,
            tippedShootPose,
            flowerIntermediatePose,
            flowerPose,
            flower2ControlPose,
            flower2IntermediatePose,
            flower2Pose,
            parkControlPose,
            parkPose;

    public AutoConfig(
            Pose startPose,
            Pose shootPose,
            Pose intermediatePose,
            Pose refuelPose,
            Pose tippedControlPose1,
            Pose tippedControlPose2,
            Pose tippedShootPose,
            Pose flowerIntermediatePose,
            Pose flowerPose,
            Pose flower2ControlPose,
            Pose flower2IntermediatePose,
            Pose flower2Pose,
            Pose parkControlPose,
            Pose parkPose
    ) {
        this.startPose = startPose;
        this.shootPose = shootPose;
        this.intermediatePose = intermediatePose;
        this.refuelPose = refuelPose;
        this.tippedControlPose1 = tippedControlPose1;
        this.tippedControlPose2 = tippedControlPose2;
        this.tippedShootPose = tippedShootPose;
        this.flowerIntermediatePose = flowerIntermediatePose;
        this.flowerPose = flowerPose;
        this.flower2ControlPose = flower2ControlPose;
        this.flower2IntermediatePose = flower2IntermediatePose;
        this.flower2Pose = flower2Pose;
        this.parkControlPose = parkControlPose;
        this.parkPose = parkPose;
    }
}

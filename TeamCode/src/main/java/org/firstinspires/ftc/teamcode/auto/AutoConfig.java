package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

// Everything BozoAuto needs to drive the route; fields are listed in route order.
public class AutoConfig {
    public Pose startPose,
            shootPreloadPose;
    public Pose[] shootPreloadControls; // bezier control points between startPose and shootPreloadPose
    public Pose gardenPose;
    public Pose[] gardenControls; // bezier control points between shootPreloadPose and gardenPose
    public double gardenTiltRad; // heading held while sweeping the garden
    public Pose shootGardenPose,
            endPose;
    public Pose[] endControls; // bezier control points between shootGardenPose and endPose

    public AutoConfig(
            Pose startPose,
            Pose shootPreloadPose,
            Pose[] shootPreloadControls,
            Pose gardenPose,
            Pose[] gardenControls,
            double gardenTiltRad,
            Pose shootGardenPose,
            Pose endPose,
            Pose[] endControls
    ) {
        this.startPose = startPose;
        this.shootPreloadPose = shootPreloadPose;
        this.shootPreloadControls = shootPreloadControls;
        this.gardenPose = gardenPose;
        this.gardenControls = gardenControls;
        this.gardenTiltRad = gardenTiltRad;
        this.shootGardenPose = shootGardenPose;
        this.endPose = endPose;
        this.endControls = endControls;
    }
}

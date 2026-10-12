package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

// Everything BozoAuto needs to drive the route; fields are listed in route order.
public class AutoConfig {
    private static final double FIELD_SIZE = 144;

    public Pose startPose,
            shootPreloadPose;
    public Pose[] shootPreloadControls;
    public Pose gardenPose;
    public Pose[] gardenControls;
    public double gardenTiltRad;
    public boolean gardenTangent;
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
            boolean gardenTangent,
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
        this.gardenTangent = gardenTangent;
        this.shootGardenPose = shootGardenPose;
        this.endPose = endPose;
        this.endControls = endControls;
    }

    public AutoConfig rotated() { //Rotates routes to form Red autos
        return new AutoConfig(
                rotate(startPose),
                rotate(shootPreloadPose),
                rotateControls(shootPreloadControls),
                rotate(gardenPose),
                rotateControls(gardenControls),
                rotateHeading(gardenTiltRad),
                gardenTangent,
                rotate(shootGardenPose),
                rotate(endPose),
                rotateControls(endControls)
        );
    }

    private static double rotateHeading(double heading) {
        double rotated = (heading + Math.PI) % (2 * Math.PI);
        return rotated < 0 ? rotated + 2 * Math.PI : rotated; // keep in [0, 2pi) like the blue headings
    }

    private static Pose rotate(Pose pose) {
        return new Pose(FIELD_SIZE - pose.x(), FIELD_SIZE - pose.y(), rotateHeading(pose.heading()));
    }

    // bezier control points only carry a position
    private static Pose[] rotateControls(Pose[] controls) {
        Pose[] rotated = new Pose[controls.length];
        for (int i = 0; i < controls.length; i++) {
            rotated[i] = new Pose(FIELD_SIZE - controls[i].x(), FIELD_SIZE - controls[i].y());
        }
        return rotated;
    }
}

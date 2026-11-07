/** this class is used to provide auto parameters **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

// Everything BozoAuto needs to drive the route; fields are listed in route order.
// The close-flower stages (closeFlowerPose through lastShotControls) are null for routes that park after the garden shot.
public class AutoConfig {
    private static final double FIELD_SIZE = 144;

    public Pose startPose,
            shootPreloadPose;
    public Pose[] shootPreloadControls;
    public Pose gardenPose;
    public Pose[] gardenControls;
    public double gardenTiltRad; // heading turns toward this over the first half of the garden path, then holds
    public Pose shootGardenPose;
    public Pose[] shootGardenControls;
    public boolean isBlue; // which alliance's Limelight pipeline and hive side to use; flipped by rotated()
    public Pose closeFlowerPose;
    public Pose[] closeFlowerControls;
    public Pose secondShotPose;
    public Pose farFlowerPose;
    public Pose[] farFlowerControls;
    public Pose lastShotPose;
    public Pose[] lastShotControls;
    public Pose endPose;
    public Pose[] endControls; // bezier control points between the final shot pose and endPose

    public AutoConfig(
            Pose startPose,
            Pose shootPreloadPose,
            Pose[] shootPreloadControls,
            Pose gardenPose,
            Pose[] gardenControls,
            double gardenTiltRad,
            Pose shootGardenPose,
            Pose[] shootGardenControls,
            boolean isBlue,
            Pose closeFlowerPose,
            Pose[] closeFlowerControls,
            Pose secondShotPose,
            Pose farFlowerPose,
            Pose[] farFlowerControls,
            Pose lastShotPose,
            Pose[] lastShotControls,
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
        this.shootGardenControls = shootGardenControls;
        this.isBlue = isBlue;
        this.closeFlowerPose = closeFlowerPose;
        this.closeFlowerControls = closeFlowerControls;
        this.secondShotPose = secondShotPose;
        this.farFlowerPose = farFlowerPose;
        this.farFlowerControls = farFlowerControls;
        this.lastShotPose = lastShotPose;
        this.lastShotControls = lastShotControls;
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
                rotate(shootGardenPose),
                rotateControls(shootGardenControls),
                !isBlue,
                rotate(closeFlowerPose),
                rotateControls(closeFlowerControls),
                rotate(secondShotPose),
                rotate(farFlowerPose),
                rotateControls(farFlowerControls),
                rotate(lastShotPose),
                rotateControls(lastShotControls),
                rotate(endPose),
                rotateControls(endControls)
        );
    }

    private static double rotateHeading(double heading) {
        double rotated = (heading + Math.PI) % (2 * Math.PI);
        return rotated < 0 ? rotated + 2 * Math.PI : rotated; // keep in [0, 2pi) like the blue headings
    }

    private static Pose rotate(Pose pose) {
        if (pose == null) return null;
        return new Pose(FIELD_SIZE - pose.x(), FIELD_SIZE - pose.y(), rotateHeading(pose.heading()));
    }

    // bezier control points only carry a position
    private static Pose[] rotateControls(Pose[] controls) {
        if (controls == null) return null;
        Pose[] rotated = new Pose[controls.length];
        for (int i = 0; i < controls.length; i++) {
            rotated[i] = new Pose(FIELD_SIZE - controls[i].x(), FIELD_SIZE - controls[i].y());
        }
        return rotated;
    }
}

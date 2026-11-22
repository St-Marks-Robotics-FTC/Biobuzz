package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.BozoAuto;

/**
 * One physical flower: the waypoint in front of it, the flower itself, and the two short paths between them. We reach
 * the intermediate pose at full speed by a path that differs per call site, then {@link #creepIn} slowly and
 * {@link #backOut} again; those two never differ, so they are built here once.
 */
public final class FlowerSite {
    public final String name;
    public final Pose intermediatePose;
    public final Pose flowerPose;
    /** intermediate pose -> flower at {@link BozoAuto#FLOWER_APPROACH_SPEED} **/
    public final Path creepIn;
    /** flower -> intermediate pose at {@link BozoAuto#FLOWER_INTAKE_SPEED} **/
    public final Path backOut;

    /** @param name used in telemetry and the route plan, e.g. "scoring flower" **/
    public FlowerSite(String name, Pose intermediatePose, Pose flowerPose) {
        this.name = name;
        this.intermediatePose = intermediatePose;
        this.flowerPose = flowerPose;
        this.creepIn = PathHelpers.creepLine(intermediatePose, flowerPose);
        this.backOut = PathHelpers.limitSpeed(PathHelpers.plainLine(flowerPose, intermediatePose), BozoAuto.FLOWER_INTAKE_SPEED);
    }
}

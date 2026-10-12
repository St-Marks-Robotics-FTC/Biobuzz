package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Configurable
@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends BozoAuto {
    public static Pose startPose = new Pose(125.8792, 8.4354, Math.toRadians(90));
    public static Pose shootPreloadPose = new Pose(82.3408, 132, Math.toRadians(270));
    public static Pose gardenPose = new Pose(135.3615, 137.07, Math.toRadians(0));
    public static Pose shootGardenPose = new Pose(82.2785, 132, Math.toRadians(270));
    public static Pose endPose = new Pose(130.3962, 34.4623, Math.toRadians(180));

    // bezier control points (ai did it bruv I don't know how controls work)
    public static Pose shootPreloadControl1 = new Pose(73.4354, 14.3792);
    public static Pose shootPreloadControl2 = new Pose(84.8638, 70.5169);
    public static Pose shootPreloadControl3 = new Pose(83.1292, 24.8292);
    public static Pose gardenControl1 = new Pose(80.79, 139.7846);
    public static Pose gardenControl2 = new Pose(111.2385, 138.7362);
    public static Pose endControl1 = new Pose(132.2538, 101.4423);

    // tilt for garden sucking
    public static double gardenTiltRad = Math.toRadians(12);

    public static AutoConfig route() {
        return new AutoConfig(
                startPose,
                shootPreloadPose,
                new Pose[]{shootPreloadControl1, shootPreloadControl2, shootPreloadControl3},
                gardenPose,
                new Pose[]{gardenControl1, gardenControl2},
                gardenTiltRad,
                false,
                shootGardenPose,
                endPose,
                new Pose[]{endControl1}
        );
    }

    @Override
    protected AutoConfig buildConfig() {
        return route();
    }
}

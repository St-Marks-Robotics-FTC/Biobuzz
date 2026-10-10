package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto {
    // Blue auto route but for the red (flipped and stuff).
    public static Pose startPose = new Pose(144 - 125.8792, 144 - 8.4354, Math.toRadians(270));
    public static Pose shootPreloadPose = new Pose(144 - 82.3408, 144 - 120, Math.toRadians(90)); // 1st shooting spot
    public static Pose gardenPose = new Pose(144 - 135.3615, 144 - 134.07, Math.toRadians(180)); // garden corner, intake faces the corner
    public static Pose shootGardenPose = new Pose(144 - 82.2785, 144 - 120.4162, Math.toRadians(90)); // 2nd shooting spot
    public static Pose endPose = new Pose(144 - 130.3962, 144 - 34.4623, Math.toRadians(0));

    // control points for the curved paths
    public static Pose shootPreloadControl1 = new Pose(144 - 73.4354, 144 - 14.3792);
    public static Pose shootPreloadControl2 = new Pose(144 - 84.8638, 144 - 70.5169);
    public static Pose shootPreloadControl3 = new Pose(144 - 83.1292, 144 - 24.8292);
    public static Pose gardenControl1 = new Pose(144 - 80.79, 144 - 137.7846);
    public static Pose gardenControl2 = new Pose(144 - 111.2385, 144 - 129.7362);
    public static Pose endControl1 = new Pose(144 - 132.2538, 144 - 101.4423);

    // Heading of attack for robot to get the garden balls (intead of just straight ramming)
    public static double gardenTiltRad = Math.toRadians(192);

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootPreloadPose,
                new Pose[]{shootPreloadControl1, shootPreloadControl2, shootPreloadControl3},
                gardenPose,
                new Pose[]{gardenControl1, gardenControl2},
                gardenTiltRad,
                shootGardenPose,
                endPose,
                new Pose[]{endControl1}
        );
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Configurable
@Autonomous(name = "BlueFarAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueFarAuto extends BozoAuto {
    public static Pose START_POSE = new Pose(125.8792, 8.4354, Math.toRadians(90));
    public static Pose SHOOT_PRELOAD_POSE = new Pose(82.3408, 132, Math.toRadians(270));
    public static Pose GARDEN_POSE = new Pose(135.3615, 137.07, Math.toRadians(0));
    public static Pose SHOOT_GARDEN_POSE = new Pose(82.2785, 132, Math.toRadians(270));
    public static Pose PARK_POSE = new Pose(130.3962, 34.4623, Math.toRadians(180));

    public static Pose SHOOT_PRELOAD_CONTROL_1 = new Pose(73.4354, 14.3792);
    public static Pose SHOOT_PRELOAD_CONTROL_2 = new Pose(84.8638, 70.5169);
    public static Pose SHOOT_PRELOAD_CONTROL_3 = new Pose(83.1292, 24.8292);
    public static Pose GARDEN_CONTROL_1 = new Pose(80.79, 139.7846);
    public static Pose GARDEN_CONTROL_2 = new Pose(111.2385, 138.7362);
    public static Pose PARK_CONTROL_1 = new Pose(132.2538, 101.4423);

    // tilt for garden sucking
    public static double GARDEN_TILT_RAD = Math.toRadians(12);

    public static AutoConfig route() {
        return new AutoConfig(
                START_POSE,
                SHOOT_PRELOAD_POSE,
                new Pose[]{SHOOT_PRELOAD_CONTROL_1, SHOOT_PRELOAD_CONTROL_2, SHOOT_PRELOAD_CONTROL_3},
                GARDEN_POSE,
                new Pose[]{GARDEN_CONTROL_1, GARDEN_CONTROL_2},
                GARDEN_TILT_RAD,
                SHOOT_GARDEN_POSE,
                new Pose[]{},
                true,
                null, null, null, null, null, null, null,
                PARK_POSE,
                new Pose[]{PARK_CONTROL_1}
        );
    }

    @Override
    protected AutoConfig buildConfig() {
        return route();
    }
}

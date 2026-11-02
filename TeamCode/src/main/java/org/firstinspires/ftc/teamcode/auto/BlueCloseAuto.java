package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Configurable
@Autonomous(name = "BlueCloseAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueCloseAuto extends BozoAuto {
    public static Pose START_POSE = new Pose(86.1934, 136.179, Math.toRadians(270));
    public static Pose SHOOT_PRELOAD_POSE = new Pose(86.1934, 136.179, Math.toRadians(270));
    public static Pose GARDEN_POSE = new Pose(135.5498, 136.2053, Math.toRadians(0));
    public static Pose SHOOT_GARDEN_POSE = new Pose(84.7873, 10.341, Math.toRadians(90));
    public static Pose CLOSE_FLOWER_POSE = new Pose(135.9861, 97.1643, Math.toRadians(0));
    public static Pose SHOOT_CLOSE_FLOWER_POSE = new Pose(84.7783, 10.4474, Math.toRadians(90));
    public static Pose FAR_FLOWER_POSE = new Pose(97.383, 10.379, Math.toRadians(270));
    public static Pose SHOOT_FAR_FLOWER_POSE = new Pose(84.8394, 10.3944, Math.toRadians(90));
    public static Pose PARK_POSE = new Pose(130.48, 31.13, Math.toRadians(24.44));

    public static Pose GARDEN_CONTROL_1 = new Pose(101.5002, 123.508);
    public static Pose GARDEN_CONTROL_2 = new Pose(109.2238, 138.1771);
    public static Pose SHOOT_GARDEN_CONTROL_1 = new Pose(122.5893, 29.7658);
    public static Pose SHOOT_GARDEN_CONTROL_2 = new Pose(93.4437, 35.0448);
    public static Pose CLOSE_FLOWER_CONTROL_1 = new Pose(133.76, 55.81);
    public static Pose CLOSE_FLOWER_CONTROL_2 = new Pose(104.31, 102.77);
    public static Pose FAR_FLOWER_CONTROL_1 = new Pose(98.5475, 34.8862);
    public static Pose SHOOT_FAR_FLOWER_CONTROL_1 = new Pose(98.5634, 34.819);

    public static AutoConfig route() {
        return new AutoConfig(
                START_POSE,
                SHOOT_PRELOAD_POSE,
                new Pose[]{},
                GARDEN_POSE,
                new Pose[]{GARDEN_CONTROL_1, GARDEN_CONTROL_2},
                GARDEN_POSE.heading(), // garden path ends holding the garden heading
                SHOOT_GARDEN_POSE,
                new Pose[]{SHOOT_GARDEN_CONTROL_1, SHOOT_GARDEN_CONTROL_2},
                true,
                CLOSE_FLOWER_POSE,
                new Pose[]{CLOSE_FLOWER_CONTROL_1, CLOSE_FLOWER_CONTROL_2},
                SHOOT_CLOSE_FLOWER_POSE,
                FAR_FLOWER_POSE,
                new Pose[]{FAR_FLOWER_CONTROL_1},
                SHOOT_FAR_FLOWER_POSE,
                new Pose[]{SHOOT_FAR_FLOWER_CONTROL_1},
                PARK_POSE,
                new Pose[]{}
        );
    }

    @Override
    protected AutoConfig buildConfig() {
        return route();
    }
}

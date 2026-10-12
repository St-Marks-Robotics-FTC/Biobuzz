package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Configurable
@Autonomous(name = "BlueCloseAuto", group = "Blue", preselectTeleOp = "BlueTeleOp")
public class BlueCloseAuto extends BozoAuto {
    public static Pose startPose = new Pose(83, 132, Math.toRadians(270));
    public static Pose shootPreloadPose = new Pose(83, 133, Math.toRadians(270));
    public static Pose gardenPose = new Pose(135.5115, 135, Math.toRadians(0));
    public static Pose shootGardenPose = new Pose(83.17, 126.1438, Math.toRadians(270));
    public static Pose endPose = new Pose(133.58, 35.53, Math.toRadians(90));

    public static Pose gardenControl1 = new Pose(103.3538, 123.0762);
    public static Pose gardenControl2 = new Pose(103.8177, 133.3892);
    public static Pose endControl1 = new Pose(114.1715, 94.9185);

    public static AutoConfig route() {
        return new AutoConfig(
                startPose,
                shootPreloadPose,
                new Pose[]{},
                gardenPose,
                new Pose[]{gardenControl1, gardenControl2},
                0, // unused: the garden path follows its tangent
                true,
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

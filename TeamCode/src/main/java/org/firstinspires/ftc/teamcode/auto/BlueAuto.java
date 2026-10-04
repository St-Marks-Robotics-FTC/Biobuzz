package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
public abstract class BlueAuto extends BozoAuto {
    public static Pose startPose = new Pose(84, 6, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(104, 6, Math.toRadians(0));
    public static Pose refuelPose = new Pose(138, 6, Math.toRadians(0));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                intermediatePose,
                refuelPose
        );
    }
}
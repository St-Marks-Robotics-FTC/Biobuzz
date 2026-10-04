package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose startPose = new Pose(60, 6, Math.toRadians(90));
    public static Pose intermediatePose = new Pose(40, 6, Math.toRadians(180));
    public static Pose refuelPose = new Pose(8, 6, Math.toRadians(180));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                intermediatePose,
                refuelPose
        );
    }
}
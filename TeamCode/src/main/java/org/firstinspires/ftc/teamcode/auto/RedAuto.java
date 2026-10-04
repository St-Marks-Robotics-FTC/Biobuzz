package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
public abstract class RedAuto extends BozoAuto {
    public static Pose startPose = new Pose(60, 12, Math.toRadians(270));
    public static Pose refuelPose = new Pose(10, 10, Math.toRadians(225));

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                refuelPose
        );
    }
}
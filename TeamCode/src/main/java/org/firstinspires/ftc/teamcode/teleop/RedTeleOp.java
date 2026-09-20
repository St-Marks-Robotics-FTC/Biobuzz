package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HandoffState;

@TeleOp(name = "RedTeleOp", group = "TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override
    public boolean isBlueTeam() {
        return false;
    }

    @Override
    public Pose getStartPose() {
        return HandoffState.pose;
    }
}

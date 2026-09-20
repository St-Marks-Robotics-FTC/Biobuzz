package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HandoffState;

@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class BlueTeleOp extends BozoTeleOp {
    @Override
    public boolean isBlueTeam() {
        return true;
    }

    @Override
    public Pose getStartPose() {
        return HandoffState.pose;
    }
}

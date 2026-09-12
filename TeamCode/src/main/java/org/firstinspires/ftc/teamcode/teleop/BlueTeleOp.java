/** this is our teleop OpMode for blue team **/

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueTeleOp", group="TeleOp")
public class BlueTeleOp extends BozoTeleOp {
    @Override
    public boolean isBlueTeam() { return true; }
}

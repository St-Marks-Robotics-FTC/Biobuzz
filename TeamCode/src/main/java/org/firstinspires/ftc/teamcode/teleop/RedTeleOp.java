/** this is our teleop OpMode for red team **/

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends BozoTeleOp {
    @Override
    public boolean isBlueTeam() { return false; }
}

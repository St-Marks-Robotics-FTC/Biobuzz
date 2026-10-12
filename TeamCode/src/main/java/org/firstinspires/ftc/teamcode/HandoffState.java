package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class HandoffState {
    public static Pose pose = new Pose(72, 72, Math.toRadians(90));
    // true once an auto has written pose; until then pose is only a default guess that may not match the real robot
    public static boolean fromAuto = false;
}

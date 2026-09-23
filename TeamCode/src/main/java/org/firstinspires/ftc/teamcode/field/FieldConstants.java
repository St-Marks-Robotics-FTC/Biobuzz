package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public class FieldConstants {

    public static class Goal {
        public final String name;
        public final Pose position;      // goal opening location, field inches
        public final double facingRad;   // outward normal of the goal opening

        public Goal(String name, double x, double y, double facingDeg) {
            this.name = name;
            this.position = new Pose(x, y, 0);
            this.facingRad = Math.toRadians(facingDeg);
        }
    }

    public static final Goal BLUE_SOUTH = new Goal("Blue South CELL", 60, 62.6, 270);
    public static final Goal BLUE_NORTH = new Goal("Blue North CELL", 60, 81.4,  90);
    public static final Goal RED_SOUTH  = new Goal("Red South CELL",  84, 62.6, 270);
    public static final Goal RED_NORTH  = new Goal("Red North CELL",  84, 81.4,  90);

    public static final Goal[] BLUE_GOALS = { BLUE_SOUTH, BLUE_NORTH };
    public static final Goal[] RED_GOALS  = { RED_SOUTH, RED_NORTH };

    public static final double MIN_SHOOT_RANGE_IN = 30.0;
    public static final double MAX_SHOOT_RANGE_IN = 45.0;
    public static final double GOAL_ACCEPT_HALF_ANGLE_DEG = 17.5; // 35 deg total cone
}

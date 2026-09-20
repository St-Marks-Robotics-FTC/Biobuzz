package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class FieldGoals {
    public static Pose redGoalNorth = new Pose(24.0, 108.0, Math.toRadians(0.0));
    public static Pose redGoalSouth = new Pose(24.0, 36.0, Math.toRadians(0.0));
    public static Pose blueGoalNorth = new Pose(120.0, 108.0, Math.toRadians(180.0));
    public static Pose blueGoalSouth = new Pose(120.0, 36.0, Math.toRadians(180.0));

    public static Pose[] allianceGoals(boolean isBlue) {
        if (isBlue) {
            return new Pose[]{blueGoalNorth, blueGoalSouth};
        }
        return new Pose[]{redGoalNorth, redGoalSouth};
    }

    public static String[] allianceGoalNames(boolean isBlue) {
        if (isBlue) {
            return new String[]{"blueNorth", "blueSouth"};
        }
        return new String[]{"redNorth", "redSouth"};
    }
}

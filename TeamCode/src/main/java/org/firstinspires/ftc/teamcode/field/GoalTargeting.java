package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public class GoalTargeting {

    public static double distanceTo(Pose robot, FieldConstants.Goal goal) {
        double dx = goal.position.x() - robot.x();
        double dy = goal.position.y() - robot.y();
        return Math.hypot(dx, dy);
    }

    public static boolean isInRange(Pose robot, FieldConstants.Goal goal) {
        double d = distanceTo(robot, goal);
        return d >= FieldConstants.MIN_SHOOT_RANGE_IN && d <= FieldConstants.MAX_SHOOT_RANGE_IN;
    }

    public static double bearingToGoal(Pose robot, FieldConstants.Goal goal) {
        double dx = goal.position.x() - robot.x();
        double dy = goal.position.y() - robot.y();
        return Math.atan2(dy, dx);
    }

    public static double offAxisAngle(Pose robot, FieldConstants.Goal goal) {
        double shotLineFromGoal = Math.atan2(robot.y() - goal.position.y(), robot.x() - goal.position.x());
        return wrapAngle(shotLineFromGoal - goal.facingRad);
    }

    public static boolean isInAngle(Pose robot, FieldConstants.Goal goal) {
        double offAxisDeg = Math.toDegrees(Math.abs(offAxisAngle(robot, goal)));
        return offAxisDeg <= FieldConstants.GOAL_ACCEPT_HALF_ANGLE_DEG;
    }

    public static boolean canScore(Pose robot, FieldConstants.Goal goal) {
        return isInRange(robot, goal) && isInAngle(robot, goal);
    }

    public static double wrapAngle(double radians) {
        double a = radians % (2 * Math.PI);
        if (a <= -Math.PI) a += 2 * Math.PI;
        if (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }
}

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Tunables;

public class ShotEvaluator {
    public static double distance(Pose robot, Pose goal) {
        double dx = goal.x() - robot.x();
        double dy = goal.y() - robot.y();
        return Math.hypot(dx, dy);
    }

    public static double bearingRad(Pose robot, Pose goal) {
        return Math.atan2(goal.y() - robot.y(), goal.x() - robot.x());
    }

    public static double angleErrorRad(Pose robot, Pose goal) {
        return PIDF.wrapAngle(bearingRad(robot, goal) - robot.heading());
    }

    public static double goalErrorRad(Pose robot, Pose goal) {
        double goalToRobot = Math.atan2(robot.y() - goal.y(), robot.x() - goal.x());
        return PIDF.wrapAngle(goalToRobot - goal.heading());
    }

    public static boolean inRange(Pose robot, Pose goal) {
        double d = distance(robot, goal);
        return d >= Tunables.minShootDistance && d <= Tunables.maxShootDistance;
    }

    public static boolean inAngle(Pose robot, Pose goal) {
        double limit = Math.toRadians(Tunables.maxAngleErrorDeg);
        return Math.abs(angleErrorRad(robot, goal)) <= limit
                && Math.abs(goalErrorRad(robot, goal)) <= limit; //Returns true if within 35 degrees
    }

    public static boolean canShoot(Pose robot, Pose goal) {
        return inRange(robot, goal) && inAngle(robot, goal);
    }

    public static boolean aimAllowed(Pose robot, Pose goal) {
        return inRange(robot, goal);
    }
}

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FieldGoals;
import org.firstinspires.ftc.teamcode.Tunables;

public class autoTurn {
    private PIDF headingPid;
    public void AimController() {
        headingPid = new PIDF(Tunables.shootP, Tunables.shootI, Tunables.shootD, Tunables.shootF);
        headingPid.outputLimit = Tunables.aimMaxPower;
        headingPid.integralLimit = Tunables.aimIntegralLimit;
    }

    public void resetAim() {
        headingPid.reset();
    }

    public void syncGains() {
        headingPid.updateTerms(Tunables.shootP, Tunables.shootI, Tunables.shootD, Tunables.shootF);
        headingPid.outputLimit = Tunables.aimMaxPower;
        headingPid.integralLimit = Tunables.aimIntegralLimit;
    }

    public double turnPower(double targetHeadingRad, double currentHeadingRad) {
        syncGains();
        double raw = headingPid.calcAngle(targetHeadingRad, currentHeadingRad);
        return Range.clip(raw, -Tunables.aimMaxPower, Tunables.aimMaxPower);
    }

    public boolean onTarget(double targetHeadingRad, double currentHeadingRad) {
        double err = Math.abs(PIDF.wrapAngle(targetHeadingRad - currentHeadingRad));
        return err <= Math.toRadians(Tunables.aimToleranceDeg);
    }

    // To shoot or not to shoot

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

    public static boolean canTurn(Pose current, Pose goal, double max) {
        if ((goal.y() > 108) && (current.y() > 108 + 12)){ //108 is the midway point, 12 is the amount space needed to shoot
            if ((inRange(current, goal)) && (bearingRad(current, goal) < Math.toRadians(max))){
                return true;
            }else{
                return false;
            }
        }else{
            return false;
        }
    }

    public Pose closestGoal(Pose currentPose, boolean isBlue){
        Pose[] allianceGoals = FieldGoals.allianceGoals(isBlue);
        Pose goalNorth = allianceGoals[0];
        Pose goalSouth = allianceGoals[1];
        double distance1 = distance(currentPose, goalNorth);
        double distance2 = distance(currentPose, goalSouth);
        if (distance1 > distance2){
            return new Pose(goalNorth.x(),goalNorth.y(),goalNorth.heading());
        }else if (distance1 < distance2){
            return new Pose(goalSouth.x(),goalSouth.y(),goalSouth.heading());
        }else{
            return new Pose(goalNorth.x(),goalNorth.y(),goalNorth.heading());
        }
    }
    public double getAutoTurn(Pose currentPose, boolean isBlueTeam) {
        if (isBlueTeam){ //If we are on the blue team
            Pose closestGoal = closestGoal(currentPose, isBlueTeam);
            if ((closestGoal.y() + 12 <= currentPose.y())
                    && inRange(currentPose, closestGoal) &&
                    (bearingRad(currentPose, closestGoal) < Math.toRadians(Tunables.maxAngleErrorDeg))){
                return angleErrorRad(currentPose, closestGoal);
            }else{
                return Double.NaN;
            }
        }
        else{
            Pose closestGoal = closestGoal(currentPose, isBlueTeam);
            if ((closestGoal.y() + 12 <= currentPose.y())
                    && inRange(currentPose, closestGoal) &&
                    (bearingRad(currentPose, closestGoal) < Math.toRadians(Tunables.maxAngleErrorDeg))){
                return angleErrorRad(currentPose, closestGoal);
            }else{
                return Double.NaN;
            }
        }
    }
}

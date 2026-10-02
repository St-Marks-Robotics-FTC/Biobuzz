package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tunables;

public class Field {

    private boolean isBlueTeam;

    public Field(boolean isBlueTeam) {
        this.isBlueTeam = isBlueTeam;
    }

    public ScoringData getScoringData(boolean isAudienceUp, Pose robotPose) {
        Pose goalPose = getGoalPose(isAudienceUp);
        double goalX = goalPose.x();
        double goalY = goalPose.y();
        double goalHeading = goalPose.heading();

        double robotX = robotPose.x();
        double robotY = robotPose.y();

        double dY = goalY - robotY;
        double dX = goalX - robotX;

        double angleToGoal = AngleUnit.normalizeRadians(Math.atan2(dY, dX));
        double dToGoal = Math.sqrt(dY * dY + dX * dX);
        boolean canScore = Math.abs(goalHeading - angleToGoal) < Tunables.maxScoringAngle;

        return new ScoringData(canScore, dToGoal, angleToGoal);
    }

    private Pose getGoalPose(boolean isAudienceUp) {
        if (isBlueTeam) {
            if (isAudienceUp) {
                return Tunables.blueAudienceGoalPose;
            } else {
                return Tunables.blueScoringGoalPose;
            }
        } else {
            if (isAudienceUp) {
                return Tunables.redAudienceGoalPose;
            } else {
                return Tunables.redScoringGoalPose;
            }
        }
    }

    public class ScoringData {
        private double distance; // distance to goal in inches
        private double angle; // angle to goal in radians
        private boolean canScore; // whether

        public ScoringData(boolean canScore, double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        public double distance() {
            return distance;
        }

        public double angle() {
            return angle;
        }

        public String toString() {
            return "canScore: " + canScore + " d: " + distance + " a: " + angle;
        }
    }
}

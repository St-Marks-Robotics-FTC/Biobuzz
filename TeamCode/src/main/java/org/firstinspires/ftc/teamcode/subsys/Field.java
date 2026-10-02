package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tunables;

public class Field {
    // goal poses
    // headings should be to score a ball, not the direction that the goal faces
    private Pose blueAudienceGoalPose = new Pose(85, 58, Math.toRadians(90));
    private Pose blueScoringGoalPose = new Pose(85, 86, Math.toRadians(270));
    private Pose redAudienceGoalPose = new Pose(59, 58, Math.toRadians(90));
    private Pose redScoringGoalPose = new Pose(59, 86, Math.toRadians(270));

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
                return blueAudienceGoalPose;
            } else {
                return blueScoringGoalPose;
            }
        } else {
            if (isAudienceUp) {
                return redAudienceGoalPose;
            } else {
                return redScoringGoalPose;
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

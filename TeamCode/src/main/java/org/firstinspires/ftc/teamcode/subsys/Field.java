package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;

import com.pedropathing.utils.Angle;
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
        boolean canScore = Angle.smallestDifference(goalHeading, angleToGoal) < Tunables.maxScoringAngle; // headings wrap at 0/2pi

        double idealRPM = calcIdealRPM(dToGoal);

        return new ScoringData(canScore, dToGoal, angleToGoal, idealRPM);
    }

    // true if the audience goal is closer to the robot than the scoring goal; used when the camera hasn't seen the hive yet
    public boolean nearestGoalIsAudience(Pose robotPose) {
        Pose audience = getGoalPose(true);
        Pose scoring = getGoalPose(false);
        double dAudience = Math.hypot(audience.x() - robotPose.x(), audience.y() - robotPose.y());
        double dScoring = Math.hypot(scoring.x() - robotPose.x(), scoring.y() - robotPose.y());
        return dAudience < dScoring;
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

    // take in distance in inches and return ideal flywheel RPM
    public double calcIdealRPM(double dist) {
        // insert regression here
        return 0;
    }

    public class ScoringData {
        private boolean canScore; // whether
        private double distance; // distance to goal in inches
        private double angle; // angle to goal in radians
        private double idealRPM; // ideal RPM for auto shoot

        public ScoringData(boolean canScore, double distance, double angle, double idealRPM) {
            this.canScore = canScore;
            this.distance = distance;
            this.angle = angle;
            this.idealRPM = idealRPM;
        }

        public boolean canScore() { return canScore; }

        public double distance() {
            return distance;
        }

        public double angle() {
            return angle;
        }

        public double idealRPM() {
            return idealRPM;
        }

        public String toString() {
            return "canScore: " + canScore + " d: " + distance + " a: " + angle;
        }
    }
}

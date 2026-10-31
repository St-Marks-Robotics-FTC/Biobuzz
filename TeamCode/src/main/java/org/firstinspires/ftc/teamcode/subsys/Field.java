package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tunables;

public class Field {

    private boolean isBlueTeam;

    public Field(boolean isBlueTeam) {
        this.isBlueTeam = isBlueTeam;
    }

    public ScoringData getScoringData(boolean isAudienceUp, Pose robotPose) {
        if (robotPose == null || robotPose.equals(new Pose(0, 0, 0))) {
            return null; // make sure it's not a bad pose
        }

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

        double idealRPM = calcIdealRPM(dToGoal);

        return new ScoringData(canScore, dToGoal, angleToGoal, idealRPM);
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
        // from Desmos https://www.desmos.com/calculator/vzl5asi9fo
        // 10-31-2026
        double calcRPM = (0.182518 * dist * dist) - (7.65866 * dist) + 3191.20234;
        return Range.clip(calcRPM, 2500, 6000); // keep RPM from going too low
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

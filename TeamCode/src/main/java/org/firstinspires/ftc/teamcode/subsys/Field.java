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

    /*

    blueAudienceRight (dx > 0): (94.7, 57.55)

    redAudienceLeft (dx < 0): (49.0, 57.55)


    blueAudienceLeft (dx < 0): (75, 57.55)

    redAudienceRight (dx > 0): (69, 57.55)

    blueGoalRight (dx > 0): (94.7, 88)

    blueGoalLeft (dx < 0): (75, 88)

    redGoalRight (dx > 0): (69, 88)

    redGoalLeft (dx < 0): (49.0, 88)

     */

    public ScoringData getScoringData(boolean isAudienceUp, Pose robotPose) {
        if (robotPose == null || robotPose.equals(new Pose(0, 0, 0))) {
            return null; // make sure it's not a bad pose
        }
        Goal target = getGoal(isAudienceUp);

        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double goalX = 0;
        double goalY = 0;

        if (robotX > target.getCenterPose().x() && robotX - target.getCenterPose().x() >= 13.5) {
            goalX = target.getLeftPose().x();
            goalY = target.getLeftPose().y();
        }
        else if (robotX < target.getCenterPose().x() && target.getCenterPose().x() - robotX >= 13.5) {
            goalX = target.getRightPose().x();
            goalY = target.getRightPose().y();
        }
        else {
            goalX = target.getCenterPose().x();
            goalY = target.getCenterPose().y();
        }

        double dY = goalY - robotY;
        double dX = goalX - robotX;

        double dFromIdealY = dY - 1.04;
        double dFromIdealX = dX - 0.885;

        if (isAudienceUp) {
            if (dFromIdealY <= 0) {
                goalY -= Tunables.dyMultiplier * dFromIdealY;
            }
            else {
                goalY += Tunables.dyMultiplier * dFromIdealY;
            }
        }
        else {
            if (dFromIdealY <= 0) {
                goalY += Tunables.dyMultiplier * dFromIdealY;
            }
            else {
                goalY -= Tunables.dyMultiplier * dFromIdealY;
            }
        }
        if (dFromIdealX <= 0) {
            goalX += Tunables.dyMultiplier * dFromIdealX;
        }
        else {
            goalX -= Tunables.dyMultiplier * dFromIdealX;
        }

        dY = goalY - robotY;
        dX = goalX - robotX;

        double angleToGoal = AngleUnit.normalizeRadians(Math.atan2(dY, dX));
        double dToGoal = Math.sqrt(dY * dY + dX * dX);
        boolean canScore = angleToGoal > Tunables.maxScoringAngle;

        double idealRPM = getShittyRPM(dToGoal);

        return new ScoringData(canScore, dToGoal, angleToGoal, idealRPM);
    }

    private Goal getGoal(boolean isAudienceUp) {
        if (isBlueTeam) {
            if (isAudienceUp) {
                return new Goal(true,
                        new Pose(75, 57.55),
                        new Pose(94.7, 57.55),
                        new Pose(85, 58));
            } else {
                return new Goal(true,
                        new Pose(75, 88),
                        new Pose(94.7, 88),
                        new Pose(85, 86));
            }
        } else {
            if (isAudienceUp) {
                return new Goal(false,
                        new Pose(49.0, 57.55),
                        new Pose(69, 57.55),
                        new Pose(59, 58));
            } else {
                return new Goal(false,
                        new Pose(49.0, 88),
                        new Pose(69, 88),
                        new Pose(59, 86));
            }
        }
    }

    // take in distance in inches and return ideal flywheel RPM
    public double getShittyRPM(double dist) {
        // from Desmos https://www.desmos.com/calculator/vzl5asi9fo
        // 10-31-2026
        double mathRPM = (0.182518 * dist * dist) - (7.65866 * dist) + 3191.20234;
        double autoRPM = mathRPM * Tunables.autoRPMMultiplier + Tunables.autoRPMAddition;
        return Range.clip(autoRPM, 2500, 4000); // keep RPM from going too low or high
    }

    public class Goal {
        private boolean isBlue;
        private Pose leftPose;
        private Pose rightPose;
        private Pose centerPose;

        Goal(boolean isBlue, Pose leftPose, Pose rightPose, Pose centerPose) {
            this.isBlue = isBlue;
            this.leftPose = leftPose;
            this.rightPose = rightPose;
            this.centerPose = centerPose;
        }

        public boolean getBlue() {
            return isBlue;
        }

        public Pose getRightPose() {
            return rightPose;
        }

        public Pose getLeftPose() {
            return leftPose;
        }
        public Pose getCenterPose() {
            return centerPose;
        }

        public void updateLeftPose(boolean direction, double value) {
            double dir = 0;
            if (direction) {
                dir = 1;
            }
            else {
                dir = -1;
            }
            leftPose = new Pose (leftPose.x() * 1, leftPose.y() * dir * value);
        }

        public void updateRightPose(boolean direction, double value) {
            double dir = 0;
            if (direction) {
                dir = 1;
            }
            else {
                dir = -1;
            }
            rightPose = new Pose (rightPose.x() * 1, rightPose.y() * dir * value);
        }

        public String toString() {
            String teamName = "";
            if (isBlue) {
                teamName = "Blue";
            }
            else {
                teamName = "Red";
            }
            return "Team: " + teamName + " Center Pose: " + centerPose.toString() + " Left Pose: " + leftPose.toString() + " Right Pose: " + rightPose.toString();

        }
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

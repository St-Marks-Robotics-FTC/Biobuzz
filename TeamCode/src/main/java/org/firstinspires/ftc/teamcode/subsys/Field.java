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
    Goal aim points. Each goal has a left post (lower x), a right post
    (higher x) and a center point. Y offsets are measured "behind" the goal, i.e. away from the
    side the robot shoots from: +y for the audience goal (robot below it), -y for the scoring
    goal (robot above it). Both goals of a color share the same offsets, mirrored.

    The audience (y=64) and scoring (y=80) goals are mirrored across y=72, so behindY flips the
    offset direction between them.

    blue: left x=76 (+2 behind), right x=94 (+2 behind), center x=85 (+3 behind)
    red:  left x=50 (+2 behind), right x=68 (+2 behind), center x=59 (+3 behind)
     */

    public ScoringData getScoringData(Pose robotPose) {
        boolean isAudienceUp = (robotPose.y() < 72);
        Goal target = getGoal(isAudienceUp);
        double behindY = isAudienceUp ? 1 : -1; // direction from the robot's side of the goal toward the backboard

        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double centerX = target.getCenterPose().x();

        Pose aimPost;

        if (robotX - centerX >= Tunables.centerThreshold) {
            aimPost = target.getLeftPose(); // robot is to the right, aim at the far (left) post
        } else if (centerX - robotX >= Tunables.centerThreshold) {
            aimPost = target.getRightPose(); // robot is to the left, aim at the far (right) post
        } else { // robot is close enough to the center
            aimPost = target.getCenterPose();
        }

        double goalX = aimPost.x();
        double goalY = aimPost.y();

        double dY = goalY - robotY;
        double dX = goalX - robotX;
        double angleToGoal = AngleUnit.normalizeRadians(Math.atan2(dY, dX));
        double dToGoal = robotPose.distance(aimPost);

        // robot must be on its shooting side of the goal (in front of it, not behind the backboard)
        double centerDistInFront = behindY * (target.getCenterPose().y() - robotY);
        boolean canScore = (Math.abs(centerX - robotX) <= Tunables.maxScoringAbsRelativeX) &&
                (centerDistInFront >= Tunables.minScoringRelativeY) &&
                (centerDistInFront <= Tunables.maxScoringRelativeY);

        double idealRPM = getGuudRPM(robotPose, aimPost);

        return new ScoringData(canScore, dToGoal, angleToGoal, idealRPM);
    }

    private Goal getGoal(boolean isAudienceUp) {
        double baseY = isAudienceUp ? Tunables.audienceY : Tunables.scoringY;
        double behindY = isAudienceUp ? 1 : -1; // +y is behind the audience goal, -y behind the scoring goal
        if (isBlueTeam) {
            // both goals for blue team share the same x values
            return new Goal(true,
                    new Pose(76, baseY + behindY * 2),  // actual x pos: 74in
                    new Pose(94, baseY + behindY * 2),  // actual x pos: 96in
                    new Pose(85, baseY + behindY * 3));
        } else {
            // both goals for red team share the same x values
            return new Goal(false,
                    new Pose(50, baseY + behindY * 2),  // actual x pos: 48in
                    new Pose(68, baseY + behindY * 2),  // actual x pos: 70in
                    new Pose(59, baseY + behindY * 3));
        }
    }

    public double getGuudRPM(Pose robotPose, Pose goalPose) {
        // from Desmos: https://www.desmos.com/calculator/gy5epbwyah
        // 11-18-2026

        double d = robotPose.distance(goalPose);
        double angleToGoal = Math.atan2(Math.abs(robotPose.y() - goalPose.y()), Math.abs(robotPose.x() - goalPose.x()));

        double mathRPM;
        // select our dataset
        if (angleToGoal > Tunables.autoRPMSwitchoffAngle) {
            // shooting pretty directly
            mathRPM = 1.02203 * d * d - 95.34311 * d + 5278.09381;
            // keep it within the regression outputs
            mathRPM = Range.clip(mathRPM, 3000, 3100);
        } else {
            // shallow angle shot
            mathRPM = 14.068 * d + 2287.62051;
            // keep it within the regression outputs
            mathRPM = Range.clip(mathRPM, 2900, 3500);
        }

        return mathRPM * Tunables.autoRPMMultiplier + Tunables.autoRPMAddition;
    }

    private class Goal {
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

        public boolean getIsBlue() {
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

        public String toString() {
            String teamName = isBlue ? "Blue" : "Red";
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

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

    blue: left x=78   (+2 behind), right x=92.7 (-2 behind), center x=85 (+3 behind)
    red:  left x=51.3 (-2 behind), right x=66   ( 0 behind), center x=59 (+3 behind)
     */

    public ScoringData getScoringData(Pose robotPose) {
        if (robotPose == null || robotPose.equals(new Pose(0, 0, 0))) {
            return null; // make sure it's not a bad pose
        }
        boolean isAudienceUp = (robotPose.y() < 72);
        Goal target = getGoal(isAudienceUp);
        double behindY = isAudienceUp ? 1 : -1; // direction from the robot's side of the goal toward the backboard

        double robotX = robotPose.x();
        double robotY = robotPose.y();
        double centerX = target.getCenterPose().x();

        Pose aimPost;
        boolean isCrossCourt = true;
        if (robotX - centerX >= Tunables.centerThreshold) {
            aimPost = target.getLeftPose(); // robot is to the right, aim at the far (left) post
        } else if (centerX - robotX >= Tunables.centerThreshold) {
            aimPost = target.getRightPose(); // robot is to the left, aim at the far (right) post
        } else { // robot is close enough to the center
            aimPost = target.getCenterPose();
            isCrossCourt = false;
        }

        double goalX = aimPost.x();
        double goalY = aimPost.y();

        /*
        if (isCrossCourt) {
            // push the aim point past the far post, away from the robot on both axes, scaled by
            // how far away the robot is. With equal multipliers this only lengthens the shot;
            // dyMultiplier > dxMultiplier swings the aim deeper into the backboard.
            double dX = goalX - robotX;
            double dY = goalY - robotY;
            goalX += Math.signum(dX) * Tunables.dxMultiplier * Math.abs(dX);
            goalY += behindY * Tunables.dyMultiplier * Math.abs(dY);
        }
         */

        double dY = goalY - robotY;
        double dX = goalX - robotX;
        double angleToGoal = AngleUnit.normalizeRadians(Math.atan2(dY, dX));
        double dToGoal = Math.sqrt(dY * dY + dX * dX);
        // robot must be on its shooting side of the goal (in front of it, not behind the backboard)
        double centerDistInFront = behindY * (target.getCenterPose().y() - robotY);
        boolean canScore = (Math.abs(centerX - robotX) <= Tunables.maxScoringAbsRelativeX) &&
                (centerDistInFront >= Tunables.minScoringRelativeY) &&
                (centerDistInFront <= Tunables.maxScoringRelativeY);

        double idealRPM = getShittyRPM(dToGoal);

        return new ScoringData(canScore, dToGoal, angleToGoal, idealRPM);
    }

    private Goal getGoal(boolean isAudienceUp) {
        double baseY = isAudienceUp ? Tunables.audienceY : Tunables.scoringY;
        double behindY = isAudienceUp ? 1 : -1; // +y is behind the audience goal, -y behind the scoring goal
        if (isBlueTeam) {
            // both goals for blue team share the same x values
            return new Goal(true,
                    new Pose(76, baseY + behindY * 2),  // actual pose: 74in
                    new Pose(94, baseY - behindY * 2),  // actual pos: 96in
                    new Pose(85, baseY + behindY * 3)); // actual middle
        } else {
            // both goals for red team share the same x values
            return new Goal(false,
                    new Pose(68, baseY - behindY * 2), // actual pose: 70in
                    new Pose(50, baseY),                    // actual pose: 48in
                    new Pose(59, baseY + behindY * 3));  // actual middle
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

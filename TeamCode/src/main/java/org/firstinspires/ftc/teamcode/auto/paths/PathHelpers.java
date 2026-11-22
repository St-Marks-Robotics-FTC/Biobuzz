package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Small path builders shared by every autonomous. All are static and only read {@link Tunables} when called. **/
public final class PathHelpers {
    private PathHelpers() {}

    /** a straight line {@code from} -> {@code to}, heading turning linearly between them **/
    public static Path plainLine(Pose from, Pose to) {
        return line(from, to).linear(from, to);
    }

    /** {@link #plainLine} capped at {@link Tunables#flowerApproachSpeed}, for the final run into a flower **/
    public static Path creepLine(Pose from, Pose to) {
        return limitSpeed(plainLine(from, to), Tunables.flowerApproachSpeed);
    }

    /** {@code path} capped to {@code speed} (fraction 0-1 of max path speed) for this path only **/
    public static Path limitSpeed(Path path, double speed) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(speed));
    }

    /**
     * Heading override for a path that crosses the field midline: face 90 deg (+y) when driving toward +y, 270 deg
     * (-y) when driving toward -y, hold it until {@link Tunables#crossFieldHoldUntil}, then turn to {@code to}'s heading.
     * Red audience->scoring is +y so 90; Blue is the point reflection so its audience->scoring is -y so 270, which
     * matches the mirror rule with no per-alliance code.
     * Precondition: {@code from.y() != to.y()}.
     */
    public static Path crossFieldHeading(Path path, Pose from, Pose to) {
        double hold = Math.toRadians(to.y() > from.y() ? 90 : 270);
        Pose holdPose = new Pose(from.x(), from.y(), hold);
        return path.heading(Interpolator.piecewise()
                .until(Tunables.crossFieldHoldUntil, Interpolator.constant(holdPose))
                .until(1.0, Interpolator.linear(holdPose, to)));
    }
}

package org.firstinspires.ftc.teamcode.auto.steps;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.BozoAuto;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.function.LongSupplier;

/**
 * Everything an autonomous routine touches while it runs: the robot, the follower, the camera, a clock, and the
 * flywheel's commanded RPM (the one piece of auto state that is not on a subsystem).
 *
 * <p>The OpMode owns the single instance and calls {@link #updateFlywheel()} every loop; steps only read it and call
 * {@link #spinUpFlywheel()} / {@link #shutOff()}.
 */
public final class AutoContext {
    public final Robot robot;
    public final Follower follower;
    public final Vision vision;
    /** monotonic milliseconds; the clock every timed step uses **/
    public final LongSupplier clockMillis = () -> System.nanoTime() / 1_000_000L;

    /** RPM we are currently commanding the flywheel to hold; 0 means the flywheel is off **/
    private double targetRPM = 0;

    public AutoContext(Robot robot, Follower follower, Vision vision) {
        this.robot = robot;
        this.follower = follower;
        this.vision = vision;
    }

    public double targetRPM() {
        return targetRPM;
    }

    /** Commands {@link BozoAuto#SHOOT_RPM}; takes effect on the next {@link #updateFlywheel()}. **/
    public void spinUpFlywheel() {
        targetRPM = BozoAuto.SHOOT_RPM;
    }

    /** Closes the transfer, stops the intake, and stops commanding the flywheel. **/
    public void shutOff() {
        robot.transfer.close();
        robot.intake.off();
        targetRPM = 0;
    }

    /** Re-runs the flywheel PIDF toward {@link #targetRPM()}; MUST be called every loop or RPM never converges. **/
    public void updateFlywheel() {
        robot.flywheel.update(targetRPM, robot.getExpansionHubVoltage());
    }

    /** true once the flywheel is within {@link BozoAuto#FLYWHEEL_RPM_MARGIN} of a nonzero target RPM **/
    public boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.flywheel.getRPM() - targetRPM) <= BozoAuto.FLYWHEEL_RPM_MARGIN;
    }
}

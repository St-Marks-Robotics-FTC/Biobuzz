package org.firstinspires.ftc.teamcode.auto.steps;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.BozoAuto;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.util.function.LongSupplier;

/** Robot, follower, camera, clock, and the flywheel's commanded RPM; one instance shared by all steps. **/
public final class AutoContext {
    public final Robot robot;
    public final Follower follower;
    public final Vision vision;
    /** monotonic milliseconds **/
    public final LongSupplier clockMillis = () -> System.nanoTime() / 1_000_000L;

    /** RPM we are commanding; 0 means off **/
    private double targetRPM = 0;

    public AutoContext(Robot robot, Follower follower, Vision vision) {
        this.robot = robot;
        this.follower = follower;
        this.vision = vision;
    }

    public double targetRPM() {
        return targetRPM;
    }

    /** Commands {@link BozoAuto#SHOOT_RPM}. **/
    public void spinUpFlywheel() {
        targetRPM = BozoAuto.SHOOT_RPM;
    }

    /** Closes the transfer, stops the intake and flywheel. **/
    public void shutOff() {
        robot.pollenLaunch.transfer.close();
        robot.pollenLaunch.intake.off();
        targetRPM = 0;
    }

    /** Re-runs the flywheel PIDF; must be called every loop. **/
    public void updateFlywheel() {
        robot.pollenLaunch.flywheel.update(targetRPM, robot.getExpansionHubVoltage());
    }

    /** true once within {@link BozoAuto#FLYWHEEL_RPM_MARGIN} of a nonzero target **/
    public boolean isFlywheelWithinMargin() {
        return targetRPM > 0 && Math.abs(robot.pollenLaunch.flywheel.getRPM() - targetRPM) <= BozoAuto.FLYWHEEL_RPM_MARGIN;
    }
}

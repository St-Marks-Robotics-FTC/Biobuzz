package org.firstinspires.ftc.teamcode.auto.steps;

import static org.firstinspires.ftc.teamcode.auto.core.Steps.instant;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.sequence;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.timeout;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.waitMillis;
import static org.firstinspires.ftc.teamcode.auto.core.Steps.waitUntil;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.BozoAuto;
import org.firstinspires.ftc.teamcode.auto.core.Step;
import org.firstinspires.ftc.teamcode.auto.paths.FlowerSite;

/**
 * This robot's autonomous vocabulary: drive, shoot, collect a flower, refuel, park. Each method returns a reusable
 * {@link Step}; routes are built by sequencing them.
 *
 * <p>Every hardware action of an autonomous lives here, once. If a flower pickup should behave differently, change it
 * in {@link #collectFlower} and every route follows.
 */
public final class RobotSteps {
    private final AutoContext ctx;
    private final Robot robot;

    public RobotSteps(AutoContext ctx) {
        this.ctx = ctx;
        this.robot = ctx.robot;
    }

    // ======================================================================== primitives

    /** Follows {@code path}; finishes when the follower is no longer busy (which includes the heading settling). **/
    public Step drive(String label, Path path) {
        return new DriveStep(label, ctx, path);
    }

    /** {@link #drive} that also gives up after {@link BozoAuto#MAX_TRAVEL_TO_REFUEL_TIME} (stuck on a ball or the wall). **/
    public Step creep(String label, Path path) {
        return timeout(ctx.clockMillis, () -> BozoAuto.MAX_TRAVEL_TO_REFUEL_TIME, drive(label, path));
    }

    /** Starts commanding {@link BozoAuto#SHOOT_RPM}. **/
    public Step spinUpFlywheel() {
        return instant("spin up flywheel", ctx::spinUpFlywheel);
    }

    /** Waits until the flywheel is within margin, launches, and finishes when the launch sequence ends. **/
    public Step shoot() {
        return sequence("shoot",
                waitUntil("wait for flywheel", ctx::isFlywheelWithinMargin), // never feed balls below speed
                instant("launch", robot::startLaunch),
                waitUntil("feeding", () -> !robot.isLaunching()));
    }

    /** Lowers the flower arm. **/
    public Step flowerDown() {
        return instant("flower down", robot.flower::goDown);
    }

    // ======================================================================== macros

    /** Drives {@code path}, then {@link #shoot}s. **/
    public Step travelShoot(String label, Path path) {
        return sequence(label, drive("travel", path), shoot());
    }

    /**
     * Collects one flower. Flower arm down and intake on, drive {@code approach} to the flower's intermediate pose,
     * creep in, shake while sitting on the flower for {@link BozoAuto#FLOWER_REFUEL_DURATION_MILLIS}, back out to the
     * intermediate pose with the shake still running, then arm up.
     *
     * @param approach path from wherever we are to {@code site.intermediatePose}; it differs per call site
     */
    public Step collectFlower(FlowerSite site, Path approach) {
        return sequence("collect " + site.name,
                instant("intake on, flower down", () -> {
                    robot.intake.forward();
                    robot.flower.goDown();
                }),
                drive("to intermediate", approach),
                creep("creep in", site.creepIn),
                instant("shake", robot.flower::startShake),
                waitMillis(ctx.clockMillis, () -> BozoAuto.FLOWER_REFUEL_DURATION_MILLIS),
                drive("back out", site.backOut),
                instant("flower up", robot.flower::goUp));
    }

    /** Creeps along {@code path} onto the refuel pose, then intakes for {@link BozoAuto#REFUEL_DURATION_MILLIS}. **/
    public Step refuel(Path path) {
        return sequence("refuel",
                creep("creep to refuel pose", path),
                instant("intake on", robot.intake::forward),
                waitMillis(ctx.clockMillis, () -> BozoAuto.REFUEL_DURATION_MILLIS));
    }

    /** Everything off, then drive {@code path} to the park pose. **/
    public Step park(Path path) {
        return sequence("park",
                instant("shut off", ctx::shutOff),
                drive("to park", path));
    }

    // ======================================================================== implementations

    private static final class DriveStep implements Step {
        private final String label;
        private final AutoContext ctx;
        private final Path path;

        DriveStep(String label, AutoContext ctx, Path path) {
            if (path == null) throw new IllegalArgumentException("drive '" + label + "' has no path");
            this.label = label;
            this.ctx = ctx;
            this.path = path;
        }

        @Override public String label() { return label; }
        @Override public void start() { ctx.follower.follow(path); }
        @Override public boolean update() { return !ctx.follower.isBusy(); }
    }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.utils.Timer;
import org.firstinspires.ftc.teamcode.FieldGoals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.AimController;
import org.firstinspires.ftc.teamcode.subsys.Shooter;
import org.firstinspires.ftc.teamcode.subsys.ShotEvaluator;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

public abstract class BozoTeleOp extends OpMode {
    protected abstract boolean isBlueTeam();
    protected abstract Pose getStartPose();
    private Robot robot;
    private Follower follower;
    private Timer loopTimer;
    private TelemetryManager telemetryM;
    private AimController aim;
    private Shooter shooter;
    private boolean isRobotCentric = false;
    private int lastAimIndex = -1;
    private boolean lastA = false;

    @Override
    public void init() {
        loopTimer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(getStartPose());
        aim = new AimController();
        shooter = new Shooter();
        telemetryM.debug("init " + loopTimer.get(TimeUnit.MILLISECONDS) + "ms");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        aim.reset();
        shooter.reset();
        lastAimIndex = -1;
        lastA = false;
    }

    @Override
    public void loop() {
        loopTimer.reset();
        boolean aNow = gamepad1.a;
        if (aNow && !lastA) {
            isRobotCentric = !isRobotCentric;
        }
        lastA = aNow;
        double slow = Range.scale(gamepad1.left_trigger, 0.0, 1.0, 1.0, Tunables.slowModeMin);
        Pose robotPose = follower.pose();
        Pose[] goals = FieldGoals.allianceGoals(isBlueTeam());
        String[] names = FieldGoals.allianceGoalNames(isBlueTeam());
        Pose goal0 = goals[0];
        Pose goal1 = goals[1];
        double dist0 = ShotEvaluator.distance(robotPose, goal0);
        double dist1 = ShotEvaluator.distance(robotPose, goal1);
        double err0Deg = Math.toDegrees(ShotEvaluator.angleErrorRad(robotPose, goal0));
        double err1Deg = Math.toDegrees(ShotEvaluator.angleErrorRad(robotPose, goal1));

        double face0Deg = Math.toDegrees(ShotEvaluator.goalErrorRad(robotPose, goal0));
        double face1Deg = Math.toDegrees(ShotEvaluator.goalErrorRad(robotPose, goal1));

        boolean range0 = ShotEvaluator.inRange(robotPose, goal0);
        boolean range1 = ShotEvaluator.inRange(robotPose, goal1);

        boolean can0 = ShotEvaluator.canShoot(robotPose, goal0);
        boolean can1 = ShotEvaluator.canShoot(robotPose, goal1);

        boolean want0 = gamepad1.left_bumper;
        boolean want1 = gamepad1.right_bumper;
        Pose aimGoal = null;
        int aimIndex = -1;
        if (want0 && want1) {
            if (range0 && range1) {
                if (Math.abs(err0Deg) <= Math.abs(err1Deg)) {
                    aimGoal = goal0;
                    aimIndex = 0;
                } else {
                    aimGoal = goal1;
                    aimIndex = 1;
                }
            } else if (range0) {
                aimGoal = goal0;
                aimIndex = 0;
            } else if (range1) {
                aimGoal = goal1;
                aimIndex = 1;
            }
        } else if (want0 && range0) {
            aimGoal = goal0;
            aimIndex = 0;
        } else if (want1 && range1) {
            aimGoal = goal1;
            aimIndex = 1;
        }
        boolean aiming = aimGoal != null;
        if (aiming && aimIndex != lastAimIndex) {
            aim.reset();
        }
        if (!aiming && lastAimIndex != -1) {
            aim.reset();
        }
        lastAimIndex = aimIndex;
        double turn;
        double targetHeading = 0.0;
        boolean onTarget = false;
        if (aiming) {
            targetHeading = ShotEvaluator.bearingRad(robotPose, aimGoal);
            turn = aim.turnPower(targetHeading, robotPose.heading());
            onTarget = aim.onTarget(targetHeading, robotPose.heading());
        } else {
            turn = -gamepad1.right_stick_x * Tunables.turnRateMultiplier * slow;
        }
        double tx = -gamepad1.left_stick_y * slow;
        double ty = -gamepad1.left_stick_x * slow;
        if (isRobotCentric) {
            follower.manual(tx, ty, turn);
        } else {
            double flip = isBlueTeam() ? -1.0 : 1.0;
            DrivePowers powers = ManualDrive.fieldCentric(tx * flip, ty * flip, turn, robotPose.heading());
            follower.manual(powers);
        }
        boolean shootHeld = gamepad1.right_trigger > 0.5;
        boolean aimReady = ShotEvaluator.aimAllowed(robotPose, aimGoal);
        shooter.setShooting(shootHeld);
        shooter.tryFire(aimReady);
        shooter.update();

        follower.update();
        Pose freshPose = follower.pose();

        telemetryM.addData("pose x", freshPose.x());
        telemetryM.addData("pose y", freshPose.y());
        telemetryM.addData("pose deg", Math.toDegrees(freshPose.heading()));
        telemetryM.addData(names[0] + " dist", dist0);
        telemetryM.addData(names[0] + " aimErr", err0Deg);
        telemetryM.addData(names[0] + " faceErr", face0Deg);
        telemetryM.addData(names[0] + " inRange", range0);
        telemetryM.addData(names[0] + " canShoot", can0);
        telemetryM.addData(names[1] + " dist", dist1);
        telemetryM.addData(names[1] + " aimErr", err1Deg);
        telemetryM.addData(names[1] + " faceErr", face1Deg);
        telemetryM.addData(names[1] + " inRange", range1);
        telemetryM.addData(names[1] + " canShoot", can1);
        telemetryM.addData("aiming", aiming);
        telemetryM.addData("onTarget", onTarget);
        telemetryM.addData("shooter rpm", shooter.getCurrentRpm());
        telemetryM.addData("shooter ready", shooter.isAtSpeed());
        telemetryM.addData("loop ms", loopTimer.get(TimeUnit.MILLISECONDS));

        if (Tunables.isDebugging) {
            telemetryM.debug("target " + Math.toDegrees(targetHeading));
            telemetryM.debug("centric " + isRobotCentric);
        }
        telemetryM.update(telemetry);
    }
}

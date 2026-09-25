package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.frontRightName.set("frontRight");
                c.backLeftName.set("backLeft");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.922341684656819);
        c.yPodOffset.set(3.6721282117948757);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // PASTE YOUR OWN "Java" tab output from the Foresight AutoTune procedure here.
    // The values below are PedroPathing's own doc example, not this robot's real numbers --
    // run Tuning.foresightTuner() on the real robot and replace this with that output.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.21503157880699678);
                Controller secondaryTranslationalForward = Controller.proportional(0.0794484226195786);
                Controller primaryTranslationalLateral = Controller.proportional(0.3473065123332834);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12832047610622277);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018045337509493928));
                c.brake.set(Controller.proportionalFeedforward(0.015338536883069838));

                c.headingFeedback.set(Controller.proportional(2.1239058413455254));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03623458805188862, 0.009869025324162737));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03455284940917896, 0.14132737182904553));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.004243572376591928, 5.841031988782842E-4));

                c.maxAchievableStrafeVelocity.set(46.61974609410268);
                c.naturalForwardDeceleration.set(32.41950525436732);
                c.naturalStrafeDeceleration.set(32.845809731388336);
            }
    );
}
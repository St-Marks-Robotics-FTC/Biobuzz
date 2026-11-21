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

                c.powerThreshold.set(0.05); // increase motor caching

                c.manualBrakeMode.set(true);
            }
    );

    // tuned 10-04-2026
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(1.98); // taken from ruler 10-30-2026
        c.yPodOffset.set(-3.7);  // taken from ruler 10-30-2026
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // tuned 11-21-2026
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.38553348405752125);
                Controller secondaryTranslationalForward = Controller.proportional(0.142444320714832);
                Controller primaryTranslationalLateral = Controller.proportional(0.5520421403520164);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20396482002240976);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017600939138013603));
                c.brake.set(Controller.proportionalFeedforward(0.014960798267311563));

                c.headingFeedback.set(Controller.proportional(3.7635771228552604));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.052094969675594294, 0.00675970361732308));

                c.linearBrakeCoefficients.set(Matrix.diag(0.059921341451838445, 0.07373992860342533));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.003508793402875559, 0.0027602564977998287));

                c.maxAchievableForwardVelocity.set(59.065821217608956);
                c.maxAchievableStrafeVelocity.set(30.61332927639438);
                c.naturalForwardDeceleration.set(23.805596635175);
                c.naturalStrafeDeceleration.set(47.72039526163324);

                c.maxPathSpeed.set(1.0);
            }
    );
}
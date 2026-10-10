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
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.072);    // taken from Onshape 10-04-2026
        c.yPodOffset.set(-3.64932); // taken from Onshape 10-04-2026
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // tuned 10-04-2026
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22233000197218108);
                Controller secondaryTranslationalForward = Controller.proportional(0.08214499496165555);
                Controller primaryTranslationalLateral = Controller.proportional(0.33143586930130875);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12245669757730018);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012076073252877517));
                c.brake.set(Controller.proportionalFeedforward(0.010264662264945889));

                c.headingFeedback.set(Controller.proportional(3.0420800202658467));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.038024109766697727, 0.00804985174818463));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0738998340860344, 0.04427225148118583));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019959651732527764, 0.0023078620606778273));

                c.maxPathSpeed.set(0.8);
                c.maxAchievableForwardVelocity.set(80.66675078131117);
                c.maxAchievableStrafeVelocity.set(56.530850251319045);
                c.naturalForwardDeceleration.set(33.03196659659963);
                c.naturalStrafeDeceleration.set(58.61417322721197);
            }
    );}
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    /* Follower needs three things: a Localizer, a Drivetrain and an Algorithm. The first two are
     * wired below. The Algorithm (Foresight) needs twelve tuned values we do not have yet, so
     * create() still refuses rather than driving on invented gains.
     *
     * Order to fix this, each step feeding the next:
     *   1. run "Pinpoint Tuner"  -> paste its output over localizerConfig
     *   2. run "Mecanum Tuner"   -> paste its output into driveConfig
     *   3. run "Foresight Tuner" -> paste its output into a foresightConfig, then finish create()
     * All three are registered in Tuning.java and show up in the Panels tuning page. */

    public static Follower create(HardwareMap h) {
        throw new IllegalStateException(
                "Pedro is not configured yet: no Foresight algorithm. Run the Pinpoint, Mecanum "
                        + "and Foresight tuners from the Panels tuning page, paste their output "
                        + "into Constants, then return new Follower(localizer(h), drivetrain(h), "
                        + "new Foresight(foresightConfig)) here.");
    }

    public static Localizer localizer(HardwareMap h) {
        return new com.pedropathing.revhub.localizers.PinpointLocalizer(h, localizerConfig);
    }

    public static Drivetrain drivetrain(HardwareMap h) {
        return new Mecanum(h, driveConfig);
    }

    /** PLACEHOLDER. Every number here is a guess; replace wholesale with Pinpoint Tuner output. **/
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo"); // matches the hardware map, see README
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0); // TODO tuner
        c.yPodOffset.set(0.0); // TODO tuner
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO tuner
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO tuner
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.frontRightName.set("frontRight");
                c.backLeftName.set("backLeft");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );
}

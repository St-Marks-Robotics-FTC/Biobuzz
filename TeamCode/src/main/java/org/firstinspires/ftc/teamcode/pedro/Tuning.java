package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

/** Tuners registered here appear on the Panels tuning page. Run them in this order: Pinpoint and
 *  Mecanum first (they need nothing), then Foresight, which drives the robot using the localizer
 *  and drivetrain the first two produced. Paste each one's output into Constants. **/
public class Tuning {

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    // Tests needs a working Foresight algorithm, so it stays off the page until Constants.create()
    // is finished. Add it back once foresightConfig is filled in.
}

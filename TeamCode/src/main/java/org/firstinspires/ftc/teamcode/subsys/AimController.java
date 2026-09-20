package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Tunables;

public class AimController {
    private final PIDF headingPid;
    public AimController() {
        headingPid = new PIDF(Tunables.aimP, Tunables.aimI, Tunables.aimD, Tunables.aimF);
        headingPid.outputLimit = Tunables.aimMaxPower;
        headingPid.integralLimit = Tunables.aimIntegralLimit;
    }

    public void reset() {
        headingPid.reset();
    }

    public void syncGains() {
        headingPid.updateTerms(Tunables.aimP, Tunables.aimI, Tunables.aimD, Tunables.aimF);
        headingPid.outputLimit = Tunables.aimMaxPower;
        headingPid.integralLimit = Tunables.aimIntegralLimit;
    }

    public double turnPower(double targetHeadingRad, double currentHeadingRad) {
        syncGains();
        double raw = headingPid.calcAngle(targetHeadingRad, currentHeadingRad);
        return Range.clip(raw, -Tunables.aimMaxPower, Tunables.aimMaxPower);
    }

    public boolean onTarget(double targetHeadingRad, double currentHeadingRad) {
        double err = Math.abs(PIDF.wrapAngle(targetHeadingRad - currentHeadingRad));
        return err <= Math.toRadians(Tunables.aimToleranceDeg);
    }
}

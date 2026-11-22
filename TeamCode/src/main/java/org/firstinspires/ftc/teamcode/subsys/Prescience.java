package org.firstinspires.ftc.teamcode.subsys;

import org.firstinspires.ftc.teamcode.Tunables;

/** like PIDF, but better **/
public class Prescience {
    private double kP; // normal proportional term
    private double kS; // static friction
    private double kV; // dynamic friction

    public Prescience(double kP, double kS, double kV) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
    }

    public void updateTerms(double kP, double kS, double kV) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
    }

    public double calc(double target, double current, double voltage) {
        double voltageCompensation = Tunables.calibrationVoltage / voltage;

        // calculate feedforward based off of friction variables
        double feedforward = kS + kV * target;

        // calculate proportional based off distance
        double proportional = (target - current) * kP;

        // bring it all together
        return (feedforward + proportional) * voltageCompensation;
    }
}

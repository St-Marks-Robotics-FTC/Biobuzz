package org.firstinspires.ftc.teamcode.subsys;

public class PIDF {
    public double kP, kI, kD, kF;
    public double integralLimit = Double.POSITIVE_INFINITY;
    public double outputLimit = Double.POSITIVE_INFINITY;
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        sumError = 0;
        lastError = 0;
        lastTime = 0;
    }

    public void updateTerms(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calc(double target, double current) {
        return calcError(target - current, target);
    }

    public double calcAngle(double target, double current) {
        return calcError(wrapAngle(target - current), target);
    }

    public static double wrapAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private double calcError(double error, double target) {
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = (lastTime == 0) ? 0 : (currentTime - lastTime);
        double P = kP * error;
        if (deltaTime > 0) {
            sumError += error * deltaTime;
            if (sumError > integralLimit) sumError = integralLimit;
            else if (sumError < -integralLimit) sumError = -integralLimit;
        }
        double I = kI * sumError;
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double D = kD * derivative;
        double F = kF * target;
        lastError = error;
        lastTime = currentTime;
        double out = P + I + D + F;
        if (out > outputLimit) return outputLimit;
        if (out < -outputLimit) return -outputLimit;
        return out;
    }
}

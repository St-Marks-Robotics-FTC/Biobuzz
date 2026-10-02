package org.firstinspires.ftc.teamcode.subsys;

import org.firstinspires.ftc.teamcode.Tunables;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.math.Pose;
public class Shooter {
    private double currentRpm = 0.0;
    private boolean shooting = false;
    private int shotsFired = 0;
    private long lastNs = 0;
    private long lastShotNs = 0;

    public void setShooting(boolean wantShoot) {
        if (wantShoot && lastNs == 0) {
            lastNs = System.nanoTime();
        }
        shooting = wantShoot;
        if (!wantShoot && currentRpm < 1.0) {
            lastNs = 0;
        }
    }

    public void update() {
        long now = System.nanoTime();
        if (lastNs == 0) {
            lastNs = now;
            return;
        }
        double dt = (now - lastNs) / 1e9;
        lastNs = now;
        if (dt < 0.0) {
            dt = 0.0;
        }
        if (dt > 0.1) {
            dt = 0.1;
        }
        double target = shooting ? Tunables.shooterTargetRpm : 0.0;
        double maxStep = Tunables.shooterRampRpmPerSec * dt;
        double err = target - currentRpm;

        if (Math.abs(err) <= maxStep) {
            currentRpm = target;
        } else {
            currentRpm += Math.signum(err) * maxStep;
        }
    }

    public boolean tryFire(boolean aimReady) {
        if (!shooting || !aimReady || !isAtSpeed()) {
            return false;
        }
        long now = System.nanoTime();
        double interval = Tunables.shooterFireIntervalSec;
        if (lastShotNs != 0 && (now - lastShotNs) / 1e9 < interval) {
            return false;
        }


        lastShotNs = now;
        shotsFired++;
        return true;
    }

    public boolean isAtSpeed() {
        return Math.abs(currentRpm - Tunables.shooterTargetRpm) <= Tunables.shooterToleranceRpm;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    public boolean isShooting() {
        return shooting;
    }

    public void reset() {
        currentRpm = 0.0;
        shooting = false;
        shotsFired = 0;
        lastNs = 0;
        lastShotNs = 0;
    }

    // Shooter PIDF
    public double kP, kI, kD, kF;
    public double integralLimit = Double.POSITIVE_INFINITY;
    public double outputLimit = Double.POSITIVE_INFINITY;
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public void PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void resetPIDF() {
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

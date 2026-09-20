package org.firstinspires.ftc.teamcode.subsys;

import org.firstinspires.ftc.teamcode.Tunables;

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
}

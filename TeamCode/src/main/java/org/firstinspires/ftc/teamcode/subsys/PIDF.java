/** general PIDF controller to use for pointing robot towards goal april tag **/
// taken from: https://roboftc.github.io/code/pidf.html

package org.firstinspires.ftc.teamcode.subsys;

public class PIDF {
    // PIDF coefficients
    public double kP, kI, kD, kF;

    // State variables
    private double sumError = 0;
    private double lastError = 0;
    private long lastTimeNanos = 0; // 0: no measurement yet

    public PIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        sumError = 0;
        lastError = 0;
        lastTimeNanos = 0;
    }

    public void updateTerms(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * @param measuredAtNanos {@link System#nanoTime()} when {@code current} was measured; I and D only advance when
     *                        this moves forward, so re-running on the same measurement never divides by zero
     */
    public double calc(double target, double current, long measuredAtNanos) {
        double deltaTime = (lastTimeNanos == 0) ? 0 : (measuredAtNanos - lastTimeNanos) / 1e9;

        double error = target - current;

        // Proportional
        double P = kP * error;

        // Integral
        if (deltaTime > 0) {
            sumError += error * deltaTime;
        }
        double I = kI * sumError;

        // Derivative
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double D = kD * derivative;

        // Feedforward
        double F = kF * target;

        // Store for next loop
        lastError = error;
        lastTimeNanos = measuredAtNanos;

        // Total output
        return P + I + D + F;
    }
}

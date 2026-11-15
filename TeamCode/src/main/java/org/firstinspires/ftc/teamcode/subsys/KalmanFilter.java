package org.firstinspires.ftc.teamcode.subsys;
public class KalmanFilter {
    private static final double MIN_VARIANCE = 1e-7; // no dividing by zero

    private double state;
    private double variance;

    public KalmanFilter(double initialState, double initialVariance) {
        reset(initialState, initialVariance);
    }

    public void reset(double initialState, double initialVariance) {
        state = initialState;
        variance = Math.max(MIN_VARIANCE, initialVariance);
    }

    // Move the estimate by a known input and add noise.
    public void predict(double control, double processNoise) {
        state += control;
        variance += Math.max(0, processNoise);
    }


    // How far the measurement is from prediction.
    public double innovation(double measurement) {
        return measurement - state;
    }

    // Total uncertainty of that change
    public double innovationVariance(double measurementVariance) {
        return variance + Math.max(MIN_VARIANCE, measurementVariance);
    }

    // True when the measurement is farther from the prediction reasonable
    public boolean isOutlier(double measurement, double measurementVariance, double sigmas) {
        return Math.abs(innovation(measurement))
                > sigmas * Math.sqrt(innovationVariance(measurementVariance));
    }

    public void update(double measurement, double measurementVariance) {
        double r = Math.max(MIN_VARIANCE, measurementVariance);
        double k = variance / (variance + r);

        state += k * (measurement - state);
        variance *= (1 - k);
        if (variance < MIN_VARIANCE) variance = MIN_VARIANCE;
    }

    // Admit we were overconfident
    public void inflateVariance(double factor) {
        if (factor > 0 && !Double.isInfinite(factor) && !Double.isNaN(factor)) {
            variance *= factor;
        }
    }

    public void setState(double newState) {
        state = newState;
    }

    public double getState() {
        return state;
    }

    public double getVariance() {
        return variance;
    }

    public double getStdDev() {
        return Math.sqrt(variance);
    }
}

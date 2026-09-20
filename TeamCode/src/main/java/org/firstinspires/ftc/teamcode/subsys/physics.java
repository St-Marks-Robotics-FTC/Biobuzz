/** fixed-angle shot solver: robot pose -> flywheel RPM, with quadratic drag **/
// geometry from the BIOBUZZ competition manual TU01, section 9.6

package org.firstinspires.ftc.teamcode.subsys;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Tunables;

public class physics {
    private static final double G = 9.80665;
    private static final double RHO = 1.225; // air density, kg/m^3
    private static final double IN = 0.0254;

    /* The raised CELL opening runs 53.5 in -> 65.6 in above the TILES and measures 20 in x 14 in.
     * 65.6 - 53.5 = 12.1 = 14*sin(60), so the opening plane sits 60 deg off horizontal and its
     * normal points outward and 30 deg above horizontal. A ball entering along that normal, i.e.
     * descending at 30 deg, sees the whole opening. */
    public static final double APERTURE_CENTER_IN = 59.55;
    public static final double APERTURE_LOW_IN = 53.5;
    public static final double APERTURE_HIGH_IN = 65.6;
    public static final double APERTURE_NORMAL_DEG = 30.0;

    /* Pivot is 43.95 in above the TILES and the two HIVES are 25.5 in apart. The aperture centre
     * sits 31.2 in out along the arm, which at the 30 deg tip is 15.77 in horizontally from its
     * own pivot. Verify the axis and signs against your field before trusting the Cell poses. */
    private static final double V_MIN = 1.0, V_MAX = 25.0; // muzzle speed search bracket, m/s

    private static final double FIELD_CENTER_IN = 72.0;
    private static final double HIVE_OFFSET_IN = 12.75;
    private static final double CELL_OFFSET_IN = 15.77;

    public enum Cell {
        RED_AUDIENCE(FIELD_CENTER_IN - HIVE_OFFSET_IN, FIELD_CENTER_IN - CELL_OFFSET_IN),
        RED_FAR(FIELD_CENTER_IN - HIVE_OFFSET_IN, FIELD_CENTER_IN + CELL_OFFSET_IN),
        BLUE_AUDIENCE(FIELD_CENTER_IN + HIVE_OFFSET_IN, FIELD_CENTER_IN - CELL_OFFSET_IN),
        BLUE_FAR(FIELD_CENTER_IN + HIVE_OFFSET_IN, FIELD_CENTER_IN + CELL_OFFSET_IN);

        public final double x, y;

        Cell(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /** physical ball properties, straight from the manual; these are facts, not tunables **/
    public enum Ball {
        POLLEN(0.027, 0.0711),
        NECTAR(0.041, 0.0914);

        public final double mass, diameter; // kg, m

        Ball(double mass, double diameter) {
            this.mass = mass;
            this.diameter = diameter;
        }

        public double dragK(double cd) { // drag deceleration is dragK * speed^2
            double area = Math.PI * diameter * diameter / 4.0;
            return 0.5 * RHO * cd * area / mass;
        }
    }

    /* One per launcher. Outtake 1 shoots POLLEN, outtake 2 shoots NECTAR. Each has its own hood,
     * wheel and feed, so nothing is shared: separate angle, exit point, speed line and drag fit.
     * Drag belongs to the ball, but the fit is stored per outtake since the pairing is one to one. */
    public enum Outtake {
        POLLEN(Ball.POLLEN),
        NECTAR(Ball.NECTAR);

        public final Ball ball;

        Outtake(Ball ball) {
            this.ball = ball;
        }

        public double angleDeg() {
            return this == POLLEN ? Tunables.pollenLaunchAngleDeg : Tunables.nectarLaunchAngleDeg;
        }

        public double heightIn() {
            return this == POLLEN ? Tunables.pollenLaunchHeightIn : Tunables.nectarLaunchHeightIn;
        }

        public double offsetIn() {
            return this == POLLEN ? Tunables.pollenLaunchOffsetIn : Tunables.nectarLaunchOffsetIn;
        }

        public double speedPerRpm() {
            return this == POLLEN ? Tunables.pollenSpeedPerRpm : Tunables.nectarSpeedPerRpm;
        }

        public double speedIntercept() {
            return this == POLLEN ? Tunables.pollenSpeedIntercept : Tunables.nectarSpeedIntercept;
        }

        public double dragCoefficient() {
            return this == POLLEN ? Tunables.pollenDragCoefficient : Tunables.nectarDragCoefficient;
        }

        /** muzzle speed this outtake produces at a given RPM **/
        public double speedAtRpm(double rpm) {
            return speedPerRpm() * rpm + speedIntercept();
        }

        /** RPM needed for a muzzle speed; NaN if the speed line has not been calibrated **/
        public double rpmForSpeed(double speed) {
            double slope = speedPerRpm();
            return slope > 1e-9 ? (speed - speedIntercept()) / slope : Double.NaN;
        }
    }

    /** starting guess for speedPerRpm from wheel geometry, before you have measured the line **/
    public static double seedSpeedPerRpm(double wheelDiameterIn, double transfer) {
        return transfer * Math.PI * wheelDiameterIn * IN / 60.0;
    }

    /** a solved shot; check feasible before using rpm **/
    public static class Shot {
        public final boolean feasible;
        public final double rpm;
        public final double ballSpeed;     // m/s at the muzzle
        public final double flightTime;    // seconds
        public final double entryAngleDeg; // positive is descending
        public final double openingFrac;   // fraction of the opening the ball sees, 1.0 is square on
        public final double rangeIn;
        public final double headingRad;    // field heading from robot to cell, NaN if solved by range

        Shot(boolean feasible, double rpm, double ballSpeed, double flightTime, double entryAngleDeg,
             double openingFrac, double rangeIn, double headingRad) {
            this.feasible = feasible;
            this.rpm = rpm;
            this.ballSpeed = ballSpeed;
            this.flightTime = flightTime;
            this.entryAngleDeg = entryAngleDeg;
            this.openingFrac = openingFrac;
            this.rangeIn = rangeIn;
            this.headingRad = headingRad;
        }
    }

    public static Shot solve(Pose robot, Cell cell, Outtake outtake) {
        double dx = cell.x - robot.x();
        double dy = cell.y - robot.y();
        return solve(Math.sqrt(dx * dx + dy * dy), outtake, Math.atan2(dy, dx));
    }

    public static Shot solve(double rangeIn, Outtake outtake) {
        return solve(rangeIn, outtake, Double.NaN);
    }

    private static Shot solve(double rangeIn, Outtake outtake, double headingRad) {
        double d = (rangeIn - outtake.offsetIn()) * IN;
        double dz = (APERTURE_CENTER_IN - outtake.heightIn()) * IN;
        double angle = Math.toRadians(outtake.angleDeg());
        double k = outtake.ball.dragK(outtake.dragCoefficient());
        double[] out = new double[4];

        // arrival height rises monotonically with muzzle speed, so bisection always converges
        double lo = V_MIN, hi = V_MAX;
        for (int i = 0; i < 24; i++) {
            double mid = 0.5 * (lo + hi);
            fly(mid, angle, k, d, out);
            if (out[0] < dz) lo = mid;
            else hi = mid;
        }

        double v = 0.5 * (lo + hi);
        fly(v, angle, k, d, out);

        double entry = Math.toDegrees(Math.atan2(-out[2], out[1]));
        double frac = Math.cos(Math.toRadians(entry - APERTURE_NORMAL_DEG));
        // pinned at either end of the bracket means no speed reaches the aperture centre
        boolean converged = d > 0 && v > V_MIN + 1e-3 && v < V_MAX - 1e-3
                && Math.abs(out[0] - dz) < 0.01;

        double rpm = outtake.rpmForSpeed(v);

        return new Shot(converged && frac > 0 && !Double.isNaN(rpm) && rpm > 0,
                rpm, v, out[3], entry, frac, rangeIn, headingRad);
    }

    /** RK4 out to horizontal distance dTarget; out = {z, vx, vz, t} on arrival **/
    private static void fly(double v0, double angle, double k, double dTarget, double[] out) {
        double dt = 0.002;
        double x = 0, z = 0, t = 0;
        double vx = v0 * Math.cos(angle), vz = v0 * Math.sin(angle);
        double px = 0, pz = 0, pvx = vx, pvz = vz, pt = 0;

        for (int i = 0; i < 1500 && x < dTarget; i++) {
            px = x;
            pz = z;
            pvx = vx;
            pvz = vz;
            pt = t;

            double s1 = mag(vx, vz);
            double a1x = -k * s1 * vx, a1z = -G - k * s1 * vz;
            double v2x = vx + 0.5 * dt * a1x, v2z = vz + 0.5 * dt * a1z;
            double s2 = mag(v2x, v2z);
            double a2x = -k * s2 * v2x, a2z = -G - k * s2 * v2z;
            double v3x = vx + 0.5 * dt * a2x, v3z = vz + 0.5 * dt * a2z;
            double s3 = mag(v3x, v3z);
            double a3x = -k * s3 * v3x, a3z = -G - k * s3 * v3z;
            double v4x = vx + dt * a3x, v4z = vz + dt * a3z;
            double s4 = mag(v4x, v4z);
            double a4x = -k * s4 * v4x, a4z = -G - k * s4 * v4z;

            x += dt / 6.0 * (vx + 2 * v2x + 2 * v3x + v4x);
            z += dt / 6.0 * (vz + 2 * v2z + 2 * v3z + v4z);
            vx += dt / 6.0 * (a1x + 2 * a2x + 2 * a3x + a4x);
            vz += dt / 6.0 * (a1z + 2 * a2z + 2 * a3z + a4z);
            t += dt;
        }

        double span = x - px;
        double f = span > 1e-9 ? (dTarget - px) / span : 0;
        out[0] = pz + f * (z - pz);
        out[1] = pvx + f * (vx - pvx);
        out[2] = pvz + f * (vz - pvz);
        out[3] = pt + f * (t - pt);
    }

    private static double mag(double a, double b) {
        return Math.sqrt(a * a + b * b);
    }
}

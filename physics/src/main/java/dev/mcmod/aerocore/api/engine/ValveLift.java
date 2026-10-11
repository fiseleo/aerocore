package dev.mcmod.aerocore.api.engine;

/**
 * Valve lift against crank angle for one cam lobe: lift = L f(u), u = (theta - centre) / (duration / 2) in [-1, 1],
 * f(u) = (1 - |u|^p)^3. The cube makes lift, velocity and acceleration zero at both seats (no clearance ramps are
 * modelled); the fullness exponent p >= 2 squares the lift curve off (more area, harder acceleration). duration is
 * seat to seat in crank radians; centre is the crank angle of peak lift in the cycle (0 = firing TDC, 4 pi per cycle).
 * Valve accelerations follow from f'' and the crank speed; a profile like this is a design choice for the game, not
 * a measured cam.
 */
public record ValveLift(double centreRad, double durationRad, double maxLiftM, double fullness) {
    public ValveLift {
        if (!(durationRad > 0) || durationRad >= 4 * Math.PI) { throw new IllegalArgumentException("Duration must be in (0, 720) degrees"); }
        if (!(maxLiftM > 0)) { throw new IllegalArgumentException("Lift must be positive"); }
        if (!(fullness >= 2) || !Double.isFinite(fullness)) { throw new IllegalArgumentException("Fullness must be at least 2"); }
    }

    /** Normalised position u of a crank angle (any cycle) on this lobe, or NaN when the valve is shut. */
    private double u(double theta) {
        double d = theta - centreRad;
        d -= 4 * Math.PI * Math.floor((d + 2 * Math.PI) / (4 * Math.PI));   // wrap to [-2 pi, 2 pi)
        double half = durationRad / 2;
        return Math.abs(d) >= half ? Double.NaN : d / half;
    }

    /** Lift at crank angle theta, m. */
    public double lift(double theta) {
        double u = u(theta);
        if (Double.isNaN(u)) { return 0; }
        double g = 1 - Math.pow(Math.abs(u), fullness);
        return maxLiftM * g * g * g;
    }

    /** Second derivative of f with respect to u (dimensionless). */
    static double shapeCurvature(double u, double p) {
        double x = Math.abs(u);
        if (x >= 1) { return 0; }
        double xp = Math.pow(x, p), g = 1 - xp;
        if (x == 0) { return p == 2 ? -6 : 0; }
        // f = g^3, g' = -p x^(p-1) (sign of u drops out of f''), g'' = -p (p-1) x^(p-2)
        double g1 = -p * xp / x, g2 = -p * (p - 1) * xp / (x * x);
        return 6 * g * g1 * g1 + 3 * g * g * g2;
    }

    /** Valve acceleration at crank angle theta for a crank speed omega (rad/s), m/s^2 (negative: decelerating towards the nose). */
    public double acceleration(double theta, double omega) {
        double u = u(theta);
        if (Double.isNaN(u)) { return 0; }
        double scale = 2 / durationRad;
        return maxLiftM * shapeCurvature(u, fullness) * scale * scale * omega * omega;
    }

    /**
     * Crank speed (rad/s) at which a spring of preload F0 (N, at the seat) and rate k (N/m) can no longer hold a
     * valve train of effective moving mass m (kg) on the lobe where it decelerates: min over the lobe of
     * sqrt((F0 + k L f(u)) / (m L |f''(u)| (2/duration)^2)). Above it the follower leaves the cam (valve float).
     */
    public double floatSpeed(double preloadN, double rateNPerM, double movingMassKg) {
        double scale = 2 / durationRad, best = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= 400; i++) {
            double u = i / 400.0, curvature = shapeCurvature(u, fullness);
            if (curvature >= 0) { continue; }
            double g = 1 - Math.pow(u, fullness), force = preloadN + rateNPerM * maxLiftM * g * g * g;
            best = Math.min(best, Math.sqrt(force / (movingMassKg * maxLiftM * -curvature * scale * scale)));
        }
        return best;
    }

    /** Crank duration (radians) during which lift exceeds h (for example 1 mm, the usual published figure). */
    public double durationAbove(double liftM) {
        if (liftM >= maxLiftM) { return 0; }
        double g = Math.cbrt(liftM / maxLiftM), u = Math.pow(1 - g, 1 / fullness);
        return durationRad * u;
    }
}

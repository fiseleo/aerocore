package dev.mcmod.aerocore.api;

/**
 * The air an aircraft flies through near the ground, beyond the mean wind (SurfaceWind):
 * <ul>
 * <li>Turbulence: the Dryden model of MIL-F-8785C (1980) section 3.7, low altitude (below 1000 ft): for altitude h
 * (ft) and the 20 ft wind W20, L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2, sigma_w = 0.1 W20,
 * sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4; the spectra (spatial frequency Omega, rad/m)
 * Phi_u = sigma_u^2 (2 L_u / pi) / (1 + (L_u Omega)^2) and Phi_w = sigma_w^2 (L_w / pi) (1 + 3 (L_w Omega)^2) /
 * (1 + (L_w Omega)^2)^2 (Phi_v like Phi_w). Gusts in time come from shaping filters driven by white noise at the
 * airspeed (Dryden, the usual first- and second-order forms), one per aircraft.</li>
 * <li>Ground effect: Wieselsberger's (1922) interference factor sigma = (1 - 1.32 h/b) / (1.05 + 7.4 h/b) for a wing
 * of span b at height h (as quoted by Hoerner and Borst, Fluid-Dynamic Lift, 1975, ch. 20): induced drag C_Di (1 -
 * sigma), the same as a wing of aspect ratio AR / (1 - sigma) in free air; sigma reaches 0 at h/b = 0.76.</li>
 * <li>Thermals: Allen's (2006, NASA/TM-2006-213958) scaling with height in a convective mixed layer of depth z_i -
 * mean updraft w_c = w* (z/z_i)^(1/3) (1 - 1.1 z/z_i), radius r2 = max(10, 0.102 (z/z_i)^(1/3) (1 - 0.25 z/z_i) z_i),
 * spacing sqrt(z_i r2 / 0.6) (his N = 0.6 A / (z_i r2)) - with Gedeon's (1988) radial profile
 * w = w_0 (1 - (r/R)^2) exp(-(r/R)^2), an updraft core in a ring of sink that carries no net flux; w_0 = 3 w_c (Allen's
 * peak over mean) and R = r2 (the core's edge, where w = 0; the mean over the core is then 0.37 w_0 = 1.1 w_c), an
 * approximation of Allen's own profile.
 * The convective velocity w* = (g / T H / (rho c_p) z_i)^(1/3) (Deardorff 1970) from the surface's sensible heat flux
 * H.</li>
 * <li>Ridge lift: air pushed up a slope rises at w = U dh/ds at the surface (s along the wind), decaying with height
 * as exp(-z / L) over the hill's half-length L (linear potential flow over a hill of length scale L); downward on lee
 * slopes. Without stratification there are no lee waves or rotors.</li>
 * </ul>
 */
public final class FlightAtmosphere {
    private static final double FEET = 0.3048;

    private FlightAtmosphere() {}

    // ------------------------------------------------------------------ Dryden turbulence

    /** Dryden intensities (m/s) and scale lengths (m) at an altitude. */
    public record Dryden(double sigmaU, double sigmaV, double sigmaW, double lengthU, double lengthV, double lengthW) {
        /** Longitudinal spectrum at spatial frequency omega (rad/m), m^3/s^2. */
        public double spectrumU(double omega) {
            return sigmaU * sigmaU * 2 * lengthU / Math.PI / (1 + Math.pow(lengthU * omega, 2));
        }
        /** Vertical spectrum at spatial frequency omega (rad/m), m^3/s^2. */
        public double spectrumW(double omega) {
            double x = Math.pow(lengthW * omega, 2);
            return sigmaW * sigmaW * lengthW / Math.PI * (1 + 3 * x) / Math.pow(1 + x, 2);
        }
    }

    /**
     * MIL-F-8785C low-altitude Dryden turbulence at height h (m, held to 3-305 m: 10-1000 ft) for the wind at 20 ft
     * (6.1 m) above the ground, m/s.
     */
    public static Dryden lowAltitude(double heightM, double wind20Mps) {
        double h = Math.clamp(heightM, 10 * FEET, 1000 * FEET) / FEET, base = 0.177 + 0.000823 * h;
        double sigmaW = 0.1 * Math.max(wind20Mps, 0), sigmaU = sigmaW / Math.pow(base, 0.4);
        double lengthW = h * FEET, lengthU = h / Math.pow(base, 1.2) * FEET;
        return new Dryden(sigmaU, sigmaU, sigmaW, lengthU, lengthU, lengthW);
    }

    /**
     * Gusts along an aircraft's path by Dryden shaping filters (exact discretisation of the first-order u filter; the
     * second-order v and w filters as two first-order stages with the Dryden zero), fed with Gaussian white noise from
     * the given random source; components in the wind axes (u along the airspeed, v right, w down).
     */
    public static final class Gusts {
        private final java.util.Random random;
        private double u, v1, v2, w1, w2;

        public Gusts(long seed) { random = new java.util.Random(seed); }

        /** Advances dt seconds at airspeed V (m/s) through turbulence d; returns {u, v, w} gusts, m/s. */
        public double[] step(Dryden d, double airspeed, double dt) {
            double speed = Math.max(airspeed, 1);
            u = firstOrder(u, d.lengthU() / speed, d.sigmaU(), dt);
            double[] v = secondOrder(v1, v2, d.lengthV() / speed, d.sigmaV(), dt);
            v1 = v[0]; v2 = v[1];
            double[] w = secondOrder(w1, w2, d.lengthW() / speed, d.sigmaW(), dt);
            w1 = w[0]; w2 = w[1];
            return new double[]{u, v[2], w[2]};
        }

        /** Ornstein-Uhlenbeck step with time constant tau and stationary deviation sigma (exact). */
        private double firstOrder(double x, double tau, double sigma, double dt) {
            double a = Math.exp(-dt / tau);
            return a * x + sigma * Math.sqrt(1 - a * a) * random.nextGaussian();
        }

        /**
         * Dryden's lateral/vertical filter sigma sqrt(L/(pi V)) (1 + sqrt(3) L/V s) / (1 + L/V s)^2 as an OU process y
         * feeding a second OU stage z of the same time constant, output (1 - sqrt(3)) z + sqrt(3) y scaled so its
         * stationary deviation is sigma (the two-pole form with the Dryden zero).
         */
        private double[] secondOrder(double y, double z, double tau, double sigma, double dt) {
            double a = Math.exp(-dt / tau), nextY = a * y + Math.sqrt(1 - a * a) * random.nextGaussian();
            double nextZ = a * z + (1 - a) * nextY;
            // Stationary moments of the unit OU y and its smoothed z: var z = (1 + a^2) / (1 + a)^2, cov(y, z) = 1 / (1 + a).
            double vz = (1 + a * a) / ((1 + a) * (1 + a)), cov = 1 / (1 + a);
            double k = Math.sqrt(3), out = k * nextY + (1 - k) * nextZ;
            double variance = k * k + (1 - k) * (1 - k) * Math.max(vz, 1e-12) + 2 * k * (1 - k) * cov;
            return new double[]{nextY, nextZ, sigma * out / Math.sqrt(Math.max(variance, 1e-12))};
        }
    }

    // ------------------------------------------------------------------ ground effect

    /** Wieselsberger's interference factor sigma at height h over span b (0 at h/b = 0.76 and above). */
    public static double groundInterference(double heightM, double spanM) {
        double r = Math.max(heightM, 0) / spanM;
        return r >= 1 / 1.32 ? 0 : (1 - 1.32 * r) / (1.05 + 7.4 * r);
    }

    /** Induced drag in ground effect over that in free air: 1 - sigma. */
    public static double inducedDragFactor(double heightM, double spanM) { return 1 - groundInterference(heightM, spanM); }

    /** Aspect ratio of the free-air wing with the same induced drag as one of aspect ratio AR in ground effect. */
    public static double effectiveAspectRatio(double aspectRatio, double heightM, double spanM) {
        return aspectRatio / inducedDragFactor(heightM, spanM);
    }

    // ------------------------------------------------------------------ thermals

    /** Convective velocity scale w* (Deardorff) for a sensible heat flux H (W/m^2), mixed layer depth z_i (m), air at T (K). */
    public static double convectiveVelocity(double heatFluxWm2, double mixedLayerM, double airKelvin) {
        if (!(heatFluxWm2 > 0)) { return 0; }
        double kinematic = heatFluxWm2 / (1.2 * 1005);
        return Math.cbrt(Atmosphere.GRAVITY_MPS2 / airKelvin * kinematic * mixedLayerM);
    }

    /** Allen's mean updraft in a thermal at height z (m) in a mixed layer of depth z_i, for w*; 0 outside 0 < z < z_i / 1.1. */
    public static double thermalMeanUpdraft(double wStar, double heightM, double mixedLayerM) {
        double s = heightM / mixedLayerM;
        return s <= 0 || s >= 1 / 1.1 ? 0 : wStar * Math.cbrt(s) * (1 - 1.1 * s);
    }

    /** Allen's thermal radius r2 at height z (m) in a mixed layer of depth z_i, m (at least 10 m). */
    public static double thermalRadius(double heightM, double mixedLayerM) {
        double s = Math.clamp(heightM / mixedLayerM, 0, 1);
        return Math.max(10, 0.102 * Math.cbrt(s) * (1 - 0.25 * s) * mixedLayerM);
    }

    /** Allen's spacing of thermals at height z, m: sqrt(z_i r2 / 0.6). */
    public static double thermalSpacing(double heightM, double mixedLayerM) {
        return Math.sqrt(mixedLayerM * thermalRadius(heightM, mixedLayerM) / 0.6);
    }

    /**
     * Vertical air speed at distance r (m) from a thermal's axis at height z (m): Gedeon's profile with core speed
     * 3 w_c and its zero at Allen's r2, m/s (negative in the sink ring around the core).
     */
    public static double thermalUpdraft(double wStar, double r, double heightM, double mixedLayerM) {
        double core = 3 * thermalMeanUpdraft(wStar, heightM, mixedLayerM);
        if (core == 0) { return 0; }
        double x = r * r / Math.pow(thermalRadius(heightM, mixedLayerM), 2);
        return core * (1 - x) * Math.exp(-x);
    }

    // ------------------------------------------------------------------ ridge lift

    /** Vertical wind over a slope: U dh/ds at the surface, decaying as exp(-z / L) with height z (m) over length L (m). */
    public static double ridgeLift(double windMps, double slopeAlongWind, double heightM, double lengthM) {
        return windMps * slopeAlongWind * Math.exp(-Math.max(heightM, 0) / Math.max(lengthM, 1));
    }
}

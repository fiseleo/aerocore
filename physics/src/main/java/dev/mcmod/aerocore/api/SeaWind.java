package dev.mcmod.aerocore.api;

/**
 * Wind over the sea and what it does to the water, for a neutral atmosphere and a horizontally uniform wind given by
 * its speed at 10 m, U10.
 * <ul>
 * <li>Profile: the log law U(z) = (u* / kappa) ln(z / z0) with kappa = 0.4 and the COARE 3.0 roughness
 * z0 = alpha u*^2 / g + 0.11 nu / u* (Fairall et al. 2003, J. Climate 16:571), Charnock's alpha = 0.011 up to 10 m/s
 * rising linearly to 0.018 at 18 m/s. Below a few metres over waves higher than about 0.5 m the air is inside the
 * wave boundary layer and the log law is approximate; heights below MIN_HEIGHT are taken at MIN_HEIGHT.</li>
 * <li>Waves: fetch-limited growth of JONSWAP (Hasselmann et al. 1973, eqs. 2.4.3 and 2.4.7 with x~ = g x / U10^2):
 * Hm0 = 1.6e-3 (U10^2 / g) x~^(1/2), Tp = (U10 / g) x~^0.33 / 3.5, up to the fully developed Pierson-Moskowitz sea
 * (Hs = 0.2092 U19.5^2 / g, omega_p = 0.877 g / U19.5, U19.5 = 1.05 U10 as in Whitecaps), which JONSWAP's growth
 * reaches at x~ = 2.1e4. In finite depth Tp <= 9.78 sqrt(d / g) (CEM II-2-39) and Hs <= CoastalWaves'
 * depth-saturated 0.59 d. Duration limits and swell are not modelled.</li>
 * <li>Drift: the wind-driven surface current is 0.53 u* (Wu 1975, J. Fluid Mech. 68:49; as applied by Laxague et al.
 * 2018, GRL 45:245), 1.9 % of U10 at 10 m/s, decaying below the surface with the e-folding depth DRIFT_DEPTH; the
 * strong shear is confined to the top decimetre (Laxague 2018) but the exponential shape is an assumption. Measured
 * total drift of the top centimetres, 3-6 % of U10, adds the waves' Stokes drift, which resolved waves give floating
 * bodies by themselves, so it is not added here.</li>
 * </ul>
 */
public final class SeaWind {
    public static final double KARMAN = 0.4;
    /** Kinematic viscosity of air near 20 C, m^2/s. */
    public static final double AIR_VISCOSITY = 1.5e-5;
    /** Wind-driven surface current per friction velocity of the air (Wu 1975). */
    public static final double DRIFT_PER_FRICTION_VELOCITY = 0.53;
    /** e-folding depth of the wind-driven current, m (the top decimetre; shape assumed). */
    public static final double DRIFT_DEPTH = 0.1;
    /** Lowest height the log profile is evaluated at, m. */
    public static final double MIN_HEIGHT = 0.1;
    /** Strongest wind accepted, m/s (the Charnock fit of COARE 3.0 is held constant beyond 18 m/s). */
    public static final double MAX_WIND = 30;
    private static final double G = Atmosphere.GRAVITY_MPS2;
    /** Pierson-Moskowitz constants and the U19.5 / U10 ratio used with them. */
    private static final double PM_HEIGHT = 0.2092, PM_PEAK = 0.877, U195_PER_U10 = 1.05;

    private SeaWind() {}

    private static void check(double u10) {
        if (!(u10 >= 0 && u10 <= MAX_WIND)) { throw new IllegalArgumentException("Wind speed must be in [0, 30] m/s"); }
    }

    /** Charnock parameter of COARE 3.0 at a 10 m wind. */
    public static double charnock(double u10) { return 0.011 + 0.007 * Math.clamp((u10 - 10) / 8, 0, 1); }

    /** Friction velocity u* of a neutral 10 m wind (fixed point of U10 = (u* / kappa) ln(10 / z0)), m/s. */
    public static double frictionVelocity(double u10) {
        check(u10);
        if (u10 == 0) { return 0; }
        double ustar = 0.035 * u10;
        for (int i = 0; i < 60; i++) {
            double next = KARMAN * u10 / Math.log(10 / roughness(ustar, u10));
            if (Math.abs(next - ustar) < 1e-12) { return next; }
            ustar = next;
        }
        return ustar;
    }

    /** Roughness length z0 = alpha u*^2 / g + 0.11 nu / u*, m. */
    public static double roughness(double ustar, double u10) { return charnock(u10) * ustar * ustar / G + 0.11 * AIR_VISCOSITY / ustar; }

    /** Neutral 10 m drag coefficient (u* / U10)^2. */
    public static double dragCoefficient(double u10) {
        double ustar = frictionVelocity(u10);
        return u10 == 0 ? 0 : ustar * ustar / (u10 * u10);
    }

    /** Mean wind speed at a height above the mean surface, m/s (log law; MIN_HEIGHT below that). */
    public static double speedAt(double u10, double heightM) {
        if (u10 == 0) { return 0; }
        double ustar = frictionVelocity(u10), z = Math.max(heightM, MIN_HEIGHT);
        return Math.max(0, ustar / KARMAN * Math.log(z / roughness(ustar, u10)));
    }

    /** Wind-driven current at a depth below the surface (0 at the surface), m/s, along the wind. */
    public static double drift(double u10, double depthM) {
        return DRIFT_PER_FRICTION_VELOCITY * frictionVelocity(u10) * Math.exp(-Math.max(depthM, 0) / DRIFT_DEPTH);
    }

    /**
     * Significant height and peak period {Hs m, Tp s} of the sea this wind raises over a fetch (m; infinite for a fully
     * developed sea) in water of a depth (m; infinite for deep water).
     */
    public static double[] sea(double u10, double fetchM, double depthM) {
        check(u10);
        if (!(fetchM > 0)) { throw new IllegalArgumentException("Fetch must be positive"); }
        if (u10 == 0) { return new double[]{0, 0}; }
        double u195 = U195_PER_U10 * u10;
        double hs = PM_HEIGHT * u195 * u195 / G, tp = 2 * Math.PI * u195 / (PM_PEAK * G);
        if (Double.isFinite(fetchM)) {
            double x = G * fetchM / (u10 * u10);
            hs = Math.min(hs, 1.6e-3 * u10 * u10 / G * Math.sqrt(x));
            tp = Math.min(tp, u10 / G * Math.pow(x, 0.33) / 3.5);
        }
        if (Double.isFinite(depthM)) {
            tp = Math.min(tp, 9.78 * Math.sqrt(depthM / G));
            hs = Math.min(hs, CoastalWaves.SATURATED_HS_PER_DEPTH * depthM);
        }
        return new double[]{hs, tp};
    }
}

package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Aerodynamic heating of a skin in steady flight (0.28), for warnings: the temperature a thin skin settles at where the
 * heat the boundary layer brings equals what it radiates away (radiative equilibrium; no conduction inward, no heat
 * capacity: the upper bound a skin reaches after some tens of seconds). Air, SI, kelvin.
 * <ul>
 * <li>Recovery temperature: T (1 + r (gamma - 1)/2 M^2), r = Pr^(1/3) = 0.89 (turbulent boundary layer).</li>
 * <li>Along a surface at a distance x from its leading edge: a turbulent flat plate by Eckert's reference temperature
 *   (T* = 0.28 T + 0.5 T_w + 0.22 T_r; properties at T*), Colburn's St = 0.0296 Re_x*^-0.2 Pr^-2/3 (Anderson,
 *   Hypersonic and High-Temperature Gas Dynamics, sec. 6.7 and 16.3).</li>
 * <li>At a blunt nose of radius R: Sutton and Graves' stagnation-point heating, q = 1.7415e-4 sqrt(rho / R) V^3 W/m^2
 *   (NASA TR R-376, 1971; cold wall), times (1 - h_w / h_0) for the wall's own heat.</li>
 * </ul>
 * Not modelled: dissociation above about 2500 K, laminar runs (laminar heating is lower: this errs hot), shock
 * interactions (leading edges in a body's shock), the skin's heat capacity (warming takes time).
 */
public final class AeroHeating {
    private AeroHeating() {}

    static final double GAMMA = 1.4, R = 287.05287, CP = GAMMA * R / (GAMMA - 1), PRANDTL = 0.71, RECOVERY = 0.89, SIGMA = 5.670374e-8;
    /** Sutton and Graves' constant for air, kg^0.5 m^-1 (q in W/m^2 with rho in kg/m^3, R in m, V in m/s). */
    public static final double SUTTON_GRAVES = 1.7415e-4;

    /** The recovery temperature of a turbulent boundary layer, K. */
    public static double recoveryTemperature(double airK, double mach) { return airK * (1 + RECOVERY * 0.5 * (GAMMA - 1) * mach * mach); }

    /** Sutherland's law for air, Pa s. */
    static double viscosity(double t) { return 1.716e-5 * Math.pow(t / 273.15, 1.5) * (273.15 + 110.4) / (t + 110.4); }

    /** The turbulent flat plate's heat transfer coefficient (W/m^2 K) at x (m) from the leading edge, for a wall at wallK. */
    public static double flatPlateCoefficient(double densityKgM3, double airK, double speedMps, double x, double wallK) {
        double mach = speedMps / Math.sqrt(GAMMA * R * airK), tr = recoveryTemperature(airK, mach);
        double ts = 0.28 * airK + 0.5 * wallK + 0.22 * tr;
        double rhoS = densityKgM3 * airK / ts, mu = viscosity(ts);
        double re = rhoS * speedMps * Math.max(x, 1e-3) / mu;
        double st = 0.0296 * Math.pow(re, -0.2) * Math.pow(PRANDTL, -2.0 / 3);
        return st * rhoS * speedMps * CP;
    }

    /** Sutton and Graves' stagnation-point heat flux (W/m^2) on a nose of radius R (m), for a wall at wallK. */
    public static double stagnationHeatFlux(double densityKgM3, double airK, double speedMps, double noseRadiusM, double wallK) {
        double h0 = CP * airK + 0.5 * speedMps * speedMps;
        return SUTTON_GRAVES * Math.sqrt(densityKgM3 / Math.max(noseRadiusM, 1e-3)) * Math.pow(speedMps, 3) * Math.max(0, 1 - CP * wallK / h0);
    }

    /**
     * The radiative-equilibrium temperature (K) of a skin x (m) behind a leading edge, emissivity e, in air at a density
     * and temperature, flying at a speed: h (T_r - T_w) = e sigma (T_w^4 - T^4).
     */
    public static double surfaceTemperature(double densityKgM3, double airK, double speedMps, double x, double emissivity) {
        double tr = recoveryTemperature(airK, speedMps / Math.sqrt(GAMMA * R * airK));
        return equilibrium(airK, tr, emissivity, tw -> flatPlateCoefficient(densityKgM3, airK, speedMps, x, tw) * (tr - tw));
    }

    /** The radiative-equilibrium temperature (K) of a blunt nose of radius R (m), emissivity e. */
    public static double noseTemperature(double densityKgM3, double airK, double speedMps, double noseRadiusM, double emissivity) {
        double t0 = airK + 0.5 * speedMps * speedMps / CP;
        return equilibrium(airK, t0, emissivity, tw -> stagnationHeatFlux(densityKgM3, airK, speedMps, noseRadiusM, tw));
    }

    /** The wall temperature between the air's and hi where the heat in equals e sigma (T_w^4 - T^4) (bisection). */
    private static double equilibrium(double airK, double hi, double emissivity, java.util.function.DoubleUnaryOperator heatIn) {
        double e = Math.clamp(emissivity, 0.05, 1), lo = airK, top = Math.max(hi, airK);
        for (int k = 0; k < 60; k++) {
            double tw = 0.5 * (lo + top);
            double net = heatIn.applyAsDouble(tw) - e * SIGMA * (Math.pow(tw, 4) - Math.pow(airK, 4));
            if (net > 0) { lo = tw; } else { top = tw; }
        }
        return 0.5 * (lo + top);
    }
}

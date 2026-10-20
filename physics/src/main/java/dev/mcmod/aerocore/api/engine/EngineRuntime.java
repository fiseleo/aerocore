package dev.mcmod.aerocore.api.engine;

import java.util.function.DoubleUnaryOperator;

/** Run-time relations of a four-stroke piston engine around its map: torque, intake manifold filling, exhaust and filter losses. */
public final class EngineRuntime {
    private EngineRuntime() {}

    /** Crank torque (N m) of a four-stroke engine from a mean effective pressure (Pa) over its displacement (m^3): p Vd / (4 pi). */
    public static double torqueFromMep(double mepPa, double displacementM3) { return mepPa * displacementM3 / (4 * Math.PI); }

    /**
     * Exhaust port pressure (Pa) of a production-like exhaust: Patton, Nitschke and Heywood's system loses
     * 0.178 (p_i/p_a S_p)^2 kPa of pumping mean effective pressure (SAE 890836, as given by Sandoval 2002, eq. 13), taken
     * as that much back pressure over ambient, times a factor (1 = a 1980s production system, about 0.5 free-flowing).
     */
    public static double exhaustPressure(double ambientPa, double manifoldPa, double meanPistonSpeed, double factor) {
        double x = manifoldPa / ambientPa * meanPistonSpeed;
        return ambientPa + factor * 178 * x * x;
    }

    /** Pressure drop (Pa) across an air filter of loss coefficient K at a mass flow (kg/s) through an area (m^2): K rho u^2 / 2. */
    public static double filterDrop(double lossCoefficient, double massFlow, double density, double area) {
        return lossCoefficient * massFlow * massFlow / (2 * density * area * area);
    }

    /**
     * Manifold pressure ratio (over ambient) at the end of a step dt: the plenum of capacity V/(R T) (kg/Pa) fills
     * through the throttle (an orifice of effective area cdA from the upstream pressure at airK) and empties into the
     * engine, draw(ratio) kg/s; solved implicitly by bisection between 0.02 and upstream/ambient (0.26: above 1 behind a
     * turbocharger; before, at most 1.2).
     */
    public static double manifoldRatio(double startRatio, double dt, double capacity, double ambientPa, double upstreamPa, double throttleCdA,
                                       double airK, double gasConstant, double gamma, DoubleUnaryOperator draw) {
        double lo = 0.02, hi = Math.max(0.03, upstreamPa / ambientPa);
        for (int k = 0; k < 40; k++) {
            double mid = 0.5 * (lo + hi), p = mid * ambientPa;
            double in = ValveFlow.massFlow(throttleCdA, upstreamPa, airK, p, gasConstant, gamma);
            double balance = capacity * (mid - startRatio) * ambientPa / dt - (in - draw.applyAsDouble(mid));
            if (balance > 0) { hi = mid; } else { lo = mid; }
        }
        return 0.5 * (lo + hi);
    }
}

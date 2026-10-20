package dev.mcmod.aerocore.api.power;

/**
 * Liquid flow in a pipe (0.26): Darcy–Weisbach, the friction factor 64 / Re in laminar flow (Re below 2300),
 * Swamee and Jain's explicit fit of Colebrook's equation in turbulent flow (above 4000; within 1 % for relative
 * roughness 1e-6 to 1e-2 and Re 5000 to 1e8), linear between; fittings and valves as a sum of loss coefficients K on
 * the velocity head (Crane Technical Paper 410; White, Fluid Mechanics ch. 6).
 */
public final class PipeFlow {
    private PipeFlow() {}

    /** Commercial steel pipe's roughness, m (Moody). */
    public static final double STEEL_ROUGHNESS_M = 4.5e-5;
    public static final double LAMINAR_RE = 2300, TURBULENT_RE = 4000;

    public static double areaM2(double diameterM) { return Math.PI / 4 * diameterM * diameterM; }

    public static double reynolds(double velocityMps, double diameterM, double density, double viscosityPaS) { return density * Math.abs(velocityMps) * diameterM / viscosityPaS; }

    /** Darcy friction factor at a Reynolds number and relative roughness (roughness over diameter). */
    public static double frictionFactor(double re, double relativeRoughness) {
        if (re <= LAMINAR_RE) { return 64 / Math.max(re, 1e-9); }
        double turbulent = swameeJain(Math.max(re, TURBULENT_RE), relativeRoughness);
        if (re >= TURBULENT_RE) { return turbulent; }
        double t = (re - LAMINAR_RE) / (TURBULENT_RE - LAMINAR_RE);
        return (1 - t) * 64 / LAMINAR_RE + t * turbulent;
    }

    private static double swameeJain(double re, double relativeRoughness) {
        double l = Math.log10(relativeRoughness / 3.7 + 5.74 / Math.pow(re, 0.9));
        return 0.25 / (l * l);
    }

    /** Pressure lost by a flow (m^3/s) through a pipe and fittings (loss coefficient sum), Pa. */
    public static double pressureDropPa(double flowM3PerS, double diameterM, double lengthM, double roughnessM, double density, double viscosityPaS, double fittingsK) {
        double v = Math.abs(flowM3PerS) / areaM2(diameterM);
        double f = frictionFactor(reynolds(v, diameterM, density, viscosityPaS), roughnessM / diameterM);
        return (f * lengthM / diameterM + fittingsK) * density * v * v / 2;
    }

    /** The flow a pressure difference drives through a pipe and fittings, m^3/s (the drop is monotone in the flow; bisection). */
    public static double flowM3PerS(double pressurePa, double diameterM, double lengthM, double roughnessM, double density, double viscosityPaS, double fittingsK) {
        if (!(pressurePa > 0)) { return 0; }
        double hi = 1e-6;
        while (pressureDropPa(hi, diameterM, lengthM, roughnessM, density, viscosityPaS, fittingsK) < pressurePa && hi < 1e3) { hi *= 2; }
        double lo = 0;
        for (int i = 0; i < 80; i++) {
            double mid = 0.5 * (lo + hi);
            if (pressureDropPa(mid, diameterM, lengthM, roughnessM, density, viscosityPaS, fittingsK) < pressurePa) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }
}

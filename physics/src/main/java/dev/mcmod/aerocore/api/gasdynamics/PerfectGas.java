package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Isentropic flow of a calorically perfect gas (constant gamma) in closed form, as tabulated in NACA Report 1135
 * (Ames Research Staff, 1953) and derived in Anderson, Modern Compressible Flow (3rd ed., ch. 3-5): total-to-static
 * ratios, the area-Mach relation, the mass-flow parameter, the Prandtl-Meyer function and the starting limits of a
 * supersonic inlet (Kantrowitz and Donaldson, NACA WR L-713, 1945). Angles in radians.
 */
public final class PerfectGas {
    private PerfectGas() {}

    private static void check(double mach, double gamma) {
        if (!(mach >= 0) || !Double.isFinite(mach) || !(gamma > 1)) { throw new IllegalArgumentException("Mach must be finite and nonnegative, gamma > 1"); }
    }

    /** T0 / T. */
    public static double totalToStaticTemperature(double mach, double gamma) {
        check(mach, gamma);
        return 1 + 0.5 * (gamma - 1) * mach * mach;
    }

    /** p0 / p. */
    public static double totalToStaticPressure(double mach, double gamma) {
        return Math.pow(totalToStaticTemperature(mach, gamma), gamma / (gamma - 1));
    }

    /** rho0 / rho. */
    public static double totalToStaticDensity(double mach, double gamma) {
        return Math.pow(totalToStaticTemperature(mach, gamma), 1 / (gamma - 1));
    }

    /** A / A*: the area that passes the flow at Mach M over the sonic (throat) area; infinite at rest. */
    public static double areaRatio(double mach, double gamma) {
        check(mach, gamma);
        if (mach == 0) { return Double.POSITIVE_INFINITY; }
        return Math.pow(2 / (gamma + 1) * totalToStaticTemperature(mach, gamma), (gamma + 1) / (2 * (gamma - 1))) / mach;
    }

    /** The mass-flow parameter mdot sqrt(R T0) / (A p0) at Mach M (sqrt(gamma) M (T0/T)^-(gamma+1)/(2(gamma-1))). */
    public static double massFlowParameter(double mach, double gamma) {
        return Math.sqrt(gamma) * mach * Math.pow(totalToStaticTemperature(mach, gamma), -(gamma + 1) / (2 * (gamma - 1)));
    }

    /** The Mach number at an area ratio A / A* (>= 1) on the subsonic or supersonic branch. */
    public static double machAtAreaRatio(double ratio, double gamma, boolean supersonic) {
        if (!(ratio >= 1)) { throw new IllegalArgumentException("A / A* must be at least 1"); }
        if (ratio == 1) { return 1; }
        double lo = supersonic ? 1 : 1e-12, hi = supersonic ? 1 : 1;
        if (supersonic) { while (areaRatio(hi, gamma) < ratio) { hi *= 2; } }
        for (int i = 0; i < 200; i++) {
            double m = 0.5 * (lo + hi);
            // the ratio falls with M below 1 and rises above
            if ((areaRatio(m, gamma) > ratio) == supersonic) { hi = m; } else { lo = m; }
            if (hi - lo < 1e-14 * hi) { break; }
        }
        return 0.5 * (lo + hi);
    }

    /** The Mach number at a total-to-static pressure ratio p0 / p (>= 1). */
    public static double machAtPressureRatio(double totalToStatic, double gamma) {
        if (!(totalToStatic >= 1)) { throw new IllegalArgumentException("p0 / p must be at least 1"); }
        return Math.sqrt(2 / (gamma - 1) * (Math.pow(totalToStatic, (gamma - 1) / gamma) - 1));
    }

    /** Mach angle asin(1 / M), M >= 1. */
    public static double machAngle(double mach) {
        if (!(mach >= 1)) { throw new IllegalArgumentException("The Mach angle needs M >= 1"); }
        return Math.asin(1 / mach);
    }

    /** The Prandtl-Meyer function nu(M), M >= 1. */
    public static double prandtlMeyer(double mach, double gamma) {
        if (!(mach >= 1)) { throw new IllegalArgumentException("Prandtl-Meyer needs M >= 1"); }
        double k = (gamma + 1) / (gamma - 1), b = Math.sqrt(mach * mach - 1);
        return Math.sqrt(k) * Math.atan(b / Math.sqrt(k)) - Math.atan(b);
    }

    /** The largest turn of a sonic stream expanding to vacuum, nu(infinity) = (sqrt((g+1)/(g-1)) - 1) pi / 2. */
    public static double maxPrandtlMeyer(double gamma) { return (Math.sqrt((gamma + 1) / (gamma - 1)) - 1) * Math.PI / 2; }

    /** The Mach number whose Prandtl-Meyer function is nu. */
    public static double machAtPrandtlMeyer(double nu, double gamma) {
        if (!(nu >= 0) || !(nu < maxPrandtlMeyer(gamma))) { throw new IllegalArgumentException("nu out of range"); }
        double lo = 1, hi = 2;
        while (prandtlMeyer(hi, gamma) < nu) { hi *= 2; }
        for (int i = 0; i < 200 && hi - lo > 1e-14 * hi; i++) {
            double m = 0.5 * (lo + hi);
            if (prandtlMeyer(m, gamma) > nu) { hi = m; } else { lo = m; }
        }
        return 0.5 * (lo + hi);
    }

    /**
     * Isentropic contraction limit A_throat / A_capture of an inlet at Mach M: compressed without shocks it reaches
     * sonic at A* / A(M); it cannot be smaller.
     */
    public static double isentropicContraction(double mach, double gamma) {
        if (!(mach > 1)) { throw new IllegalArgumentException("Inlet starting needs a supersonic Mach number"); }
        return 1 / areaRatio(mach, gamma);
    }

    /**
     * Kantrowitz-Donaldson starting limit A_throat / A_capture of an internal-contraction inlet at Mach M: with a
     * normal shock standing at its entrance, the subsonic flow behind it must pass the throat, so the throat can be no
     * smaller than the sonic area of that flow; a smaller one keeps the shock out (unstarted).
     */
    public static double kantrowitzContraction(double mach, double gamma) {
        if (!(mach > 1)) { throw new IllegalArgumentException("Inlet starting needs a supersonic Mach number"); }
        NormalShock s = NormalShock.of(mach, gamma);
        return isentropicContraction(mach, gamma) / s.totalPressureRatio();
    }
}

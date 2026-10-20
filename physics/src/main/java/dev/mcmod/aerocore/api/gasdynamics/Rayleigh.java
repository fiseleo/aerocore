package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Rayleigh flow: frictionless flow in a constant-area duct heated or cooled, calorically perfect gas (NACA Report
 * 1135 eqs. 117-122; Shapiro, The Dynamics and Thermodynamics of Compressible Fluid Flow, ch. 8). Ratios are to the
 * sonic (thermally choked) state of the same flow. Heating drives both subsonic and supersonic flow toward Mach 1;
 * a stream cannot take more heat than brings T0 to T0*.
 */
public final class Rayleigh {
    private Rayleigh() {}

    /** p / p*. */
    public static double pressureRatio(double mach, double gamma) { return (1 + gamma) / (1 + gamma * mach * mach); }

    /** T / T*. */
    public static double temperatureRatio(double mach, double gamma) {
        double p = pressureRatio(mach, gamma);
        return mach * mach * p * p;
    }

    /** T0 / T0*. */
    public static double totalTemperatureRatio(double mach, double gamma) {
        double m2 = mach * mach, d = 1 + gamma * m2;
        return (gamma + 1) * m2 * (2 + (gamma - 1) * m2) / (d * d);
    }

    /** p0 / p0*. */
    public static double totalPressureRatio(double mach, double gamma) {
        return pressureRatio(mach, gamma) * Math.pow((2 + (gamma - 1) * mach * mach) / (gamma + 1), gamma / (gamma - 1));
    }

    /** V / V* (= rho* / rho). */
    public static double speedRatio(double mach, double gamma) { return (1 + gamma) * mach * mach / (1 + gamma * mach * mach); }

    /** The Mach number at T0 / T0* (<= 1) on the subsonic or supersonic branch. */
    public static double machAtTotalTemperatureRatio(double ratio, double gamma, boolean supersonic) {
        if (!(ratio > 0) || !(ratio <= 1 + 1e-12)) { throw new IllegalArgumentException("T0 / T0* must be in (0, 1]"); }
        if (ratio >= 1) { return 1; }
        double lo = supersonic ? 1 : 0, hi = supersonic ? 2 : 1;
        // supersonic: the ratio falls from 1 at Mach 1 toward (g^2 - 1) / g^2
        if (supersonic && ratio <= (gamma * gamma - 1) / (gamma * gamma)) { throw new IllegalArgumentException("No supersonic Rayleigh flow has T0 / T0* that low"); }
        if (supersonic) { while (totalTemperatureRatio(hi, gamma) > ratio) { hi *= 2; } }
        for (int i = 0; i < 200 && hi - lo > 1e-15 * Math.max(1, hi); i++) {
            double m = 0.5 * (lo + hi);
            if ((totalTemperatureRatio(m, gamma) < ratio) != supersonic) { lo = m; } else { hi = m; }
        }
        return 0.5 * (lo + hi);
    }
}

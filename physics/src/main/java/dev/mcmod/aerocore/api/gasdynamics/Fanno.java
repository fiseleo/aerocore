package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Fanno flow: adiabatic flow with wall friction in a constant-area duct, calorically perfect gas (NACA Report 1135
 * eqs. 123-128; Shapiro, ch. 6). Ratios are to the sonic (friction-choked) state; 4 f L* / D is the duct length that
 * brings the flow to Mach 1 (f the Fanning friction factor, D the hydraulic diameter).
 */
public final class Fanno {
    private Fanno() {}

    /** T / T*. */
    public static double temperatureRatio(double mach, double gamma) { return (gamma + 1) / (2 + (gamma - 1) * mach * mach); }

    /** p / p*. */
    public static double pressureRatio(double mach, double gamma) { return Math.sqrt(temperatureRatio(mach, gamma)) / mach; }

    /** p0 / p0*. */
    public static double totalPressureRatio(double mach, double gamma) {
        return Math.pow((2 + (gamma - 1) * mach * mach) / (gamma + 1), (gamma + 1) / (2 * (gamma - 1))) / mach;
    }

    /** 4 f L* / D. */
    public static double frictionLength(double mach, double gamma) {
        double m2 = mach * mach;
        return (1 - m2) / (gamma * m2) + (gamma + 1) / (2 * gamma) * Math.log((gamma + 1) * m2 / (2 + (gamma - 1) * m2));
    }

    /** The Mach number with a given 4 f L* / D on the subsonic or supersonic branch. */
    public static double machAtFrictionLength(double length, double gamma, boolean supersonic) {
        if (!(length >= 0)) { throw new IllegalArgumentException("4 f L* / D must be nonnegative"); }
        if (length == 0) { return 1; }
        if (supersonic) {
            double limit = (gamma + 1) / (2 * gamma) * Math.log((gamma + 1) / (gamma - 1)) - 1 / gamma;
            if (length >= limit) { throw new IllegalArgumentException("No supersonic Fanno flow is that long"); }
        }
        double lo = supersonic ? 1 : 1e-9, hi = supersonic ? 2 : 1;
        if (supersonic) { while (frictionLength(hi, gamma) < length) { hi *= 2; } }
        for (int i = 0; i < 200 && hi - lo > 1e-15 * Math.max(1, hi); i++) {
            double m = 0.5 * (lo + hi);
            // the length falls toward Mach 1 on either branch
            if ((frictionLength(m, gamma) > length) != supersonic) { lo = m; } else { hi = m; }
        }
        return 0.5 * (lo + hi);
    }
}

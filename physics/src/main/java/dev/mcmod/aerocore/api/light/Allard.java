package dev.mcmod.aerocore.api.light;

/**
 * How far a light is seen through the air (0.38): Allard's law, E = I exp(-sigma d) / d^2, the illuminance a light of
 * intensity I gives at the eye d away through air of extinction coefficient sigma; the light is seen while E is
 * above the eye's threshold E_t, which rises with the brightness of the background.
 * <ul>
 *   <li>Extinction from the meteorological optical range (MOR, the visibility a weather report gives): Koschmieder
 *   with the 5 % contrast threshold MOR is defined with (WMO-No. 8), sigma = -ln 0.05 / MOR = 3.0 / MOR.</li>
 *   <li>Threshold: log10 E_t = -5.7 + 0.64 log10 B (lux; B the background luminance in cd/m^2), not below
 *   6.8e-6 lux, its value at B = 6.85 cd/m^2, the background ICAO counts as night (ICAO Doc 9328, Manual of Runway
 *   Visual Range Observing and Reporting Practices, as used for runway visual range).</li>
 * </ul>
 * For display (how strongly a renderer shows a light) these are a rule, not a model of the eye.
 */
public final class Allard {
    private Allard() {}

    /** The contrast threshold MOR is defined with. */
    public static final double MOR_CONTRAST = 0.05;
    /** Background luminance (cd/m^2): ICAO's night limit, and a bright day sky. */
    public static final double NIGHT_BACKGROUND = 6.85, DAY_BACKGROUND = 10_000;

    /** Extinction coefficient (1/m) for a visibility (MOR, m). */
    public static double extinctionPerM(double visibilityM) {
        if (!(visibilityM > 0)) { throw new IllegalArgumentException("Visibility must be positive"); }
        return -Math.log(MOR_CONTRAST) / visibilityM;
    }

    /** Illuminance at the eye (lux) from a light of that intensity (cd) at that distance (m) in that visibility (m). */
    public static double illuminanceLux(double intensityCd, double distanceM, double visibilityM) {
        double d = Math.max(distanceM, 1e-3);
        return intensityCd * Math.exp(-extinctionPerM(visibilityM) * d) / (d * d);
    }

    /** The eye's threshold illuminance (lux) against a background of that luminance (cd/m^2). */
    public static double thresholdLux(double backgroundCdM2) {
        double b = Math.max(backgroundCdM2, NIGHT_BACKGROUND);
        return Math.pow(10, -5.7 + 0.64 * Math.log10(b));
    }

    /** How far a light of that intensity is seen (m): where its illuminance falls to the threshold. 0 if not even at 1 m. */
    public static double rangeM(double intensityCd, double visibilityM, double thresholdLux) {
        if (!(intensityCd > 0) || illuminanceLux(intensityCd, 1, visibilityM) < thresholdLux) { return 0; }
        double lo = 1, hi = 2;
        while (illuminanceLux(intensityCd, hi, visibilityM) >= thresholdLux) { lo = hi; hi *= 2; }
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (illuminanceLux(intensityCd, mid, visibilityM) >= thresholdLux) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }
}

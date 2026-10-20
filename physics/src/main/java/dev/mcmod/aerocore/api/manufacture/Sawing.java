package dev.mcmod.aerocore.api.manufacture;

/**
 * Sawing (0.35): band, circular and power hack saws cutting stock off, after Groover (sec. 22.5) and Kalpakjian and
 * Schmid (ch. 24).
 *
 * <ul>
 * <li>A band or circular saw's teeth pass at v / p a second (blade speed over pitch), each taking a feed f_t, so the
 *     cut advances at v_f = f_t v / p (for a circular saw of z teeth at N rev/s, f_t z N, the same thing).</li>
 * <li>Its power is the specific cutting energy times the kerf, the length of cut across the work and v_f; a saw
 *     short of power feeds slower: v_f at most eta P / (u kerf width).</li>
 * <li>A power hack saw cuts only on its forward stroke, a feed a stroke: v_f = feed x strokes a second.</li>
 * <li>Cutting through a height h at v_f takes h / v_f; the kerf turns kerf x area x density of the bar into chips.</li>
 * </ul>
 */
public final class Sawing {
    private Sawing() {}

    /** Feed rate of a band or circular saw: feed a tooth (m), blade speed (m/s) and tooth pitch (m), m/s. */
    public static double feedMps(double toothFeedM, double bladeMps, double pitchM) {
        positive(toothFeedM, bladeMps, pitchM);
        return toothFeedM * bladeMps / pitchM;
    }

    /** The fastest feed a saw's power (W) at an efficiency allows in a material (J/m^3) for a kerf (m) and a length of cut (m), m/s. */
    public static double powerLimitedFeedMps(double powerW, double efficiency, double specificEnergyJPerM3, double kerfM, double cutLengthM) {
        positive(powerW, efficiency, specificEnergyJPerM3);
        positive(kerfM, cutLengthM, 1);
        return efficiency * powerW / (specificEnergyJPerM3 * kerfM * cutLengthM);
    }

    /** Feed rate of a power hack saw: feed a stroke (m) and strokes a second, m/s. */
    public static double hacksawFeedMps(double feedPerStrokeM, double strokesPerS) {
        positive(feedPerStrokeM, strokesPerS, 1);
        return feedPerStrokeM * strokesPerS;
    }

    /** Time to cut through a height (m) at a feed rate (m/s), s. */
    public static double cutS(double heightM, double feedMps) {
        positive(feedMps, 1, 1);
        return Math.max(heightM, 0) / feedMps;
    }

    /** Metal a cut turns into chips: the cross-section (m^2), the kerf (m) and the density (kg/m^3), kg. */
    public static double kerfKg(double sectionM2, double kerfM, double density) {
        return Math.max(sectionM2, 0) * Math.max(kerfM, 0) * Math.max(density, 0);
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Feeds, speeds, pitches, powers and sizes must be positive"); }
    }
}

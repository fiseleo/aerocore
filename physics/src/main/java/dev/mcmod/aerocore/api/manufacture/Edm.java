package dev.mcmod.aerocore.api.manufacture;

/**
 * Electric discharge machining (0.35): die sinking, wire cutting and hole drilling, after Groover (Fundamentals of
 * Modern Manufacturing, sec. 26.3) and Kalpakjian and Schmid (ch. 27).
 *
 * <ul>
 * <li>Removal rate (Weller's equation, as Groover gives it): R = K I / T_m^1.23 mm^3/s, I the discharge current (A),
 *     T_m the work's melting point (deg C) and K = 664. Here in SI: m^3/s from A and K.</li>
 * <li>The electrode wears too: the wear ratio is its volume lost over the work's removed (graphite a few per cent in
 *     roughing, copper more; a tube electrode drilling holes loses about as much length as it drills).</li>
 * <li>Wire EDM: the wire cuts a kerf of its diameter plus twice the overcut (spark gap) through a thickness h, so it
 *     advances at R / (kerf h). The wire runs off the spool at its own speed all the while (fresh wire at the gap).</li>
 * <li>The generator's power is the gap voltage times the current times the duty (the share of time the pulse is on).</li>
 * </ul>
 */
public final class Edm {
    private Edm() {}

    /** Weller's constant (mm^3/s from A and deg C). */
    public static final double WELLER_K = 664, WELLER_EXPONENT = 1.23;

    /** Removal rate at a discharge current (A) in a work melting at a temperature (K), m^3/s. */
    public static double removalRate(double currentA, double meltingK) {
        if (!(currentA > 0) || !(meltingK > 273.15)) { throw new IllegalArgumentException("Current positive, melting point above 0 C"); }
        return WELLER_K * currentA / Math.pow(meltingK - 273.15, WELLER_EXPONENT) * 1e-9;
    }

    /** Time to remove a volume (m^3) at a removal rate (m^3/s), s. */
    public static double sinkS(double volumeM3, double removalM3PerS) {
        positive(removalM3PerS, 1, 1);
        return Math.max(volumeM3, 0) / removalM3PerS;
    }

    /** Electrode volume lost removing a volume (m^3) at a wear ratio, m^3. */
    public static double electrodeWearM3(double removedM3, double wearRatio) {
        if (!(wearRatio >= 0)) { throw new IllegalArgumentException("Wear ratio not negative"); }
        return Math.max(removedM3, 0) * wearRatio;
    }

    /** A wire's kerf: its diameter (m) and the overcut each side (m), m. */
    public static double kerfM(double wireDiameterM, double overcutM) {
        positive(wireDiameterM, 1, 1);
        return wireDiameterM + 2 * Math.max(overcutM, 0);
    }

    /** Wire EDM's cutting feed: removal rate (m^3/s), kerf (m) and the work's thickness (m), m/s. */
    public static double wireFeedMps(double removalM3PerS, double kerfM, double thicknessM) {
        positive(removalM3PerS, kerfM, thicknessM);
        return removalM3PerS / (kerfM * thicknessM);
    }

    /** Wire used: its running speed (m/s) for a time (s), m. */
    public static double wireUsedM(double wireMps, double seconds) {
        return Math.max(wireMps, 0) * Math.max(seconds, 0);
    }

    /** Time to EDM-drill holes of a diameter and depth (m) at a removal rate (m^3/s), s. */
    public static double drillS(int holes, double diameterM, double depthM, double removalM3PerS) {
        positive(diameterM, removalM3PerS, 1);
        return Math.max(holes, 0) * Math.PI / 4 * diameterM * diameterM * Math.max(depthM, 0) / removalM3PerS;
    }

    /** The generator's power: gap voltage (V), current (A) and pulse duty (0..1), W. */
    public static double powerW(double gapVolts, double currentA, double duty) {
        if (!(duty > 0 && duty <= 1)) { throw new IllegalArgumentException("Duty in (0, 1]"); }
        return Math.max(gapVolts, 0) * Math.max(currentA, 0) * duty;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Rates, sizes and speeds must be positive"); }
    }
}

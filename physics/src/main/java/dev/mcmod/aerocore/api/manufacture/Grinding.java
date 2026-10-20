package dev.mcmod.aerocore.api.manufacture;

/**
 * Grinding (0.35), after Groover (Fundamentals of Modern Manufacturing, ch. 25), Kalpakjian and Schmid (ch. 26) and
 * Malkin and Guo (Grinding Technology, 2nd ed.).
 *
 * <ul>
 * <li>Removal: a surface grinder taking a depth d over a width w at work speed v_w removes v_w w d a second; the power
 *     is that rate times the specific grinding energy u, several times the energy of turning (small chips, negative
 *     rake, rubbing and ploughing; Malkin and Guo ch. 6).</li>
 * <li>Wheel wear: the grinding ratio G is the work's volume removed over the wheel's volume worn, so the wheel loses
 *     V / G (Kalpakjian and Schmid: 2 to 200 for conventional wheels, far more for CBN).</li>
 * <li>Surface grinding (reciprocating table): strokes of the table's length plus an overtravel at each end, at the work
 *     speed; a stroke a crossfeed across the width, a pass of strokes an infeed down the depth.</li>
 * <li>Cylindrical grinding between centres: the work turns at N_w = v_w / (pi D); traversing a length L at an axial
 *     feed f_a a revolution takes L / (f_a N_w) a pass, a pass an infeed down the radial stock. Plunge grinding feeds
 *     the wheel straight in: the stock over the infeed rate, then a spark-out.</li>
 * <li>Centreless through-feed: the regulating wheel, tilted by I, drives the work along at f_r = pi D_r N_r sin I
 *     (Groover sec. 25.1); a batch fed end to end passes in (count L + wheel width) / f_r.</li>
 * </ul>
 */
public final class Grinding {
    private Grinding() {}

    /** Removal rate of a surface grinder: work speed (m/s), width (m) and depth (m) of the cut, m^3/s. */
    public static double removalRate(double workSpeedMps, double widthM, double depthM) {
        positive(workSpeedMps, widthM, depthM);
        return workSpeedMps * widthM * depthM;
    }

    /** Power to remove at a rate (m^3/s) with a specific grinding energy (J/m^3), W. */
    public static double powerW(double specificEnergyJPerM3, double removalM3PerS) {
        positive(specificEnergyJPerM3, 1, 1);
        return specificEnergyJPerM3 * Math.max(removalM3PerS, 0);
    }

    /** The wheel's volume worn removing a volume (m^3) at a grinding ratio, m^3. */
    public static double wheelWearM3(double removedM3, double grindingRatio) {
        positive(grindingRatio, 1, 1);
        return Math.max(removedM3, 0) / grindingRatio;
    }

    /**
     * Time to surface-grind a face of a length and width (m) down a depth (m) at a work speed (m/s), a crossfeed (m) a
     * stroke, an infeed (m) a pass and an overtravel (m) at each end of a stroke, s. Whole strokes and passes.
     */
    public static double surfaceS(double lengthM, double widthM, double depthM, double workSpeedMps, double crossfeedM, double infeedM, double overtravelM) {
        positive(workSpeedMps, crossfeedM, infeedM);
        if (lengthM <= 0 || widthM <= 0 || depthM <= 0) { return 0; }
        double strokes = Math.ceil(widthM / crossfeedM - 1e-9), passes = Math.ceil(depthM / infeedM - 1e-9);
        return passes * strokes * (lengthM + 2 * Math.max(overtravelM, 0)) / workSpeedMps;
    }

    /**
     * Time to traverse-grind a length (m) of a diameter (m) down a radial stock (m) between centres, at a work surface
     * speed (m/s), an axial feed (m a revolution) and an infeed (m a pass), s. Whole passes.
     */
    public static double cylindricalS(double lengthM, double diameterM, double stockM, double workSpeedMps, double axialFeedM, double infeedM) {
        positive(workSpeedMps, axialFeedM, infeedM);
        positive(diameterM, 1, 1);
        if (lengthM <= 0 || stockM <= 0) { return 0; }
        double passes = Math.ceil(stockM / infeedM - 1e-9);
        return passes * Math.PI * diameterM * lengthM / (axialFeedM * workSpeedMps);
    }

    /** Time to plunge-grind a radial stock (m) at an infeed rate (m/s), with a spark-out (s), s. */
    public static double plungeS(double stockM, double infeedMps, double sparkOutS) {
        positive(infeedMps, 1, 1);
        return Math.max(stockM, 0) / infeedMps + Math.max(sparkOutS, 0);
    }

    /** Through-feed rate of a centreless grinder: the regulating wheel's diameter (m), speed (rev/s) and inclination (rad), m/s. */
    public static double throughfeedMps(double regulatingDiameterM, double regulatingRevPerS, double inclinationRad) {
        positive(regulatingDiameterM, regulatingRevPerS, 1);
        if (!(inclinationRad > 0 && inclinationRad < Math.PI / 2)) { throw new IllegalArgumentException("Inclination between 0 and 90 degrees"); }
        return Math.PI * regulatingDiameterM * regulatingRevPerS * Math.sin(inclinationRad);
    }

    /** Time to through-feed a batch of pieces of a length (m) end to end past a wheel of a width (m) at a through-feed rate (m/s), s. */
    public static double centrelessS(double pieceLengthM, int count, double wheelWidthM, double throughfeedMps) {
        positive(throughfeedMps, 1, 1);
        if (count <= 0) { return 0; }
        return (count * Math.max(pieceLengthM, 0) + Math.max(wheelWidthM, 0)) / throughfeedMps;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Speeds, feeds, sizes and energies must be positive"); }
    }
}

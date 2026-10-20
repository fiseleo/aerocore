package dev.mcmod.aerocore.api.manufacture;

/**
 * ISO 286-1 standard tolerance grades (0.25): the international tolerance (IT) of a size and a grade, and the
 * coarsest grade that holds a tolerance.
 *
 * <ul>
 * <li>Sizes are taken in ISO's steps (0-3, 3-6, 6-10, 10-18, 18-30, 30-50, 50-80, 80-120, 120-180, 180-250, 250-315,
 *     315-400, 400-500 mm, then 500-630 ... 2500-3150 mm); a step's tolerance uses the geometric mean D of its bounds
 *     (1 and 3 mm for the first).</li>
 * <li>Standard tolerance factor: i = 0.45 D^(1/3) + 0.001 D micrometres up to 500 mm, I = 0.004 D + 2.1 above.</li>
 * <li>IT5 to IT18: 7, 10, 16, 25, 40, 64, 100, 160, 250, 400, 640, 1000, 1600, 2500 times the factor; IT1 is
 *     0.8 + 0.020 D micrometres (up to 500 mm; 2 I above) and IT2 to IT4 lie geometrically between IT1 and IT5.</li>
 * </ul>
 * The values are the formulas' (the standard's tables round them: IT7 at 18-30 mm is 20.9 here, 21 in the table).
 */
public final class Tolerance {
    private Tolerance() {}

    /** Size step bounds, mm. */
    static final double[] STEPS = {0, 3, 6, 10, 18, 30, 50, 80, 120, 180, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150};
    /** Multipliers of the tolerance factor for IT5..IT18. */
    static final double[] FACTORS = {7, 10, 16, 25, 40, 64, 100, 160, 250, 400, 640, 1000, 1600, 2500};
    public static final int FINEST = 1, COARSEST = 18;

    /** The geometric mean of the size step a size (m) falls in, mm. */
    static double stepMean(double sizeM) {
        double d = Math.max(sizeM * 1000, 1e-6);
        for (int k = 1; k < STEPS.length; k++) {
            if (d <= STEPS[k]) { return Math.sqrt(Math.max(STEPS[k - 1], 1) * STEPS[k]); }
        }
        return Math.sqrt(STEPS[STEPS.length - 2] * STEPS[STEPS.length - 1]);
    }

    /** The standard tolerance factor (i or I) of a size, micrometres. */
    public static double factorMicrometres(double sizeM) {
        double d = stepMean(sizeM);
        return d <= 500 ? 0.45 * Math.cbrt(d) + 0.001 * d : 0.004 * d + 2.1;
    }

    /** The tolerance of a size (m) in a grade (1..18), m. */
    public static double toleranceM(double sizeM, int grade) {
        if (grade < FINEST || grade > COARSEST) { throw new IllegalArgumentException("IT grades are 1 to 18"); }
        double i = factorMicrometres(sizeM), d = stepMean(sizeM);
        if (grade >= 5) { return FACTORS[grade - 5] * i * 1e-6; }
        double it1 = d <= 500 ? 0.8 + 0.020 * d : 2 * i, it5 = FACTORS[0] * i;
        return it1 * Math.pow(it5 / it1, (grade - 1) / 4.0) * 1e-6;
    }

    /** The coarsest grade whose tolerance at a size is within a tolerance (m); FINEST when even IT1 is too coarse. */
    public static int grade(double sizeM, double toleranceM) {
        for (int g = COARSEST; g > FINEST; g--) {
            if (toleranceM(sizeM, g) <= toleranceM * (1 + 1e-9)) { return g; }
        }
        return FINEST;
    }
}

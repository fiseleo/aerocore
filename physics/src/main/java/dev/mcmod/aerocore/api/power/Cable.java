package dev.mcmod.aerocore.api.power;

/**
 * Power cables (0.26): current-carrying capacity of copper conductors with PVC insulation, three loaded conductors,
 * clipped direct (installation method C), 30 C air: IEC 60364-5-52, table B.52.4. A three-phase line of a voltage V
 * (line to line) and power factor pf carries sqrt(3) V I pf. Resistance of copper at 20 C: 0.0172 ohm mm^2 / m.
 */
public final class Cable {
    private Cable() {}

    private static final double[][] AMPACITY = {{1.5, 17.5}, {2.5, 24}, {4, 32}, {6, 41}, {10, 57}, {16, 76}, {25, 96}, {35, 119}, {50, 144},
            {70, 184}, {95, 223}, {120, 259}, {150, 299}, {185, 341}, {240, 403}};
    public static final double COPPER_OHM_MM2_PER_M = 0.0172;

    /** Current a conductor of a section (mm^2) carries continuously, A (interpolated; below the table proportionally). */
    public static double ampacityA(double sectionMm2) {
        if (sectionMm2 <= AMPACITY[0][0]) { return AMPACITY[0][1] * Math.max(sectionMm2, 0) / AMPACITY[0][0]; }
        for (int i = 1; i < AMPACITY.length; i++) {
            if (sectionMm2 <= AMPACITY[i][0]) {
                double t = (sectionMm2 - AMPACITY[i - 1][0]) / (AMPACITY[i][0] - AMPACITY[i - 1][0]);
                return AMPACITY[i - 1][1] + t * (AMPACITY[i][1] - AMPACITY[i - 1][1]);
            }
        }
        double[] last = AMPACITY[AMPACITY.length - 1];
        return last[1] * Math.pow(sectionMm2 / last[0], 0.8);
    }

    /** Power a three-phase cable carries at a line voltage and power factor, W. */
    public static double powerW(double sectionMm2, double lineVolts, double powerFactor) {
        return Math.sqrt(3) * lineVolts * ampacityA(sectionMm2) * powerFactor;
    }

    /** Resistive loss of a three-phase run carrying a current (A) over a length (m), W. */
    public static double lossW(double currentA, double lengthM, double sectionMm2) {
        return 3 * currentA * currentA * COPPER_OHM_MM2_PER_M * lengthM / sectionMm2;
    }
}

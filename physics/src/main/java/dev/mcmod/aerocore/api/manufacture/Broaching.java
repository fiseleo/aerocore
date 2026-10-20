package dev.mcmod.aerocore.api.manufacture;

/**
 * Broaching (0.35), after Groover (Fundamentals of Modern Manufacturing, sec. 22.5) and Kalpakjian and Schmid (ch. 23).
 *
 * <ul>
 * <li>A broach's teeth each rise t_r over the one before, so cutting a depth d takes d / t_r cutting teeth (whole
 *     teeth), plus a few semi-finishing and finishing teeth of no rise. At a pitch p the toothed length is teeth x p,
 *     and a work of length L has about L / p teeth in it at once (at least one).</li>
 * <li>The force is the specific cutting energy times the chip each engaged tooth takes (the cut's width times the rise)
 *     times the teeth engaged; the power is force times speed.</li>
 * <li>A stroke pulls the whole broach past the work, (broach length + work length) / v, and returns it at the return
 *     speed.</li>
 * </ul>
 */
public final class Broaching {
    private Broaching() {}

    /** Cutting teeth to take a depth (m) at a rise a tooth (m): whole teeth. */
    public static int cuttingTeeth(double depthM, double riseM) {
        positive(riseM, 1, 1);
        return depthM <= 0 ? 0 : (int) Math.ceil(depthM / riseM - 1e-9);
    }

    /** Teeth in a work of a length (m) at once, at a pitch (m): at least one. */
    public static int engagedTeeth(double workLengthM, double pitchM) {
        positive(pitchM, 1, 1);
        return Math.max(1, (int) Math.ceil(Math.max(workLengthM, 0) / pitchM - 1e-9));
    }

    /** The broaching force: specific cutting energy (J/m^3), the cut's width (m), the rise a tooth (m) and the teeth engaged, N. */
    public static double forceN(double specificEnergyJPerM3, double widthM, double riseM, int engaged) {
        positive(specificEnergyJPerM3, widthM, riseM);
        return specificEnergyJPerM3 * widthM * riseM * Math.max(engaged, 1);
    }

    /** One stroke: a broach of a toothed length (m) past a work of a length (m) at a cutting speed (m/s) and back at a return speed (m/s), s. */
    public static double strokeS(double broachLengthM, double workLengthM, double cuttingMps, double returnMps) {
        positive(cuttingMps, returnMps, 1);
        double travel = Math.max(broachLengthM, 0) + Math.max(workLengthM, 0);
        return travel / cuttingMps + travel / returnMps;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Rises, pitches, widths, energies and speeds must be positive"); }
    }
}

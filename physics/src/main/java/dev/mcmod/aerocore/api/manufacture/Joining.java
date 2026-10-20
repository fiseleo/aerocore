package dev.mcmod.aerocore.api.manufacture;

/**
 * Joining (0.25): weld metal, arc time, rivets and spot welds.
 *
 * <ul>
 * <li>A fillet weld of leg a holds a / 2 times a of metal per metre (its triangular section); a butt weld of a plate
 *     of thickness t with a single-V groove of included angle theta and a root gap g holds t^2 tan(theta / 2) + g t.
 *     Weld metal = section x length x density.</li>
 * <li>Arc time: the metal over the process's deposition rate (kg/s; MIG with 1.2 mm steel wire 2-5 kg/h, TIG 0.5-1,
 *     stick 1-2: Lincoln Electric's procedure handbook); the welder's time is the arc time over the operator factor
 *     (the share of time the arc burns: about 0.3 by hand, 0.6-0.9 mechanised).</li>
 * <li>Rivets and spot welds: one per pitch along each row of a seam; their time is their count times the seconds
 *     each takes.</li>
 * </ul>
 */
public final class Joining {
    private Joining() {}

    /** Weld metal section of a fillet of a leg (m), m^2. */
    public static double filletAreaM2(double legM) { return 0.5 * legM * legM; }

    /** Weld metal section of a single-V butt weld in a plate of a thickness (m), groove angle (rad) and root gap (m), m^2. */
    public static double buttAreaM2(double thicknessM, double grooveRad, double gapM) {
        return thicknessM * thicknessM * Math.tan(grooveRad / 2) + gapM * thicknessM;
    }

    /** Weld metal mass of a section (m^2) along a length (m) of a density (kg/m^3), kg. */
    public static double metalKg(double sectionM2, double lengthM, double density) { return sectionM2 * Math.max(lengthM, 0) * density; }

    /** The welder's time for a mass of weld metal at a deposition rate (kg/s) and an operator factor (0..1), s. */
    public static double weldS(double metalKg, double depositionKgPerS, double operatorFactor) {
        if (!(depositionKgPerS > 0) || !(operatorFactor > 0 && operatorFactor <= 1)) { throw new IllegalArgumentException("Deposition positive, operator factor 0-1"); }
        return Math.max(metalKg, 0) / depositionKgPerS / operatorFactor;
    }

    /** Fasteners (rivets, spot welds) along a seam: one per pitch in each row, at least one a row. */
    public static int fasteners(double seamM, double pitchM, int rows) {
        if (!(pitchM > 0)) { throw new IllegalArgumentException("Pitch must be positive"); }
        return seamM <= 0 ? 0 : Math.max(rows, 1) * Math.max(1, (int) Math.ceil(seamM / pitchM));
    }
}

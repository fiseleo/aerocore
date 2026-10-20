package dev.mcmod.aerocore.api.manufacture;

/**
 * Electrochemical machining (0.35), after Groover (sec. 26.2) and Kalpakjian and Schmid (ch. 27).
 *
 * <ul>
 * <li>Faraday's law: a current I dissolves the work at eta I M / (z F) kg a second (M its molar mass, z the valence it
 *     dissolves at, F = 96485 C/mol, eta the current efficiency), a volume eta I M / (z F rho) a second. The
 *     specific removal C = M / (z F rho) (m^3/A s) is a property of the work.</li>
 * <li>The tool advances at the rate the face dissolves: f = eta C I / A over the face's area A, that is eta C J.</li>
 * <li>The gap: the current density through the electrolyte is J = kappa (V - dV) / g (kappa its conductivity, dV the
 *     electrodes' overpotential), so at a steady feed f the gap settles at g = eta C kappa (V - dV) / f.</li>
 * <li>The power is V I; the tool does not wear.</li>
 * </ul>
 */
public final class Ecm {
    private Ecm() {}

    public static final double FARADAY = 96485.33212;

    /** Specific removal of a metal: molar mass (kg/mol), valence and density (kg/m^3), m^3 per coulomb. */
    public static double specificRemoval(double molarMassKg, int valence, double density) {
        if (!(molarMassKg > 0) || valence < 1 || !(density > 0)) { throw new IllegalArgumentException("Molar mass, valence and density must be positive"); }
        return molarMassKg / (valence * FARADAY * density);
    }

    /** Removal rate: current (A), specific removal (m^3/C) and current efficiency, m^3/s. */
    public static double removalRate(double currentA, double specificRemoval, double efficiency) {
        positive(currentA, specificRemoval, efficiency);
        return efficiency * specificRemoval * currentA;
    }

    /** The tool's feed: removal rate (m^3/s) over the face's area (m^2), m/s. */
    public static double feedMps(double removalM3PerS, double areaM2) {
        positive(removalM3PerS, areaM2, 1);
        return removalM3PerS / areaM2;
    }

    /** The equilibrium gap: specific removal (m^3/C), efficiency, electrolyte conductivity (S/m), voltage and overpotential (V), feed (m/s), m. */
    public static double gapM(double specificRemoval, double efficiency, double conductivity, double volts, double overpotential, double feedMps) {
        positive(specificRemoval, efficiency, conductivity);
        positive(feedMps, 1, 1);
        if (!(volts > overpotential)) { throw new IllegalArgumentException("The voltage must exceed the overpotential"); }
        return efficiency * specificRemoval * conductivity * (volts - overpotential) / feedMps;
    }

    /** Time to dissolve a volume (m^3) at a removal rate (m^3/s), s. */
    public static double machineS(double volumeM3, double removalM3PerS) {
        positive(removalM3PerS, 1, 1);
        return Math.max(volumeM3, 0) / removalM3PerS;
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Currents, rates, areas and properties must be positive"); }
    }
}

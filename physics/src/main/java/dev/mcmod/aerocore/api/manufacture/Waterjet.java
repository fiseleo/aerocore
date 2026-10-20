package dev.mcmod.aerocore.api.manufacture;

/**
 * Abrasive waterjet cutting (0.35), after Hashish (1989, "A model for abrasive-waterjet (AWJ) machining", J. Eng.
 * Mater. Technol. 111) and Groover (sec. 26.1).
 *
 * <ul>
 * <li>The orifice turns the pump's pressure p into a jet at v = C_v sqrt(2 p / rho) (Bernoulli with a velocity
 *     coefficient 0.9-0.98); it passes Q = C_d A sqrt(2 p / rho) (discharge coefficient about 0.7), the pump's
 *     hydraulic power is p Q.</li>
 * <li>Mixing (momentum transfer, Hashish): the abrasive leaves the focusing tube at v_a = eta v / (1 + R), R the
 *     abrasive's mass flow over the water's and eta about 0.7-0.8.</li>
 * <li>Erosion: the particles' kinetic energy, m_a v_a^2 / 2 a second, removes the work at that power over its specific
 *     erosion energy (J/m^3, a fitted property: tens of J/mm^3 for steel); the cut advances at the removal over the
 *     kerf times the thickness.</li>
 * </ul>
 */
public final class Waterjet {
    private Waterjet() {}

    /** Jet speed: pressure (Pa), the water's density (kg/m^3) and the velocity coefficient, m/s. */
    public static double jetMps(double pressurePa, double density, double velocityCoefficient) {
        positive(pressurePa, density, velocityCoefficient);
        return velocityCoefficient * Math.sqrt(2 * pressurePa / density);
    }

    /** Water flow through an orifice of a diameter (m) at a pressure (Pa) with a discharge coefficient, m^3/s. */
    public static double waterM3PerS(double orificeM, double pressurePa, double density, double dischargeCoefficient) {
        positive(orificeM, pressurePa, density);
        positive(dischargeCoefficient, 1, 1);
        return dischargeCoefficient * Math.PI / 4 * orificeM * orificeM * Math.sqrt(2 * pressurePa / density);
    }

    /** The pump's hydraulic power: pressure (Pa) times flow (m^3/s), W. */
    public static double hydraulicW(double pressurePa, double flowM3PerS) { return Math.max(pressurePa, 0) * Math.max(flowM3PerS, 0); }

    /** The abrasive's speed: the jet's speed (m/s), abrasive over water mass flow, and the momentum transfer efficiency, m/s. */
    public static double abrasiveMps(double jetMps, double loading, double efficiency) {
        positive(jetMps, efficiency, 1);
        if (!(loading >= 0)) { throw new IllegalArgumentException("Loading not negative"); }
        return efficiency * jetMps / (1 + loading);
    }

    /** Removal rate: abrasive flow (kg/s), its speed (m/s) and the work's specific erosion energy (J/m^3), m^3/s. */
    public static double removalRate(double abrasiveKgPerS, double abrasiveMps, double erosionJPerM3) {
        positive(erosionJPerM3, 1, 1);
        return 0.5 * Math.max(abrasiveKgPerS, 0) * abrasiveMps * abrasiveMps / erosionJPerM3;
    }

    /** Cutting speed: removal rate (m^3/s) over the kerf (m) and thickness (m), m/s. */
    public static double cutMps(double removalM3PerS, double kerfM, double thicknessM) {
        positive(removalM3PerS, kerfM, thicknessM);
        return removalM3PerS / (kerfM * thicknessM);
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Pressures, sizes, flows and energies must be positive"); }
    }
}

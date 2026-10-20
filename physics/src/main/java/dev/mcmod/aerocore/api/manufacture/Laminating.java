package dev.mcmod.aerocore.api.manufacture;

/**
 * Composites and rubber (0.25): laying up plies, curing a laminate, vulcanizing a tyre.
 *
 * <ul>
 * <li>Lay-up: plies of an area laid at a rate (m^2 of ply an hour: hand lay-up of woven cloth about 1-2 m^2/h a worker,
 *     automated tape laying ten times that; Campbell, Manufacturing Processes for Advanced Composites, ch. 3), so
 *     t = A n / rate.</li>
 * <li>Cure: heating the laminate through (Forming.soakS at its half-thickness) and then the resin's hold at its cure
 *     temperature (an epoxy prepreg 2 h at 120-180 C, a room-temperature epoxy about a day).</li>
 * <li>Vulcanization: the heat has to reach the thickest rubber, so the cure time grows as the thickness squared from a
 *     reference (a passenger tyre about 10-15 min in its press, a truck tyre an hour or more): t = t_ref (h / h_ref)^2.</li>
 * </ul>
 */
public final class Laminating {
    private Laminating() {}

    /** Time to lay plies of an area (m^2) at a rate (m^2/s), s. */
    public static double layupS(double areaM2, int plies, double rateM2PerS) {
        if (!(rateM2PerS > 0)) { throw new IllegalArgumentException("Rate must be positive"); }
        return Math.max(areaM2, 0) * Math.max(plies, 0) / rateM2PerS;
    }

    /** Time to cure: heat through a half-thickness (m) of a laminate of a conductivity, density and specific heat, then hold (s). */
    public static double cureS(double halfThicknessM, double conductivity, double density, double specificHeat, double holdS) {
        return Forming.soakS(halfThicknessM, conductivity, density, specificHeat, 1) + Math.max(holdS, 0);
    }

    /** Vulcanizing time of rubber of a thickness (m) from a reference time and thickness, s. */
    public static double vulcanizeS(double thicknessM, double referenceS, double referenceThicknessM) {
        double r = Math.max(thicknessM, 0) / referenceThicknessM;
        return referenceS * r * r;
    }
}

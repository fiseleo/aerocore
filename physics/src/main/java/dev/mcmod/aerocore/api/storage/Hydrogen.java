package dev.mcmod.aerocore.api.storage;

/**
 * Hydrogen as a stored fuel (0.28): the liquid at its normal boiling point, the gas compressed to 350-700 bar, what
 * compressing and liquefying cost, and how fast a liquid store boils off. SI units.
 * <p>
 * Properties of normal hydrogen (NIST Chemistry WebBook, Leachman et al. 2009 equation of state): the liquid at
 * 101.325 kPa boils at 20.37 K, 70.85 kg/m^3, heat of vaporization 445.6 kJ/kg. The gas's compressibility at room
 * temperature is close to linear in pressure: Z = 1 + alpha(T) p with alpha = 0.0068 per MPa at 293.15 K and alpha
 * proportional to 1/T (the second virial coefficient is nearly constant there) — a fit to Z of 1.201 at 30 MPa and
 * 1.344 at 50 MPa and about 1.47 at 70 MPa (Moradi and Groth, "Hydrogen storage and delivery: review of the state of
 * the art technologies", arXiv 1702.06015, from NIST), within 1 % from 0 to 70 MPa at 273-313 K.
 */
public final class Hydrogen {
    private Hydrogen() {}

    /** Specific gas constant, J/(kg K). */
    public static final double GAS_CONSTANT = 4124.2;
    public static final double BOILING_K = 20.37, LIQUID_DENSITY_KG_PER_M3 = 70.85, VAPORIZATION_J_PER_KG = 445.6e3;
    /** Compressibility slope at 293.15 K, per Pa. */
    private static final double ALPHA_293 = 0.0068e-6;

    /** Compressibility factor Z of the gas at pressure p and temperature T. */
    public static double compressibility(double pressurePa, double temperatureK) {
        if (!(pressurePa >= 0) || !(temperatureK > 0)) { throw new IllegalArgumentException("Pressure and temperature must be valid"); }
        return 1 + ALPHA_293 * 293.15 / temperatureK * pressurePa;
    }

    /** Density of the gas, kg/m^3: p / (Z R T). */
    public static double gasDensity(double pressurePa, double temperatureK) {
        return pressurePa / (compressibility(pressurePa, temperatureK) * GAS_CONSTANT * temperatureK);
    }

    /**
     * Work to compress gas from p1 to p2 at temperature T, J/kg: the isothermal work of the real gas, the integral of
     * v dp = R T (ln(p2/p1) + alpha (p2 - p1)), over the compressor's isothermal efficiency (multistage intercooled
     * machines 0.5-0.65). From 20 to 700 bar at 0.55 this is about 2.4 kWh/kg, inside the 2.2-2.8 kWh/kg reported for
     * filling stations (arXiv 1702.06015).
     */
    public static double compressionWorkJPerKg(double fromPa, double toPa, double temperatureK, double isothermalEfficiency) {
        if (!(fromPa > 0) || !(toPa >= fromPa) || !(isothermalEfficiency > 0 && isothermalEfficiency <= 1)) {
            throw new IllegalArgumentException("Compression needs 0 < p1 <= p2 and an efficiency in (0, 1]");
        }
        double alpha = ALPHA_293 * 293.15 / temperatureK;
        return GAS_CONSTANT * temperatureK * (Math.log(toPa / fromPa) + alpha * (toPa - fromPa)) / isothermalEfficiency;
    }

    /**
     * Ideal (reversible) work to liquefy hydrogen from 300 K and 1 bar including the ortho-para conversion, J/kg:
     * 3.92 kWh/kg (Barron, Cryogenic Systems; the figure every liquefier is measured against).
     */
    public static final double IDEAL_LIQUEFACTION_J_PER_KG = 3.92 * 3.6e6;
    /** Figure of merit of a large liquefier (ideal over actual work): Linde's Leuna plant uses about 11.9 kWh/kg. */
    public static final double LIQUEFIER_FIGURE_OF_MERIT = 0.33;

    /** Work to liquefy, J/kg, for a liquefier of a figure of merit (ideal over actual). */
    public static double liquefactionWorkJPerKg(double figureOfMerit) {
        if (!(figureOfMerit > 0 && figureOfMerit <= 1)) { throw new IllegalArgumentException("Figure of merit in (0, 1]"); }
        return IDEAL_LIQUEFACTION_J_PER_KG / figureOfMerit;
    }

    /** Liquid boiled off by a heat leak into the store, kg/s (the vapour vented). */
    public static double boilOffKgPerS(double heatLeakW) { return Math.max(0, heatLeakW) / VAPORIZATION_J_PER_KG; }
}

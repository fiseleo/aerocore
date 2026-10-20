package dev.mcmod.aerocore.api.manufacture;

/**
 * Separating air into oxygen, nitrogen and argon (0.35).
 *
 * <ul>
 * <li>Dry air by mass: nitrogen 0.7552, oxygen 0.2314, argon 0.0129, the rest mostly carbon dioxide (by volume 0.7808,
 *     0.2095, 0.0093; the US Standard Atmosphere's composition).</li>
 * <li>The least work to draw a pure component of mole fraction x out of a great deal of air at T is R T ln(1 / x) a
 *     mole (the free energy of mixing): for oxygen at 298 K about 0.034 kWh/kg. A real plant takes that over its
 *     second-law efficiency (the caller's; cryogenic plants reach roughly 0.1-0.2 of it, the small ones less).</li>
 * <li>A plant recovers only part of each component it draws from the air it compresses (the caller's recovery).</li>
 * </ul>
 */
public final class AirSeparation {
    private AirSeparation() {}

    public static final double GAS_CONSTANT = 8.314462618;
    /** Mass fractions of dry air: nitrogen, oxygen, argon. */
    public static final double N2_MASS = 0.7552, O2_MASS = 0.2314, AR_MASS = 0.0129;
    /** Mole fractions of dry air: nitrogen, oxygen, argon. */
    public static final double N2_MOLE = 0.7808, O2_MOLE = 0.2095, AR_MOLE = 0.0093;

    /** Least work to draw a kilogram of a component of a mole fraction and molar mass (kg/mol) out of air at a temperature (K), J/kg. */
    public static double minimumJPerKg(double moleFraction, double molarMassKg, double temperatureK) {
        if (!(moleFraction > 0 && moleFraction < 1) || !(molarMassKg > 0) || !(temperatureK > 0)) { throw new IllegalArgumentException("Bad component"); }
        return GAS_CONSTANT * temperatureK * Math.log(1 / moleFraction) / molarMassKg;
    }

    /** A plant's work a kilogram: the least work over its second-law efficiency (0..1), J/kg. */
    public static double plantJPerKg(double minimumJPerKg, double secondLawEfficiency) {
        if (!(secondLawEfficiency > 0 && secondLawEfficiency <= 1)) { throw new IllegalArgumentException("Efficiency in (0, 1]"); }
        return minimumJPerKg / secondLawEfficiency;
    }

    /** Air (kg) a plant must take in for a mass of a component (kg) at its mass fraction and the plant's recovery (0..1). */
    public static double airKg(double productKg, double massFraction, double recovery) {
        if (!(massFraction > 0 && massFraction < 1) || !(recovery > 0 && recovery <= 1)) { throw new IllegalArgumentException("Fraction and recovery in (0, 1]"); }
        return Math.max(productKg, 0) / (massFraction * recovery);
    }
}

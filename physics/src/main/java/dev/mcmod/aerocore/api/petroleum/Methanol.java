package dev.mcmod.aerocore.api.petroleum;

/**
 * Methanol from light hydrocarbons (0.40): steam reforming to synthesis gas and synthesis over a copper catalyst.
 *
 * <ul>
 * <li>Overall, for propane: C3H8 + 3 H2O -> 3 CH3OH + H2 (reforming C3H8 + 3 H2O -> 3 CO + 7 H2, synthesis
 *     CO + 2 H2 -> CH3OH, a hydrogen left over). 1.0 kg of propane and 1.23 kg of water make 2.18 kg of methanol
 *     and 0.05 kg of hydrogen at the stoichiometry.</li>
 * <li>A plant burns more feed to fire the reformer's furnace and loses some to purge gas: modern natural-gas plants
 *     take 29-33 GJ (LHV) of gas a tonne of methanol, about 1.45 times the 20 GJ the methanol holds (Ullmann's,
 *     "Methanol"). The caller's fired share is the extra feed burned a kilogram of feed reacted (about 0.45).</li>
 * <li>Propane's lower heating value 46.35 MJ/kg, methanol's 19.9 MJ/kg.</li>
 * </ul>
 */
public final class Methanol {
    private Methanol() {}

    public static final double PROPANE_KG = 0.044097, WATER_KG = 0.018015, METHANOL_KG = 0.032042, HYDROGEN_KG = 0.002016;
    public static final double PROPANE_LHV = 46.35e6, METHANOL_LHV = 19.9e6;

    /** Kilograms of each a kilogram of methanol: propane reacted, water, hydrogen left over. */
    public static double propaneKgPerKg() { return PROPANE_KG / (3 * METHANOL_KG); }

    public static double waterKgPerKg() { return 3 * WATER_KG / (3 * METHANOL_KG); }

    public static double hydrogenKgPerKg() { return HYDROGEN_KG / (3 * METHANOL_KG); }

    /** Propane a kilogram of methanol takes in all, with a fired share burned beyond what reacts, kg. */
    public static double feedKgPerKg(double firedShare) {
        if (!(firedShare >= 0)) { throw new IllegalArgumentException("Fired share must be nonnegative"); }
        return propaneKgPerKg() * (1 + firedShare);
    }

    /** Feed energy (LHV) a kilogram of methanol takes, J/kg. */
    public static double feedJPerKg(double firedShare) { return feedKgPerKg(firedShare) * PROPANE_LHV; }
}

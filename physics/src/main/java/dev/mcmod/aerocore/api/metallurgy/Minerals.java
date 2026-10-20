package dev.mcmod.aerocore.api.metallurgy;

/**
 * Ore minerals (0.40): the metal a kilogram of the pure mineral holds, and a typical run-of-mine share of the mineral
 * (the grade a mined lump is taken to have).
 *
 * <ul>
 * <li>Metal fractions from the formulas and the IUPAC atomic weights: gibbsite Al(OH)3 34.6 % Al; chromite FeCr2O4
 *     46.5 % Cr; ilmenite FeTiO3 31.6 % Ti (36.8 % Fe); magnesite MgCO3 28.8 % Mg; cassiterite SnO2 78.8 % Sn;
 *     sphalerite ZnS 67.1 % Zn; pentlandite (Fe,Ni)9S8 (Fe4.5Ni4.5S8) 34.2 % Ni.</li>
 * <li>Typical mineral shares of a good ore (USGS Mineral Commodity Summaries and the deposit literature): bauxite about
 *     77 % gibbsite (50 % Al2O3); metallurgical chromite ore at its usual 45 % Cr2O3 holds two thirds of pure
 *     FeCr2O4's chromium (the natural spinel carries Mg, Al and Fe3+ too); beach-sand ilmenite concentrate about
 *     95 %; magnesite rock about 95 %. Tin, zinc and nickel are taken as concentrates (as Minecraft's raw iron is).</li>
 * </ul>
 */
public enum Minerals {
    GIBBSITE(Species.ALOH3, Species.AL, 1, 0.77),
    CHROMITE(Species.FECR2O4, Species.CR, 2, 0.66),
    ILMENITE(Species.FETIO3, Species.TI, 1, 0.95),
    MAGNESITE(Species.MGCO3, Species.MG, 1, 0.95);

    /** The mineral, its metal, atoms of the metal a formula unit, and the mineral's share of a typical ore. */
    public final Species mineral, metal;
    public final int atoms;
    public final double oreShare;

    Minerals(Species mineral, Species metal, int atoms, double oreShare) {
        this.mineral = mineral;
        this.metal = metal;
        this.atoms = atoms;
        this.oreShare = oreShare;
    }

    /** Metal a kilogram of the pure mineral holds, kg. */
    public double metalFraction() { return atoms * metal.molarMassKg / mineral.molarMassKg; }

    /** Metal a kilogram of typical ore holds, kg. */
    public double oreGrade() { return metalFraction() * oreShare; }

    /** Metal fraction of a formula: atoms of a metal of a molar mass in a compound of a molar mass (both kg/mol). */
    public static double fraction(int atoms, double metalMolarMassKg, double compoundMolarMassKg) {
        return atoms * metalMolarMassKg / compoundMolarMassKg;
    }
}

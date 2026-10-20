package dev.mcmod.aerocore.api.metallurgy;

/**
 * Electrolysis (0.40): the electricity a kilogram of product takes from Faraday's law, the cell voltage and the current
 * efficiency.
 *
 * <ul>
 * <li>Faraday's law: a mole of product takes z F coulombs (z electrons a formula unit, F = 96485.33212 C/mol); at a
 *     current efficiency eta only that share of the current makes product. The energy a mole is z F U / eta at the
 *     cell voltage U (the operating voltage: decomposition, overvoltages, the ohmic drop in the bath and the
 *     busbars).</li>
 * <li>Typical cells (each the caller's to change), with the specific energy they give:
 *   <ul>
 *   <li>Hall-Heroult aluminium: 4.1 V at 94 %: 13.0 kWh/kg (modern pot lines 12.5-15; Grjotheim and Kvande,
 *       Introduction to Aluminium Electrolysis, 1993; IAI statistics). The carbon anode burns to CO2: 0.334 kg C
 *       a kg of aluminium by the reaction 2 Al2O3 + 3 C -> 4 Al + 3 CO2; net consumption runs 0.40-0.45 (air burn,
 *       dusting), about 1.25 times that.</li>
 *   <li>Magnesium from molten MgCl2: 5.0 V at 85 %: 13.0 kWh/kg (10-14 in modern cells, Dow's older cells 18;
 *       Ullmann's, "Magnesium"); 2.92 kg of chlorine a kg.</li>
 *   <li>Sodium chlorate from brine: 6 electrons, 3.2 V at 90 %: 5.4 kWh/kg (industry 5.0-6.0); perchlorate from
 *       chlorate: 2 electrons, 6.0 V at 85 %: 3.1 kWh/kg of NaClO4 (2.5-3.5); Schumacher, Perchlorates (1960) and
 *       Ullmann's, "Chlorine oxides and chlorine oxygen acids".</li>
 *   <li>Water (alkaline): 1.9 V at 98 %: 51.6 kWh/kg of hydrogen (commercial 50-55 at the stack).</li>
 *   </ul></li>
 * </ul>
 */
public final class Electrolysis {
    private Electrolysis() {}

    public static final double FARADAY = 96485.33212;
    private static final double J_PER_KWH = 3.6e6;

    /** A cell: its product's molar mass (kg/mol), electrons a formula unit, operating voltage and current efficiency. */
    public record Cell(String name, double molarMassKg, int electrons, double volts, double currentEfficiency) {
        public Cell {
            if (!(molarMassKg > 0) || electrons < 1 || !(volts > 0) || !(currentEfficiency > 0 && currentEfficiency <= 1)) {
                throw new IllegalArgumentException("Bad cell " + name);
            }
        }

        /** Electricity a mole of product takes, J/mol. */
        public double jPerMol() { return electrons * FARADAY * volts / currentEfficiency; }

        /** Electricity a kilogram of product takes, J/kg. */
        public double jPerKg() { return jPerMol() / molarMassKg; }

        /** The same in kWh/kg. */
        public double kWhPerKg() { return jPerKg() / J_PER_KWH; }

        /** Product a cell makes at a current (A), kg/s. */
        public double kgPerS(double amperes) { return Math.max(amperes, 0) * currentEfficiency * molarMassKg / (electrons * FARADAY); }

        /** The same cell at another voltage. */
        public Cell at(double volts) { return new Cell(name, molarMassKg, electrons, volts, currentEfficiency); }
    }

    public static final Cell HALL_HEROULT = new Cell("aluminium", Species.AL.molarMassKg, 3, 4.1, 0.94);
    public static final Cell MAGNESIUM_CHLORIDE = new Cell("magnesium", Species.MG.molarMassKg, 2, 5.0, 0.85);
    public static final Cell SODIUM_CHLORATE = new Cell("sodium chlorate", 0.106440, 6, 3.2, 0.90);
    public static final Cell SODIUM_PERCHLORATE = new Cell("sodium perchlorate", 0.122440, 2, 6.0, 0.85);
    public static final Cell WATER = new Cell("hydrogen", Species.H2.molarMassKg, 2, 1.9, 0.98);

    /** Molar mass of ammonium perchlorate, kg/mol. */
    public static final double AMMONIUM_PERCHLORATE_KG = 0.117490;

    /** The anode carbon the Hall-Heroult reaction burns, kg a kg of aluminium (2 Al2O3 + 3 C -> 4 Al + 3 CO2). */
    public static double anodeCarbonKgPerKg() {
        return 3 * Species.C.molarMassKg / (4 * Species.AL.molarMassKg);
    }

    /** The alumina a kilogram of aluminium takes, kg. */
    public static double aluminaKgPerKg() {
        return Species.AL2O3.molarMassKg / (2 * Species.AL.molarMassKg);
    }

    /** Chlorine set free a kilogram of magnesium, kg. */
    public static double chlorineKgPerKgMagnesium() {
        return Species.CL2.molarMassKg / Species.MG.molarMassKg;
    }

    /**
     * Electricity a kilogram of ammonium perchlorate takes by way of sodium chlorate and sodium perchlorate (one mole of
     * each a mole of AP; the metathesis with an ammonium salt takes none), J/kg.
     */
    public static double ammoniumPerchlorateJPerKg(Cell chlorate, Cell perchlorate) {
        return (chlorate.jPerMol() + perchlorate.jPerMol()) / AMMONIUM_PERCHLORATE_KG;
    }
}

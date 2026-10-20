package dev.mcmod.aerocore.api.metallurgy;

/**
 * Furnace reductions and melting (0.40): the reactions of the usual routes and the electricity an electric furnace
 * takes for them.
 *
 * <ul>
 * <li>An electric furnace's energy a kilogram of product is (reaction enthalpy, when it takes heat, + the heat the
 *     products carry out at tapping) / the furnace's electrical efficiency (losses to the shell, the cooling water and
 *     the off-gas); arc furnaces melting scrap also get a share of the melting heat from oxygen and burners (the
 *     caller's).</li>
 * <li>Typical values, each with its source (the caller picks them; the tests check the combinations below against
 *     plant figures):
 *   <ul>
 *   <li>Liquid steel at about 1650 C carries 1.40 MJ/kg and furnace slag about 1.9 MJ/kg (Jones, Bowman and
 *       Lefrank, "Electric furnace steelmaking", in The Making, Shaping and Treating of Steel, 11th ed., 1998).
 *       With a third from chemical energy at 65 % electrical efficiency: 0.40 kWh/kg (plants 0.35-0.45).</li>
 *   <li>75 % ferrosilicon (SiO2 + 2 C -> Si + 2 CO; the iron comes in as scrap) carries about 2.4 MJ/kg at
 *       1800 C (silicon's latent heat is 1.79 MJ/kg); at 70 %: 8.3 kWh/kg (plants 8-9.5; Schei, Tuset and Tveit,
 *       Production of High Silicon Alloys, 1998).</li>
 *   <li>High-carbon ferrochrome straight from chromite (FeCr2O4 + 4 C -> Fe + 2 Cr + 4 CO), tapped with about 1.2 kg
 *       of slag a kg at 78 %: 3.6 kWh/kg (plants 3.2-4.2 in open or semi-closed furnaces; Riekkola-Vanhanen,
 *       Finnish Expert Report on BAT in Ferrochromium Production, 1999).</li>
 *   <li>Titanium slag from ilmenite (FeTiO3 + C -> TiO2 + Fe + CO): slag at about 1650 C 2.0 MJ/kg, iron 0.7 kg a kg
 *       of TiO2; at 75 %: 1.9 kWh/kg of slag (plant figures are around 2 MWh a tonne of slag; Pistorius,
 *       "Ilmenite smelting: the basics", J. S. Afr. Inst. Min. Metall. 108 (2008) 35).</li>
 *   </ul></li>
 * <li>The Kroll process: TiO2 + 2 Cl2 + C -> TiCl4 + CO2 in a chlorinator, then TiCl4 + 2 Mg -> Ti + 2 MgCl2 in a
 *     steel retort near 850 C; both give heat. The magnesium (1.02 kg a kg of titanium) comes back from the MgCl2 by
 *     electrolysis (Electrolysis.MAGNESIUM_CHLORIDE), which is most of the route's electricity.</li>
 * <li>Aluminothermic chromium: Cr2O3 + 2 Al -> 2 Cr + Al2O3 releases 2.6 kJ a gram of charge, above the 2.3 kJ/g a
 *     thermite needs to run by itself (Reaction.selfSustaining).</li>
 * <li>The Pidgeon process: 2 MgO + Si -> 2 Mg + SiO2 takes heat (the lime in the charge binds the silica and the
 *     magnesium distils off under vacuum); 0.58 kg of silicon a kg of magnesium by the reaction.</li>
 * <li>Calcining: gibbsite 2 Al(OH)3 -> Al2O3 + 3 H2O and magnesite MgCO3 -> MgO + CO2 take heat; chromite roasting
 *     4 FeCr2O4 + O2 -> 4 Cr2O3 + 2 Fe2O3 (here standing in for the soda-ash roast and leach) gives it.</li>
 * </ul>
 */
public final class Smelting {
    private Smelting() {}

    private static final double J_PER_KWH = 3.6e6;

    /** Heat liquid steel carries at tapping (about 1650 C) and furnace slag, J/kg. */
    public static final double STEEL_TAP_J_PER_KG = 1.40e6, SLAG_TAP_J_PER_KG = 1.9e6;
    /** Heat 75 % ferrosilicon carries at about 1800 C, J/kg. */
    public static final double FERROSILICON_TAP_J_PER_KG = 2.4e6;
    /** Heat titanium slag carries at about 1650 C, J/kg. */
    public static final double TITANIUM_SLAG_TAP_J_PER_KG = 2.0e6;

    public static final Reaction FERROSILICON = Reaction.of("ferrosilicon", Species.SIO2, 1, Species.C, 2, null, Species.SI, 1, Species.CO, 2);
    public static final Reaction FERROCHROME = Reaction.of("ferrochrome", Species.FECR2O4, 1, Species.C, 4, null, Species.FE, 1, Species.CR, 2, Species.CO, 4);
    public static final Reaction ILMENITE_SMELTING = Reaction.of("ilmenite smelting", Species.FETIO3, 1, Species.C, 1, null, Species.TIO2, 1, Species.FE, 1, Species.CO, 1);
    public static final Reaction CHLORINATION = Reaction.of("chlorination", Species.TIO2, 1, Species.CL2, 2, Species.C, 1, null, Species.TICL4, 1, Species.CO2, 1);
    public static final Reaction KROLL = Reaction.of("Kroll", Species.TICL4, 1, Species.MG, 2, null, Species.TI, 1, Species.MGCL2, 2);
    public static final Reaction ALUMINOTHERMIC_CHROMIUM = Reaction.of("aluminothermic chromium", Species.CR2O3, 1, Species.AL, 2, null, Species.CR, 2, Species.AL2O3, 1);
    public static final Reaction PIDGEON = Reaction.of("Pidgeon", Species.MGO, 2, Species.SI, 1, null, Species.MG, 2, Species.SIO2, 1);
    public static final Reaction GIBBSITE_CALCINING = Reaction.of("gibbsite calcining", Species.ALOH3, 2, null, Species.AL2O3, 1, Species.H2O_GAS, 3);
    public static final Reaction MAGNESITE_CALCINING = Reaction.of("magnesite calcining", Species.MGCO3, 1, null, Species.MGO, 1, Species.CO2, 1);
    public static final Reaction CHROMITE_ROASTING = Reaction.of("chromite roasting", Species.FECR2O4, 4, Species.O2, 1, null, Species.CR2O3, 4, Species.FE2O3, 2);

    /**
     * Electricity a kilogram of product takes in an electric furnace: the reaction's enthalpy a kilogram of product
     * (J/kg; nothing when it gives heat) and the heat the product and its slag carry out (J/kg of product), over the
     * furnace's electrical efficiency (0..1), J/kg.
     */
    public static double furnaceJPerKg(double reactionJPerKg, double tapJPerKg, double efficiency) {
        if (!(efficiency > 0 && efficiency <= 1)) { throw new IllegalArgumentException("Efficiency in (0, 1]"); }
        return (Math.max(reactionJPerKg, 0) + Math.max(tapJPerKg, 0)) / efficiency;
    }

    /** An arc furnace melting scrap: the melting heat (J/kg), the share of it from oxygen and burners, and the electrical efficiency, J/kg. */
    public static double arcMeltJPerKg(double meltJPerKg, double chemicalShare, double efficiency) {
        if (!(chemicalShare >= 0 && chemicalShare < 1)) { throw new IllegalArgumentException("Chemical share in [0, 1)"); }
        return furnaceJPerKg(0, meltJPerKg * (1 - chemicalShare), efficiency);
    }

    /** Liquid steel from scrap in an arc furnace with a third of the heat chemical, at 65 %, J/kg (about 0.40 kWh/kg). */
    public static double steelJPerKg() { return arcMeltJPerKg(STEEL_TAP_J_PER_KG, 1.0 / 3, 0.65); }

    /** 75 % ferrosilicon at 70 % efficiency, J/kg of alloy (about 8.3 kWh/kg). */
    public static double ferrosiliconJPerKg() {
        return furnaceJPerKg(0.75 * FERROSILICON.enthalpyJPerKg(Species.SI), FERROSILICON_TAP_J_PER_KG, 0.70);
    }

    /** High-carbon ferrochrome from chromite with 1.2 kg of slag a kg at 78 % efficiency, J/kg of alloy (about 3.6 kWh/kg). */
    public static double ferrochromeJPerKg() {
        double alloyKg = Species.FE.molarMassKg + 2 * Species.CR.molarMassKg;
        return furnaceJPerKg(FERROCHROME.enthalpyJ() / alloyKg, STEEL_TAP_J_PER_KG + 1.2 * SLAG_TAP_J_PER_KG, 0.78);
    }

    /** Titanium slag from ilmenite at 75 % efficiency, J/kg of slag (about 1.9 kWh/kg); the iron comes out alongside. */
    public static double titaniumSlagJPerKg() {
        double iron = ILMENITE_SMELTING.kgPerKg(Species.FE, Species.TIO2);
        return furnaceJPerKg(ILMENITE_SMELTING.enthalpyJPerKg(Species.TIO2), TITANIUM_SLAG_TAP_J_PER_KG + iron * STEEL_TAP_J_PER_KG, 0.75);
    }

    /** J/kg in kWh/kg. */
    public static double kWh(double jPerKg) { return jPerKg / J_PER_KWH; }
}

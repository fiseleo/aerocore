package dev.mcmod.aerocore.api.metallurgy;

import dev.mcmod.aerocore.api.petroleum.Methanol;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Electrolysis, furnace reductions, minerals and methanol (0.40) against plant figures and the formulas. */
class MetallurgyTest {
    private static final double KWH = 3.6e6;

    private static void within(double lo, double hi, double v, String what) {
        assertTrue(v >= lo && v <= hi, what + " is " + v + ", not in " + lo + "-" + hi);
    }

    @Test void cellsTakeWhatPlantsDo() {
        // Hall-Heroult: 3 x 96485 x 4.1 / 0.026982 / 0.94 = 46.8 MJ/kg = 13.0 kWh/kg (pot lines 12.5-15)
        assertEquals(3 * Electrolysis.FARADAY * 4.1 / 0.026982 / 0.94 / KWH, Electrolysis.HALL_HEROULT.kWhPerKg(), 1e-9);
        within(12.5, 15, Electrolysis.HALL_HEROULT.kWhPerKg(), "aluminium");
        // 1.89 kg of alumina and 0.334 kg of anode carbon a kg of aluminium
        assertEquals(1.889, Electrolysis.aluminaKgPerKg(), 0.001);
        assertEquals(0.334, Electrolysis.anodeCarbonKgPerKg(), 0.001);
        // a 300 kA pot at 94 %: 2.27 t a day
        assertEquals(2.27, Electrolysis.HALL_HEROULT.kgPerS(300e3) * 86400 / 1000, 0.01);
        within(10, 14, Electrolysis.MAGNESIUM_CHLORIDE.kWhPerKg(), "magnesium");
        assertEquals(2.917, Electrolysis.chlorineKgPerKgMagnesium(), 0.001);
        within(5.0, 6.0, Electrolysis.SODIUM_CHLORATE.kWhPerKg(), "sodium chlorate");
        within(2.5, 3.5, Electrolysis.SODIUM_PERCHLORATE.kWhPerKg(), "sodium perchlorate");
        within(50, 55, Electrolysis.WATER.kWhPerKg(), "hydrogen");
        // ammonium perchlorate by way of chlorate and perchlorate: 8 electrons a mole, about 8 kWh/kg
        double ap = Electrolysis.ammoniumPerchlorateJPerKg(Electrolysis.SODIUM_CHLORATE, Electrolysis.SODIUM_PERCHLORATE) / KWH;
        within(7.5, 8.5, ap, "ammonium perchlorate");
        // a higher voltage takes proportionally more
        assertEquals(2 * Electrolysis.HALL_HEROULT.kWhPerKg(), Electrolysis.HALL_HEROULT.at(8.2).kWhPerKg(), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> new Electrolysis.Cell("bad", 0.02, 1, 1, 1.2));
    }

    @Test void reactionsAreHessAndBalanced() {
        // Kroll: 2 x (-641.3) - (-804.2) = -478.4 kJ a mole of titanium; 1.016 kg Mg and 3.978 kg MgCl2 a kg
        assertEquals(-478.4e3, Smelting.KROLL.enthalpyJ(), 1e-6);
        assertEquals(1.0155, Smelting.KROLL.kgPerKg(Species.MG, Species.TI), 1e-3);
        assertEquals(3.978, Smelting.KROLL.kgPerKg(Species.MGCL2, Species.TI), 1e-3);
        assertTrue(Smelting.CHLORINATION.enthalpyJ() < 0);
        // aluminothermic chromium: 0.519 kg Al a kg Cr, 2.6 kJ/g of charge: runs by itself
        assertEquals(0.519, Smelting.ALUMINOTHERMIC_CHROMIUM.kgPerKg(Species.AL, Species.CR), 1e-3);
        assertEquals(2.60e6, Smelting.ALUMINOTHERMIC_CHROMIUM.releasedJPerKgCharge(), 0.01e6);
        assertTrue(Smelting.ALUMINOTHERMIC_CHROMIUM.selfSustaining());
        // aluminium does not reduce magnesia (it takes heat): no thermite there
        Reaction magnesia = Reaction.of("aluminium on magnesia", Species.MGO, 3, Species.AL, 2, null, Species.MG, 3, Species.AL2O3, 1);
        assertTrue(magnesia.enthalpyJ() > 0);
        assertFalse(magnesia.selfSustaining());
        // Pidgeon takes heat, 0.578 kg Si a kg Mg; calcining takes heat, roasting gives it
        assertTrue(Smelting.PIDGEON.enthalpyJ() > 0);
        assertEquals(0.578, Smelting.PIDGEON.kgPerKg(Species.SI, Species.MG), 1e-3);
        within(1.7e6, 1.9e6, Smelting.GIBBSITE_CALCINING.enthalpyJPerKg(Species.AL2O3), "gibbsite to alumina");
        within(2.4e6, 2.6e6, Smelting.MAGNESITE_CALCINING.enthalpyJPerKg(Species.MGO), "magnesite to magnesia");
        assertTrue(Smelting.CHROMITE_ROASTING.enthalpyJ() < 0);
        // an unbalanced reaction is refused
        assertThrows(IllegalArgumentException.class, () -> Reaction.of("bad", Species.SIO2, 1, Species.C, 1, null, Species.SI, 1, Species.CO, 2));
    }

    @Test void electricFurnacesTakeWhatPlantsDo() {
        within(0.35, 0.45, Smelting.kWh(Smelting.steelJPerKg()), "arc furnace steel");
        within(8.0, 9.5, Smelting.kWh(Smelting.ferrosiliconJPerKg()), "75 % ferrosilicon");
        within(3.2, 4.2, Smelting.kWh(Smelting.ferrochromeJPerKg()), "high-carbon ferrochrome");
        within(1.6, 2.4, Smelting.kWh(Smelting.titaniumSlagJPerKg()), "titanium slag");
        // the furnace's heat is the reaction's (when it takes heat) and the tapping's over the efficiency
        assertEquals((1e6 + 2e6) / 0.5, Smelting.furnaceJPerKg(1e6, 2e6, 0.5), 1e-6);
        assertEquals(2e6 / 0.5, Smelting.furnaceJPerKg(-5e6, 2e6, 0.5), 1e-6);
        assertThrows(IllegalArgumentException.class, () -> Smelting.furnaceJPerKg(0, 1, 0));
    }

    @Test void mineralsHoldTheirFormulasMetal() {
        assertEquals(0.3459, Minerals.GIBBSITE.metalFraction(), 1e-4);
        assertEquals(0.4646, Minerals.CHROMITE.metalFraction(), 1e-4);
        assertEquals(0.3155, Minerals.ILMENITE.metalFraction(), 1e-4);
        assertEquals(0.2883, Minerals.MAGNESITE.metalFraction(), 1e-4);
        // bauxite at 77 % gibbsite is 50 % Al2O3; chromite ore 45 % Cr2O3
        assertEquals(0.50, Minerals.GIBBSITE.oreShare * Species.AL2O3.molarMassKg / (2 * Species.ALOH3.molarMassKg), 0.01);
        assertEquals(0.45, Minerals.CHROMITE.oreGrade() * Species.CR2O3.molarMassKg / (2 * Species.CR.molarMassKg), 0.01);
        assertEquals(0.7877, Minerals.fraction(1, Species.SN.molarMassKg, Species.SN.molarMassKg + Species.O2.molarMassKg), 1e-4);
    }

    @Test void carbonToolSteelWearsFarFasterThanHighSpeedSteel() {
        // Groover table 23.2: at 18 m/min on steel a carbon steel edge lasts 60 (20 / 18)^10 = 172 s, high-speed steel hours
        double v = 0.3;
        double carbon = dev.mcmod.aerocore.api.manufacture.ToolLife.lifeS(v, dev.mcmod.aerocore.api.manufacture.ToolLife.C_CARBON_STEEL,
                dev.mcmod.aerocore.api.manufacture.ToolLife.N_CARBON);
        double hss = dev.mcmod.aerocore.api.manufacture.ToolLife.lifeS(v, dev.mcmod.aerocore.api.manufacture.ToolLife.C_HSS_STEEL,
                dev.mcmod.aerocore.api.manufacture.ToolLife.N_HSS);
        assertEquals(60 * Math.pow(20.0 / 18, 10), carbon, 1e-6);
        assertTrue(carbon < hss / 100);
    }

    @Test void methanolIsStoichiometricPlusTheFiredFeed() {
        // C3H8 + 3 H2O -> 3 CH3OH + H2: a kg of methanol from 0.459 kg propane and 0.562 kg water, 0.021 kg hydrogen over
        assertEquals(0.4587, Methanol.propaneKgPerKg(), 1e-4);
        assertEquals(0.5622, Methanol.waterKgPerKg(), 1e-4);
        assertEquals(0.0210, Methanol.hydrogenKgPerKg(), 1e-4);
        assertEquals(1.0, Methanol.propaneKgPerKg() + Methanol.waterKgPerKg() - Methanol.hydrogenKgPerKg(), 1e-3);
        // with 0.45 more burned: 30.8 GJ of feed a tonne (plants 29-33)
        within(29e6, 33e6, Methanol.feedJPerKg(0.45), "methanol feed");
        assertThrows(IllegalArgumentException.class, () -> Methanol.feedKgPerKg(-0.1));
    }
}

package dev.mcmod.aerocore.api.storage;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class HydrogenTest {
    @Test void compressedHydrogenFollowsNistDensities() {
        // Z at 20 C: 1.201 at 30 MPa, 1.344 at 50 MPa, about 1.47 at 70 MPa (arXiv 1702.06015, from NIST)
        assertEquals(1.201, Hydrogen.compressibility(30e6, 293.15), 0.01 * 1.201);
        assertEquals(1.344, Hydrogen.compressibility(50e6, 293.15), 0.01 * 1.344);
        assertEquals(1.47, Hydrogen.compressibility(70e6, 293.15), 0.02 * 1.47);
        // 700 bar at 15 C: about 39-41 kg/m^3
        double rho = Hydrogen.gasDensity(70e6, 288.15);
        assertTrue(rho > 38.7 && rho < 41.5, "700 bar density " + rho);
        // an ideal gas at low pressure
        assertEquals(1e5 / (Hydrogen.GAS_CONSTANT * 300), Hydrogen.gasDensity(1e5, 300), 0.001 * 0.08);
    }

    @Test void compressingAndLiquefyingCostWhatPlantsUse() {
        double kwh = Hydrogen.compressionWorkJPerKg(20e5, 700e5, 293.15, 0.55) / 3.6e6;
        assertTrue(kwh > 2.2 && kwh < 2.8, "20 to 700 bar: " + kwh + " kWh/kg");
        assertEquals(11.9, Hydrogen.liquefactionWorkJPerKg(Hydrogen.LIQUEFIER_FIGURE_OF_MERIT) / 3.6e6, 0.2, "Leuna-class liquefier");
    }

    @Test void aVacuumInsulatedCarTankBoilsOffAFewPercentADay() {
        // 170 l, 0.5 m across, 30 mm of multilayer insulation, 20 C outside (BMW Hydrogen 7 class: a few percent a day)
        double r = 0.25, volume = 0.17, length = volume / (Math.PI * r * r), area = 2 * Math.PI * r * length + 2 * Math.PI * r * r;
        double leak = area * Insulation.VACUUM_MLI.heatFlux(0.03, 293.15, Hydrogen.BOILING_K);
        double perDay = Hydrogen.boilOffKgPerS(leak) * 86400 / (volume * Hydrogen.LIQUID_DENSITY_KG_PER_M3);
        assertTrue(perDay > 0.01 && perDay < 0.06, "boil-off " + perDay * 100 + " %/day at " + leak + " W");
        // the same tank in sprayed foam boils off in a day or so
        double foam = area * Insulation.FOAM.heatFlux(0.025, 293.15, Hydrogen.BOILING_K);
        assertTrue(Hydrogen.boilOffKgPerS(foam) * 86400 > 0.5 * volume * Hydrogen.LIQUID_DENSITY_KG_PER_M3, "foam is not for keeping");
    }

    @Test void aMiraiClassTankWeighsWhatToyotasDoes() {
        // Toyota Mirai (2014): two type IV tanks, 122.4 l, 70 MPa, burst at 2.25 times; 87.5 kg for the pair with fittings.
        // Wound carbon laminate of 2.0 GPa effective strength (Hua et al., Argonne ANL-10/24, 2011: about 30 mm of laminate
        // on a 0.25 m radius 700 bar tank), 1550 kg/m^3; 5 mm HDPE liner; 2 mm glass-fibre outer layer; 4 kg of boss, valve
        // and relief device each.
        double r = 0.165, burst = 2.25 * 70e6, wall = PressureVessel.compositeCylinderWallM(burst, r, 2.0e9);
        double mass = 0;
        for (double v : new double[]{0.0600, 0.0624}) {
            double length = (v - 4.0 / 3 * Math.PI * r * r * r) / (Math.PI * r * r), area = 2 * Math.PI * r * length + 4 * Math.PI * r * r;
            mass += area * (wall * 1550 + 0.005 * 950 + 0.002 * 1900) + 4;
        }
        assertEquals(87.5, mass, 0.2 * 87.5, "pair of tanks");
        assertTrue(wall > 0.015 && wall < 0.025, "laminate " + wall * 1e3 + " mm");
    }
}

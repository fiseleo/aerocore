package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class HydrogenFuelTest {
    @Test void hydrogenBurnsToWaterAtItsStoichiometricRatio() {
        Fuel h = Fuel.HYDROGEN;
        assertSame(h, Fuel.byName("hydrogen"));
        assertSame(h, Fuel.byName("liquid_hydrogen"));
        // air-fuel ratio 34.3 (Heywood, appendix D)
        assertEquals(34.3, h.stoichiometricAirFuel(), 0.005 * 34.3);
        // lean and stoichiometric: water and leftover oxygen, no carbon species
        Map<Species, Double> lean = h.productMoles(2);
        assertEquals(1, lean.get(Species.H2O), 1e-12);
        assertEquals(0.5, lean.get(Species.O2), 1e-12);
        assertEquals(0, lean.getOrDefault(Species.CO2, 0.0), 1e-15);
        // rich at lambda 0.5: half the hydrogen burns, half is left, and only half the heat comes out
        Map<Species, Double> rich = h.productMoles(0.5);
        assertEquals(0.5, rich.get(Species.H2O), 1e-12);
        assertEquals(0.5, rich.get(Species.H2), 1e-12);
        assertEquals(0, rich.getOrDefault(Species.CO, 0.0), 1e-15);
        assertEquals(0.5 * h.lowerHeatingValueJPerKg(), h.heatRelease(0.5), 0.003 * h.lowerHeatingValueJPerKg());
    }

    @Test void theBurnerHeatsAirWithHydrogenAsItsHeatingValueSays() {
        // a turbojet's burner at 600 K: the fuel for 1500 K and back; far less fuel than kerosene for the same heat
        double f = GasPath.fuelAirRatio(Fuel.HYDROGEN, 600, 1500, 0.99);
        assertEquals(1500, GasPath.burnerExit(Fuel.HYDROGEN, 600, f, 0.99), 0.5);
        double fk = GasPath.fuelAirRatio(Fuel.KEROSENE, 600, 1500, 0.99);
        assertEquals(Fuel.KEROSENE.lowerHeatingValueJPerKg() / Fuel.HYDROGEN.lowerHeatingValueJPerKg(), f / fk, 0.05);
        // the adiabatic flame at stoichiometric from 300 K, without dissociation, is above the 2390 K measured (dissociation takes about 150 K)
        double stoich = 1 / Fuel.HYDROGEN.stoichiometricAirFuel();
        double t = GasPath.burnerExit(Fuel.HYDROGEN, 300, stoich, 1);
        assertTrue(t > 2390 && t < 2700, "stoichiometric flame " + t + " K");
    }
}

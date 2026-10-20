package dev.mcmod.aerocore.api.petroleum;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Petroleum calculations (0.26) against worked examples and published ranges. */
class PetroleumTest {
    @Test void assaysAndDistillation() {
        // API 38: SG 141.5 / 169.5 = 0.8348, 834 kg/m^3; API 22: 0.9218, 921 kg/m^3
        assertEquals(834.0, CrudeAssay.LIGHT_SWEET.densityKgM3(), 0.1);
        assertEquals(920.9, CrudeAssay.HEAVY_SOUR.densityKgM3(), 0.1);
        for (CrudeAssay a : new CrudeAssay[]{CrudeAssay.LIGHT_SWEET, CrudeAssay.HEAVY_SOUR}) {
            double sum = 0;
            for (CrudeAssay.Cut c : CrudeAssay.Cut.values()) { sum += a.massYield(c); }
            assertEquals(1, sum, 1e-12);
            assertEquals(1, a.massBoiledBelow(900), 1e-12);
        }
        // a light crude gives far more light products (gas to diesel) than a heavy one
        assertTrue(CrudeAssay.LIGHT_SWEET.massBoiledBelow(350) > 0.5 && CrudeAssay.HEAVY_SOUR.massBoiledBelow(350) < 0.4);
        assertThrows(IllegalArgumentException.class, () -> new CrudeAssay("bad", 30, 1, java.util.Map.of(CrudeAssay.Cut.GAS, 0.5)));
        // Souders-Brown: rho_L 700, rho_V 3 -> 0.08 sqrt(697 / 3) = 1.219 m/s; a 1 m column at 80 % carries 2.30 kg/s of vapour
        assertEquals(1.219, Distillation.floodingVelocity(700, 3), 0.001);
        assertEquals(2.298, Distillation.capacityKgPerS(1, 700, 3), 0.005);
        assertEquals(1, Distillation.diameterM(Distillation.capacityKgPerS(1, 700, 3), 700, 3), 1e-12);
        // the heater burns 1-2 % of the crude as fuel oil (40.5 MJ/kg, an 85 % furnace)
        double share = Distillation.heaterFuelShare(CrudeAssay.LIGHT_SWEET, 40.5e6, 0.85);
        assertTrue(share > 0.01 && share < 0.02, "heater fuel " + share);
    }

    @Test void conversionsCloseTheirMassBalance() {
        assertEquals(0.50, Conversion.FCC.yield("gasoline"), 0);
        assertEquals(0.30, Conversion.NAPHTHA_CRACKER.yield("ethylene"), 0);
        assertEquals(26e6, Conversion.NAPHTHA_CRACKER.energyJPerKgFeed() / Conversion.NAPHTHA_CRACKER.yield("ethylene"), 1);
        assertTrue(Conversion.PROPANE_CRACKER.yield("ethylene") > Conversion.NAPHTHA_CRACKER.yield("ethylene"), "ethane and propane crack to more ethylene");
        assertThrows(IllegalArgumentException.class, () -> new Conversion("bad", "x", java.util.Map.of("a", 0.6), 0));
    }

    @Test void reservoirsWellsPumpsAndBits() {
        // 1 km^2 x 20 m x 0.2 x 0.75 / 1.15 = 2.61 million m^3 in place
        Reservoir r = new Reservoir(1e6, 20, 0.2, 0.75, 1.15, 100 * Reservoir.MILLIDARCY_M2, 0.002, 200e5, 20e5, 0.25);
        assertEquals(2.609e6, r.oilInPlaceM3(), 0.001e6);
        // J = 2 pi k h / (mu B (ln(300 / 0.1) - 0.75)): about 6.4 m^3/day per bar
        double j = r.productivityIndex(300, 0.1);
        assertEquals(6.42, j * 1e5 * 86400, 0.05);
        // Vogel's curve: q_max at no back pressure, 0.7 of it at half the reservoir pressure, nothing at it
        double qMax = j * 200e5 / 1.8;
        assertEquals(qMax, Reservoir.vogelRate(j, 200e5, 0), 1e-12);
        assertEquals(0.7 * qMax, Reservoir.vogelRate(j, 200e5, 100e5), 1e-12);
        assertEquals(0, Reservoir.vogelRate(j, 200e5, 200e5), 0);
        // from 1500 m the oil column is 125 bar: the new well flows, the depleted one needs a pump
        assertTrue(r.naturalRate(j, 1500, 850, 10e5, 0) > 0);
        assertEquals(0, r.naturalRate(j, 1500, 850, 10e5, 0.6 * r.recoverableM3()), 0);
        assertEquals(20e5, r.pressurePa(2 * r.recoverableM3()), 1e-6);
        // a 1.75 in plunger, 64 in stroke, 10 strokes a minute: 29 m^3/day
        double q = Lift.rodPumpM3PerS(0.04445, 1.6256, 10);
        assertEquals(29.06, q * 86400, 0.05);
        assertEquals(q * 850 * 9.81 * 1500 / 0.5, Lift.liftPowerW(q, 850, 1500, 0), 1e-9);
        // Teale: 75 kW on a 0.2 m bit in 150 MPa rock, 35 % efficient: 20 m/h
        assertEquals(20.1, Lift.penetrationMPerS(75e3, 0.2, 150e6) * 3600, 0.1);
    }
}

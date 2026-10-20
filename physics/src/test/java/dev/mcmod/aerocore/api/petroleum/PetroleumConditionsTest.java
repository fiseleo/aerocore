package dev.mcmod.aerocore.api.petroleum;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Petroleum operating conditions (0.27): the units' yields away from their nominal tables, the heater's flash, the wellhead choke. */
class PetroleumConditionsTest {
    private static double conversion(Conversion c) { return 1 - c.yield("light_cycle_oil") - c.yield("slurry"); }

    @Test void fccFollowsItsRiserTemperature() {
        assertSame(Conversion.FCC, Conversion.fcc(Conversion.NOMINAL_RISER_C));
        // the fit is the table at 530 C: a hair away from it, the same yields
        Conversion near = Conversion.fcc(530.0001);
        for (String p : Conversion.FCC.yields().keySet()) { assertEquals(Conversion.FCC.yield(p), near.yield(p), 1e-4, p); }
        double before = 0, peakAt = 0, peak = 0;
        for (double t = Conversion.RISER_MIN_C; t <= Conversion.RISER_MAX_C; t += 5) {
            Conversion c = Conversion.fcc(t);
            for (double y : c.yields().values()) { assertTrue(y >= 0, t + " C: " + c.yields()); }
            assertTrue(conversion(c) > before, "conversion rises with the riser's temperature");
            before = conversion(c);
            if (c.yield("gasoline") > peak) { peak = c.yield("gasoline"); peakAt = t; }
        }
        // gasoline peaks near 540 C (overcracking beyond), a little above the table's 50 %
        assertEquals(540, peakAt, 5);
        assertTrue(peak > 0.5 && peak < 0.51, "peak gasoline " + peak);
        // maximum distillate (500 C): more light cycle oil, less gas; maximum olefins (550 C): more LPG
        Conversion distillate = Conversion.fcc(500), olefins = Conversion.fcc(550);
        assertTrue(distillate.yield("light_cycle_oil") > 0.2 && distillate.yield("lpg") < Conversion.FCC.yield("lpg"), "500 C: " + distillate.yields());
        assertTrue(olefins.yield("lpg") > 0.19, "550 C: " + olefins.yields());
        assertTrue(conversion(distillate) > 0.6 && conversion(Conversion.fcc(560)) < 0.85, "conversion stays in an FCC's range");
        assertThrows(IllegalArgumentException.class, () -> Conversion.fcc(600));
    }

    @Test void reformingToAHigherOctaneMakesLessReformateAndMoreHydrogen() {
        assertSame(Conversion.REFORMER, Conversion.reformer(Conversion.NOMINAL_OCTANE));
        double reformate = 1, hydrogen = 0, energy = 0;
        for (double ron = Conversion.OCTANE_MIN; ron <= Conversion.OCTANE_MAX; ron += 1) {
            Conversion c = Conversion.reformer(ron);
            assertTrue(c.yield("reformate") < reformate && c.yield("hydrogen") > hydrogen && c.energyJPerKgFeed() > energy, "RON " + ron);
            reformate = c.yield("reformate"); hydrogen = c.yield("hydrogen"); energy = c.energyJPerKgFeed();
        }
        // about 88 % at RON 95, 78 % at RON 104 (a medium naphtha)
        assertEquals(0.879, Conversion.reformer(95).yield("reformate"), 0.002);
        assertEquals(0.776, Conversion.reformer(104).yield("reformate"), 0.002);
        assertThrows(IllegalArgumentException.class, () -> Conversion.reformer(110));
    }

    @Test void steamCrackingHarderMakesMoreEthyleneLessPropylene() {
        assertSame(Conversion.NAPHTHA_CRACKER, Conversion.steamCracking("naphtha", Conversion.NOMINAL_COIL_C));
        assertSame(Conversion.PROPANE_CRACKER, Conversion.steamCracking("lpg", Conversion.NOMINAL_COIL_C));
        for (String feed : new String[]{"naphtha", "lpg"}) {
            double ethylene = 0, propylene = 1;
            for (double t = Conversion.COIL_MIN_C; t <= Conversion.COIL_MAX_C; t += 5) {
                Conversion c = Conversion.steamCracking(feed, t);
                assertTrue(c.yield("ethylene") > ethylene && c.yield("propylene") < propylene, feed + " at " + t);
                ethylene = c.yield("ethylene"); propylene = c.yield("propylene");
                assertEquals(26e6, c.energyJPerKgFeed() / c.yield("ethylene"), 1e-3);
            }
        }
        // naphtha: propylene to ethylene about 0.67 at low severity, 0.44 at high
        Conversion low = Conversion.steamCracking("naphtha", 800), high = Conversion.steamCracking("naphtha", 860);
        assertEquals(0.67, low.yield("propylene") / low.yield("ethylene"), 0.01);
        assertEquals(0.44, high.yield("propylene") / high.yield("ethylene"), 0.01);
        assertThrows(IllegalArgumentException.class, () -> Conversion.steamCracking("gas_oil", 835));
    }

    @Test void aHotterFlashBurnsMoreFuel() {
        CrudeAssay crude = CrudeAssay.LIGHT_SWEET;
        assertEquals(Distillation.heaterJPerKg(crude), Distillation.heaterJPerKg(crude, Distillation.FLASH_C), 0);
        assertEquals(Distillation.heaterFuelShare(crude, 40e6, 0.85), Distillation.heaterFuelShare(crude, Distillation.FLASH_C, 40e6, 0.85), 0);
        assertTrue(Distillation.heaterFuelShare(crude, 380, 40e6, 0.85) > Distillation.heaterFuelShare(crude, 340, 40e6, 0.85));
    }

    @Test void aChokeHoldsAWellBack() {
        Reservoir r = new Reservoir(1e6, 20, 0.2, 0.75, 1.15, 100 * Reservoir.MILLIDARCY_M2, 0.002, 200e5, 20e5, 0.25);
        double j = r.productivityIndex(300, 0.1), open = r.naturalRate(j, 1500, 850, 0, 0);
        assertEquals(open, r.chokedRate(j, 1500, 850, 0, 0), 0);
        double inch64 = 0.0254 / 64, before = 0;
        for (int size : new int[]{4, 8, 16, 32, 64}) {
            double q = r.chokedRate(j, 1500, 850, size * inch64, 0);
            assertTrue(q > before && q < open, size + "/64 in: " + q * 86400 + " m^3/day of " + open * 86400);
            before = q;
        }
        // the orifice equation holds at the solution: the choke passes what the reservoir gives at that bottom-hole pressure
        double bore = 8 * inch64, q = r.chokedRate(j, 1500, 850, bore, 0);
        double dp = 850 / 2.0 * Math.pow(q / (Reservoir.CHOKE_DISCHARGE * Math.PI / 4 * bore * bore), 2);
        assertEquals(q, Reservoir.vogelRate(j, 200e5, 850 * 9.81 * 1500 + dp), 1e-6 * q);
        // a bore far wider than the tubing changes nothing; a depleted well flows through none
        assertEquals(open, r.chokedRate(j, 1500, 850, 1.0, 0), 1e-3 * open);
        assertEquals(0, r.chokedRate(j, 1500, 850, bore, 0.6 * r.recoverableM3()), 0);
    }
}

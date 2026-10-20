package dev.mcmod.aerocore.api.manufacture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Tool life, grinding and abrasive finishing (0.35): Taylor's equation and the grinding and Preston relations, worked through. */
class AbrasiveTest {
    private static final double MM = 1e-3, UM = 1e-6;

    @Test void taylorToolLifeFitsTwoTestsAndGivesTheEconomicLives() {
        // two tests: 160 m/min lasts 5 min, 100 m/min lasts 41 min -> n = ln 1.6 / ln 8.2 = 0.2234, C = 160 x 5^n = 229 m/min
        ToolLife.Taylor t = ToolLife.fit(160.0 / 60, 5 * 60, 100.0 / 60, 41 * 60);
        assertEquals(0.2234, t.n(), 1e-4);
        assertEquals(229.2, t.oneMinuteMps() * 60, 0.1);
        assertEquals(41 * 60, t.lifeS(100.0 / 60), 1e-6);
        assertEquals(100.0 / 60, t.speedFor(41 * 60), 1e-9);
        // C is the one-minute speed; halving the speed of a carbide tool on steel (n 0.25) lasts 2^4 = 16 minutes
        assertEquals(60, ToolLife.lifeS(ToolLife.C_CARBIDE_STEEL, ToolLife.C_CARBIDE_STEEL, ToolLife.N_CARBIDE), 1e-9);
        assertEquals(16 * 60, ToolLife.lifeS(ToolLife.C_CARBIDE_STEEL / 2, ToolLife.C_CARBIDE_STEEL, ToolLife.N_CARBIDE), 1e-6);
        // 20 % faster cuts a high-speed steel tool's life (n 0.125) by 1.2^8 = 4.3 times, a carbide tool's by 1.2^4 = 2.1
        double hss = ToolLife.lifeS(0.5, ToolLife.C_HSS_STEEL, ToolLife.N_HSS) / ToolLife.lifeS(0.6, ToolLife.C_HSS_STEEL, ToolLife.N_HSS);
        double carbide = ToolLife.lifeS(2.5, ToolLife.C_CARBIDE_STEEL, ToolLife.N_CARBIDE) / ToolLife.lifeS(3.0, ToolLife.C_CARBIDE_STEEL, ToolLife.N_CARBIDE);
        assertEquals(Math.pow(1.2, 8), hss, 1e-9);
        assertEquals(Math.pow(1.2, 4), carbide, 1e-9);
        assertTrue(hss > carbide);
        // economic lives: n 0.25 and a 2 min tool change -> 6 min for the most pieces; an edge worth 2.5 min of the machine -> 13.5 min for the least cost
        assertEquals(6 * 60, ToolLife.maxProductionLifeS(0.25, 120), 1e-9);
        assertEquals(13.5 * 60, ToolLife.minCostLifeS(0.25, 120, 2.5, 1.0 / 60), 1e-6);
        assertThrows(IllegalArgumentException.class, () -> ToolLife.lifeS(1, 1, 1.2));
    }

    @Test void grindingTimesRatesAndWheelWear() {
        // surface grinding 200 x 50 mm down 0.1 mm: 0.025 mm passes (4), 5 mm crossfeed (10 strokes), 25 mm overtravel, 0.25 m/s -> 40 strokes of 1 s
        assertEquals(40, Grinding.surfaceS(0.2, 0.05, 0.1 * MM, 0.25, 5 * MM, 0.025 * MM, 25 * MM), 1e-9);
        // a 50 mm shaft 200 mm long, 0.1 mm radial stock at 0.01 mm a pass, 10 mm/rev traverse, 0.5 m/s: 10 passes of pi x 0.05 x 0.2 / (0.01 x 0.5)
        assertEquals(10 * Math.PI * 0.05 * 0.2 / (0.01 * 0.5), Grinding.cylindricalS(0.2, 0.05, 0.1 * MM, 0.5, 10 * MM, 0.01 * MM), 1e-9);
        assertEquals(0.05 * MM / (5 * UM) + 3, Grinding.plungeS(0.05 * MM, 5 * UM, 3), 1e-9);
        // removal 0.25 m/s x 20 mm x 0.02 mm = 100 mm^3/s; at 40 J/mm^3 that takes 4 kW; at G 50 the wheel loses 2 mm^3 a second
        double rate = Grinding.removalRate(0.25, 20 * MM, 0.02 * MM);
        assertEquals(100e-9, rate, 1e-15);
        assertEquals(4000, Grinding.powerW(40e9, rate), 1e-6);
        assertEquals(2e-9, Grinding.wheelWearM3(rate, 50), 1e-18);
        // centreless: a 200 mm regulating wheel at 30 rev/min tilted 3 degrees drives the work at pi x 0.2 x 0.5 x sin 3 = 16.4 mm/s
        double f = Grinding.throughfeedMps(0.2, 0.5, Math.toRadians(3));
        assertEquals(16.44, f / MM, 0.01);
        // 100 pins 50 mm long past a 200 mm wheel
        assertEquals((100 * 0.05 + 0.2) / f, Grinding.centrelessS(0.05, 100, 0.2, f), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> Grinding.throughfeedMps(0.2, 0.5, 0));
    }

    @Test void lappingPolishingAndHoningFollowPrestonAndTheCrosshatch() {
        // K 1e-13 /Pa, 0.1 MPa, 1 m/s -> 10 nm/s; 2 micrometres take 200 s; twice the pressure halves it
        assertEquals(1e-8, Finishing.prestonMps(1e-13, 1e5, 1), 1e-20);
        assertEquals(200, Finishing.prestonS(2 * UM, 1e-13, 1e5, 1), 1e-9);
        assertEquals(100, Finishing.prestonS(2 * UM, 1e-13, 2e5, 1), 1e-9);
        // honing 50 micrometres at 1 micrometre a second; stroking at tan 15 degrees of the rotation leaves a 30 degree crosshatch
        assertEquals(50, Finishing.honingS(50 * UM, 1 * UM), 1e-9);
        double stroke = Finishing.strokingMps(Math.toRadians(30), 1.0);
        assertEquals(Math.tan(Math.toRadians(15)), stroke, 1e-12);
        assertEquals(30, Math.toDegrees(Finishing.crosshatchRad(stroke, 1.0)), 1e-9);
    }
}

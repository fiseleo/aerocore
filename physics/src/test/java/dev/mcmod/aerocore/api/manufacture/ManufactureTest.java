package dev.mcmod.aerocore.api.manufacture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Manufacturing calculations (0.25) against the standards' tables and the textbooks' worked examples. */
class ManufactureTest {
    private static final double MM = 1e-3, UM = 1e-6;

    @Test void isoTolerancesMatchTheTable() {
        // ISO 286-1 table 1 (rounded there): IT5 3-6 mm 5, IT6 50-80 19, IT7 18-30 21, IT8 120-180 63, IT9 30-50 62, IT11 6-10 90 micrometres
        assertEquals(5, Tolerance.toleranceM(5 * MM, 5) / UM, 0.5);
        assertEquals(19, Tolerance.toleranceM(60 * MM, 6) / UM, 0.5);
        assertEquals(21, Tolerance.toleranceM(25 * MM, 7) / UM, 0.5);
        assertEquals(63, Tolerance.toleranceM(150 * MM, 8) / UM, 0.5);
        assertEquals(62, Tolerance.toleranceM(40 * MM, 9) / UM, 0.5);
        assertEquals(90, Tolerance.toleranceM(8 * MM, 11) / UM, 0.5);
        // above 500 mm: IT7 800-1000 mm is 90 micrometres
        assertEquals(90, Tolerance.toleranceM(900 * MM, 7) / UM, 1.5);
        // IT1 by its formula is 1.27 micrometres at 18-30 mm (the table rounds it up to 1.5); the finer grades shrink monotonically
        assertEquals(0.8 + 0.020 * Math.sqrt(18 * 30), Tolerance.toleranceM(25 * MM, 1) / UM, 1e-9);
        for (int g = 2; g <= 18; g++) { assertTrue(Tolerance.toleranceM(25 * MM, g) > Tolerance.toleranceM(25 * MM, g - 1)); }
        // a 0.02 mm tolerance on an 86 mm bore is IT6 (22 micrometres is IT6 at 80-120 mm)
        assertEquals(6, Tolerance.grade(86 * MM, 0.022 * MM));
        assertEquals(5, Tolerance.grade(86 * MM, 0.021 * MM));
        assertThrows(IllegalArgumentException.class, () -> Tolerance.toleranceM(0.01, 19));
    }

    @Test void turningAndDrillingTimesFollowGrooversExamples() {
        // Groover example 22.1: 200 mm x 700 mm turned at 2.30 m/s, 0.32 mm/rev: 598 s
        assertEquals(598, Cutting.turnS(0.7, 0.2, 2.3, 0.32 * MM), 2);
        // Groover example 22.2: a 12.7 mm drill, 60 mm blind hole, 118 degree point, 25 m/min, 0.30 mm/rev: 0.339 min
        assertEquals(0.339 * 60, Cutting.drillS(1, 12.7 * MM, 60 * MM, 25.0 / 60, 0.30 * MM, Math.toRadians(118)), 0.3);
        // ideal roughness: 0.2 mm/rev with a 0.8 mm nose radius gives Ra 1.56 micrometres, and back
        assertEquals(1.5625, Cutting.roughness(0.2 * MM, 0.8 * MM) / UM, 1e-6);
        assertEquals(0.2 * MM, Cutting.feedFor(1.5625 * UM, 0.8 * MM), 1e-12);
        // roughing 1 litre of steel (u 2.7 GJ/m^3) with 10 kW at 90 %: 300 s; a rigidity limit slows it
        assertEquals(300, Cutting.roughS(1e-3, 10e3, 0.9, 2.7e9, 0), 1e-9);
        assertEquals(1000, Cutting.roughS(1e-3, 10e3, 0.9, 2.7e9, 1e-6), 1e-9);
    }

    @Test void chvorinovFollowsGrooversExample() {
        // Groover example 10.2: a 50 mm cube solidifies in 155 s; a 30 x 50 mm cylinder of the same alloy and mould: 74.3 s
        double cube = 0.05, constant = Casting.mouldConstant(cube * cube * cube, 6 * cube * cube, 155);
        assertEquals(2.232, constant * 1e-6, 0.001);
        double r = 0.015, l = 0.05, v = Math.PI * r * r * l, a = 2 * Math.PI * r * r + 2 * Math.PI * r * l;
        assertEquals(74.3, Casting.solidificationS(v, a, constant), 0.3);
        // melting 1 kg of aluminium from 20 C (c 900 J/kg K to 660 C, L 397 kJ/kg) and superheating 50 K (c 1100): about 1.03 MJ
        assertEquals(0.9 * 640 + 397 + 1.1 * 50, Casting.meltJ(1, 900, 293.15, 933.15, 397e3, 1100, 983.15) / 1e3, 1e-6);
    }

    @Test void formingLoadsFollowGrooversExamples() {
        // Groover example 19.2: a 50 mm x 50 mm cylinder upset to 25 mm, K 350 MPa, n 0.17, mu 0.2: 1.58 MN at the end
        double v = Math.PI * 0.025 * 0.025 * 0.05;
        assertEquals(1.582e6, Forming.upsetForceN(v, 0.05, 0.025, 0.2, 350e6, 0.17), 0.01 * 1.582e6);
        // Groover example 19.1: 300 x 25 mm strip to 22 mm, R 250 mm, 50 rev/min, K 275 MPa, n 0.15: 1.44 MN, 207 kW
        double f = Forming.rollForceN(0.3, 0.025, 0.022, 0.25, 275e6, 0.15), contact = Forming.rollContactM(0.25, 0.003);
        assertEquals(27.4e-3, contact, 0.1e-3);
        assertEquals(1.445e6, f, 0.01 * 1.445e6);
        assertEquals(207e3, Forming.rollPowerW(50.0 / 60, f, contact), 0.01 * 207e3);
        // heating a 100 mm steel bar through (k 45, rho 7850, c 470): about 4 minutes at Fo 1
        assertEquals(0.05 * 0.05 / (45 / (7850 * 470.0)), Forming.soakS(0.05, 45, 7850, 470, 1), 1e-9);
    }

    @Test void weldsRivetsAndLaminates() {
        // a 6 mm fillet holds 18 mm^2: 0.141 kg of steel a metre; at 3 kg/h and an operator factor of 0.3, 9.4 min a metre
        double metal = Joining.metalKg(Joining.filletAreaM2(6 * MM), 1, 7850);
        assertEquals(0.1413, metal, 1e-4);
        assertEquals(metal / (3.0 / 3600) / 0.3, Joining.weldS(metal, 3.0 / 3600, 0.3), 1e-9);
        // a 10 mm plate, 60 degree V, 2 mm gap: 77.7 mm^2
        assertEquals(100 * Math.tan(Math.toRadians(30)) + 20, Joining.buttAreaM2(10 * MM, Math.toRadians(60), 2 * MM) / (MM * MM), 1e-9);
        // a 2.5 m seam, rivets at 25 mm in two rows: 200
        assertEquals(200, Joining.fasteners(2.5, 0.025, 2));
        assertEquals(0, Joining.fasteners(0, 0.025, 2));
        // 4 m^2 of 6 plies at 1.5 m^2/h: 16 h; a 15 min tyre doubled in thickness: an hour
        assertEquals(16 * 3600, Laminating.layupS(4, 6, 1.5 / 3600), 1e-6);
        assertEquals(3600, Laminating.vulcanizeS(0.04, 900, 0.02), 1e-9);
    }
}

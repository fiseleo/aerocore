package dev.mcmod.aerocore.api.drive;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Gearing, clutch and converter formulas (0.24) against published numbers. */
class DriveTest {
    @Test void theConverterKeepsMathWorksCurveAndStretchesToADesign() {
        TorqueConverter table = new TorqueConverter(2.232, 12.29);
        for (int i = 0; i < TorqueConverter.NU.length; i++) {
            assertEquals(TorqueConverter.RATIO[i], table.torqueRatio(TorqueConverter.NU[i]), 1e-9, "torque ratio at " + TorqueConverter.NU[i]);
            assertEquals(TorqueConverter.K[i], table.capacity(TorqueConverter.NU[i]), 1e-9, "K at " + TorqueConverter.NU[i]);
        }
        // stall: 200 N m from an engine holds the pump at K sqrt(T) = 174 rad/s (1660 rpm), the turbine pushed 2.232 times as hard
        double[] stall = table.torques(12.29 * Math.sqrt(200), 0);
        assertEquals(-200, stall[0], 1e-6);
        assertEquals(2.232 * 200, stall[1], 1e-6);
        // the coupling point (torque ratio 1) near 0.94 (GM: about 0.9; Naunheimer's examples 0.88), and in converter mode an
        // efficiency nu x mu of 0.94 there, rising toward the coupling's nu above it
        double coupling = 0;
        for (double nu = 0; nu < 0.97; nu += 0.001) { if (table.torqueRatio(nu) <= 1) { coupling = nu; break; } }
        assertEquals(0.935, coupling, 0.01);
        assertEquals(0.94, coupling * table.torqueRatio(coupling), 0.01);
        // a design: stall ratio 2.0 at stall, the coupling end unchanged; capacity scaled to the diameter (D^-2.5)
        TorqueConverter design = new TorqueConverter(2.0, TorqueConverter.stallK(0.28));
        assertEquals(2.0, design.torqueRatio(0), 1e-12);
        assertEquals(0.9983, design.torqueRatio(0.95), 1e-9);
        assertEquals(12.29 * Math.pow(0.26 / 0.28, 2.5), design.stallK(), 1e-12);
        // coasting: the turbine faster drives the pump, no multiplication
        double[] coast = table.torques(100, 150);
        assertTrue(coast[0] > 0 && coast[1] < 0, "engine braking through the converter");
        assertEquals(-coast[0], coast[1], 1e-12);
    }

    @Test void planetaryRatiosReproducePublishedBoxes() {
        // GM TH400 (Simpson): sun 39, both rings 81 -> 2.481 / 1.481 / 1 / -2.077
        assertArrayEquals(new double[]{2.481, 1.481, 1, -2.077}, Driveline.simpson(39, 81, 81), 0.0005);
        // ZF 6HP26 (Lepelletier): S1/R1 37/71, S2 31, S3 38, R3 85 -> 4.171 2.340 1.521 1.143 0.867 0.691, R -3.403
        assertArrayEquals(new double[]{4.171, 2.340, 1.521, 1.143, 0.867, 0.691, -3.403}, Driveline.lepelletier(37, 71, 31, 38, 85), 0.0005);
        // Ford AOD (Ravigneaux): R/S_f 2.40, R/S_r 2.00 -> 2.40 / 1.47 / 1 / 0.67, R -2.00
        assertArrayEquals(new double[]{2.40, 1.47, 1, 0.67, -2.00}, Driveline.ravigneaux(30, 36, 72), 0.005);
    }

    @Test void clutchesSynchronisersBeltsAndDifferentials() {
        // LuK's clutch course: d 134 / 190 mm, 3500 N, mu 0.27, one plate (two faces): 153 N m
        assertEquals(153, Driveline.clutchCapacity(2, 0.27, 3500, 0.134, 0.190), 1);
        // a single cone at 7 degrees: friction 0.1, 500 N, 70 mm: j mu F d / (2 sin a)
        assertEquals(0.1 * 500 * 0.07 / (2 * Math.sin(Math.toRadians(7))), Driveline.synchroniserTorque(1, 0.1, 500, 0.07, Math.toRadians(7)), 1e-9);
        // CK2 push belt in overdrive: 10 kN clamp, mu 0.1, 66 mm radius, 11 degrees
        assertEquals(2 * 0.1 * 10000 * 0.066 / Math.cos(Math.toRadians(11)), Driveline.beltCapacity(0.1, 10000, 0.066, Math.toRadians(11)), 1e-9);
        // ramp LSD: lock factor 0.5 is a 3:1 bias, 0.25 is 1.67, 0.6 is 4
        assertEquals(3, Driveline.biasRatio(0.5), 1e-12);
        assertEquals(1.67, Driveline.biasRatio(0.25), 0.005);
        assertEquals(4, Driveline.biasRatio(0.6), 1e-12);
        // 45 degree ramps, mu 0.1, 8 faces, friction radius 0.04, ramp radius 0.04: L = 0.8 (cot 45 = 1)
        assertEquals(0.8, Driveline.lockFactor(0.1, 8, 0.04, 0.04, Math.toRadians(45)), 1e-12);
        assertEquals(Math.PI * 10 * 50 * (Math.pow(0.08, 4) - Math.pow(0.04, 4)) / (2 * 3e-4), Driveline.viscousCoefficient(10, 50, 0.04, 0.08, 3e-4), 1e-9);
    }
}

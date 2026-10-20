package dev.mcmod.aerocore.api.manufacture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Gear cutting, broaching, sawing, threading and riveting (0.35): the kinematic times and loads, worked through. */
class OperationsTest {
    private static final double MM = 1e-3, UM = 1e-6;

    @Test void gearCuttingTimesFollowTheKinematics() {
        // module 3 (tooth depth 2.25 m = 6.75 mm) with an 80 mm hob: it starts sqrt(6.75 x 73.25) = 22.2 mm before the face
        double a = GearCutting.approachM(80 * MM, 6.75 * MM);
        assertEquals(22.24, a / MM, 0.01);
        // 90 m/min on an 80 mm hob: 5.97 rev/s; a 40-tooth gear 30 mm wide, 5 mm overrun, single start, 2 mm a revolution: about 3.2 min
        double n = GearCutting.hobRevPerS(1.5, 80 * MM);
        assertEquals(5.968, n, 0.001);
        double t = GearCutting.hobbingS(40, 30 * MM, a, 5 * MM, 1, n, 2 * MM);
        assertEquals(40 * (0.03 + a + 0.005) / (n * 0.002), t, 1e-9);
        assertEquals(191.8, t, 0.2);
        // a two-start hob takes half the time
        assertEquals(t / 2, GearCutting.hobbingS(40, 30 * MM, a, 5 * MM, 2, n, 2 * MM), 1e-9);
        // shaping a 120 mm pitch circle at 0.3 mm a stroke, 10 strokes a second, two cuts: 251 s
        assertEquals(2 * Math.PI * 0.12 / 0.003, GearCutting.shapingS(0.12, 0.3 * MM, 10, 2), 1e-9);
        // shaving 30 mm at 0.2 mm a revolution, 2 rev/s, 4 passes: 300 s; profile grinding 40 gaps, two strokes of 40 mm at 0.1 m/s: 32 s
        assertEquals(300, GearCutting.shavingS(30 * MM, 0.2 * MM, 2, 4), 1e-9);
        assertEquals(32, GearCutting.profileGrindingS(40, 30 * MM, 5 * MM, 0.1, 2), 1e-9);
        assertThrows(IllegalArgumentException.class, () -> GearCutting.approachM(5 * MM, 6 * MM));
    }

    @Test void broachingTeethForceAndStroke() {
        // 3 mm deep at 0.075 mm a tooth: 40 cutting teeth; a 50 mm work at a 12 mm pitch has 5 in it
        assertEquals(40, Broaching.cuttingTeeth(3 * MM, 0.075 * MM));
        assertEquals(5, Broaching.engagedTeeth(50 * MM, 12 * MM));
        assertEquals(1, Broaching.engagedTeeth(5 * MM, 12 * MM));
        // steel (3 J/mm^3), 20 mm wide: 3e9 x 0.02 x 0.075e-3 x 5 = 22.5 kN
        assertEquals(22.5e3, Broaching.forceN(3e9, 20 * MM, 0.075 * MM, 5), 1e-6);
        // a 540 mm broach past 50 mm at 6 m/min, back at 30 m/min
        assertEquals(0.59 / 0.1 + 0.59 / 0.5, Broaching.strokeS(0.54, 0.05, 0.1, 0.5), 1e-9);
    }

    @Test void sawingFeedsPowerAndKerf() {
        // 2 micrometres a tooth, 60 m/min, 6.35 mm pitch: 0.315 mm/s
        double f = Sawing.feedMps(2 * UM, 1.0, 6.35 * MM);
        assertEquals(0.315, f / MM, 0.001);
        assertEquals(0.1 / f, Sawing.cutS(0.1, f), 1e-9);
        // a 2 kW saw at 80 % in steel (2.7 J/mm^3) with a 1.3 mm kerf across 100 mm: 4.56 mm/s at most
        assertEquals(1600 / (2.7e9 * 1.3e-3 * 0.1), Sawing.powerLimitedFeedMps(2e3, 0.8, 2.7e9, 1.3 * MM, 0.1), 1e-12);
        // a hack saw at 0.1 mm a stroke, a stroke a second, through a 50 mm bar: 500 s
        assertEquals(500, Sawing.cutS(0.05, Sawing.hacksawFeedMps(0.1 * MM, 1)), 1e-9);
        // a 100 mm steel bar loses pi/4 x 0.01 x 0.0013 x 7850 = 80 g to the kerf
        assertEquals(0.0801, Sawing.kerfKg(Math.PI / 4 * 0.01, 1.3 * MM, 7850), 1e-4);
    }

    @Test void threadingTimes() {
        // tapping M10 x 1.5 20 mm deep at 5 rev/s, out at twice the speed: 2.67 s in, 1.33 out
        assertEquals(4.0, Threading.tappingS(20 * MM, 1.5 * MM, 5, 2), 1e-9);
        // single-point: 50 mm of 2 mm pitch at 4 rev/s, 6 passes: 37.5 s
        assertEquals(37.5, Threading.singlePointS(50 * MM, 2 * MM, 4, 6), 1e-9);
        // thread milling M20 x 2.5 inside with a 16 mm cutter: the centre circles 4 mm, 8 turns, the centre moves at a fifth of the edge feed
        double t = Threading.threadMillingS(20 * MM, 16 * MM, 20 * MM, 2.5 * MM, 5 * MM, true);
        double path = 8 * Math.hypot(Math.PI * 4 * MM, 2.5 * MM);
        assertEquals(path / (5 * MM * 4 / 20), t, 1e-9);
        // outside a 20 mm shaft the centre circles 36 mm and moves faster than the edge
        assertTrue(Threading.threadMillingS(20 * MM, 16 * MM, 20 * MM, 2.5 * MM, 5 * MM, false) < t);
        assertThrows(IllegalArgumentException.class, () -> Threading.threadMillingS(10 * MM, 12 * MM, 20 * MM, 1.5 * MM, 5 * MM, true));
        // rolling: 15 work revolutions at 5 rev/s; flat dies 200 mm at 0.5 m/s and back
        assertEquals(3, Threading.inFeedRollingS(15, 5), 1e-9);
        assertEquals(0.8, Threading.flatDieS(0.2, 0.5), 1e-9);
    }

    @Test void rivetsLengthAndSqueeze() {
        // a 4 mm rivet through a 3 mm grip is 9 mm long
        assertEquals(9 * MM, Riveting.lengthM(3 * MM, 4 * MM), 1e-12);
        // its 6 mm protrusion upset into a 6 mm head is 2/3 d = 2.67 mm high; aluminium (K 200 MPa, n 0.2), mu 0.1: about 6 kN
        double d = 4 * MM, v = Math.PI / 4 * d * d * 1.5 * d;
        double f = Riveting.squeezeForceN(d, 200e6, 0.2, 0.1);
        assertEquals(Forming.upsetForceN(v, 1.5 * d, 2 * d / 3, 0.1, 200e6, 0.2), f, 1e-6);
        assertEquals(5.9e3, f, 0.3e3);
        // a stronger rivet takes more
        assertTrue(Riveting.squeezeForceN(d, 550e6, 0.2, 0.1) > f);
    }
}

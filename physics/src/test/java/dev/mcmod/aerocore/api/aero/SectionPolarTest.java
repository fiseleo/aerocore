package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.AeroCoefficients;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Sections from their shape against the measurements behind Abbott and von Doenhoff's Theory of Wing Sections: NACA
 * Report 824 (2412, 4412; smooth, LTPT), Loftin and Smith's NACA TN 1945 (0012), Ladson's NASA TM 4074 (0012, free
 * transition), and Sheldahl and Klimas' SAND80-2114 table (0012 at low Reynolds numbers). Values read from the reports'
 * charts and tables by the 0.22 research pass (docs/WING.md).
 */
class SectionPolarTest {
    private static final Airfoil N0012 = Airfoil.naca4(0, 0, 0.12), N2412 = Airfoil.naca4(0.02, 0.4, 0.12), N4412 = Airfoil.naca4(0.04, 0.4, 0.12);

    @Test void nacaShapesHaveTheirThicknessCamberAndAClosedTrailingEdge() {
        assertEquals(0.12, N0012.thickness(), 0.002);
        assertEquals(0.30, N0012.thicknessPosition(), 0.02);
        assertEquals(0.04, N4412.camber(), 0.002);
        assertEquals(0, N4412.upperAt(1) - N4412.lowerAt(1), 1e-9);
        // a CST fit reproduces the section it was fitted to
        double[][] w = Airfoil.cstFit(N2412, 6);
        Airfoil fit = Airfoil.cst(w[0], w[1], 0);
        for (double x : new double[]{0.05, 0.3, 0.6, 0.9}) {
            assertEquals(N2412.upperAt(x), fit.upperAt(x), 1.5e-3, "upper at " + x);
            assertEquals(N2412.lowerAt(x), fit.lowerAt(x), 1.5e-3, "lower at " + x);
        }
        // and Selig points read back
        List<double[]> points = new ArrayList<>();
        for (int i = N2412.x().length - 1; i >= 0; i--) { points.add(new double[]{N2412.x()[i], N2412.upper()[i]}); }
        for (int i = 1; i < N2412.x().length; i++) { points.add(new double[]{N2412.x()[i], N2412.lower()[i]}); }
        Airfoil back = Airfoil.of(points);
        assertEquals(N2412.thickness(), back.thickness(), 1e-3);
        assertEquals(N2412, Airfoil.naca4(0.02, 0.4, 0.12), "shapes compare by value");
    }

    @Test void liftSlopeZeroLiftAngleAndMomentFollowTheData() {
        // Report 824, TN 1945, TM 4074: lift slopes 0.105-0.11 per degree; zero-lift angles 0, -2.0, -4.0 deg;
        // quarter-chord moments 0, -0.045 to -0.05, -0.09 to -0.095.
        double re = 6e6;
        SectionPolar a = SectionPolar.of(N0012), b = SectionPolar.of(N2412), c = SectionPolar.of(N4412);
        for (SectionPolar p : new SectionPolar[]{a, b, c}) { assertEquals(0.106, Math.toRadians(p.liftSlope(re)), 0.006, "slope per degree"); }
        assertEquals(0, Math.toDegrees(a.zeroLiftAngle()), 0.05);
        assertEquals(-2.0, Math.toDegrees(b.zeroLiftAngle()), 0.3);
        assertEquals(-4.0, Math.toDegrees(c.zeroLiftAngle()), 0.4);
        assertEquals(0, a.sample(Math.toRadians(4), re, 0).pitchingMoment(), 0.01);
        assertEquals(-0.047, b.sample(Math.toRadians(2), re, 0).pitchingMoment(), 0.012);
        assertEquals(-0.093, c.sample(Math.toRadians(2), re, 0).pitchingMoment(), 0.015);
        // least drag 0.0057 (0012), 0.0060 (2412), 0.0062 (4412)
        double[] cd = {0.0057, 0.0060, 0.0062};
        SectionPolar[] all = {a, b, c};
        for (int i = 0; i < 3; i++) { assertEquals(cd[i], all[i].minimumDrag(re), 0.0012); }
    }

    @Test void maximumLiftIsWithinFifteenPercentOfTheData() {
        // {Reynolds number, maximum lift, section}: TN 1945 and TM 4074 (0012), Report 824 (2412, 4412)
        double[][] cases = {{6e6, 1.60, 0}, {3.94e6, 1.48, 0}, {5.7e6, 1.66, 1}, {3.1e6, 1.58, 1}, {6e6, 1.64, 2}, {3e6, 1.53, 2}};
        Airfoil[] shapes = {N0012, N2412, N4412};
        for (double[] k : cases) {
            SectionPolar p = SectionPolar.of(shapes[(int) k[2]]);
            assertEquals(k[1], p.maxLift(k[0]), 0.15 * k[1], shapes[(int) k[2]] + " clmax at " + k[0]);
            double stall = Math.toDegrees(p.stallAngle(k[0]));
            assertTrue(stall > 12 && stall < 19, "stall angle " + stall);
        }
        // Sheldahl and Klimas' table, NACA 0012: 0.98 at 360 000, 1.12 at a million, 1.22 at two million
        SectionPolar p = SectionPolar.of(N0012);
        assertEquals(0.98, p.maxLift(3.6e5), 0.1);
        assertEquals(1.12, p.maxLift(1e6), 0.1);
        assertEquals(1.22, p.maxLift(2e6), 0.1);
        assertTrue(p.maxLift(1e5) < p.maxLift(1e6) && p.maxLift(1e6) < p.maxLift(1e7));
        assertTrue(p.minimumDrag(1e5) > 1.5 * p.minimumDrag(6e6), "thicker layers at low Reynolds numbers");
    }

    @Test void symmetricSectionsAreOddInLiftAndEvenInDragAtAnyAngle() {
        SectionPolar p = SectionPolar.of(N0012);
        for (double d = -180; d <= 180; d += 7.5) {
            AeroCoefficients a = p.sample(Math.toRadians(d), 1e6, 0.2), b = p.sample(Math.toRadians(-d), 1e6, 0.2);
            assertEquals(a.lift(), -b.lift(), 1e-6, "lift at " + d);
            assertEquals(a.drag(), b.drag(), 1e-6, "drag at " + d);
        }
        // a flat plate across the flow: drag about 2, no lift
        assertEquals(2.0, p.sample(Math.toRadians(90), 1e6, 0).drag(), 0.05);
        assertEquals(0, p.sample(Math.toRadians(90), 1e6, 0).lift(), 0.02);
    }

    @Test void neverThrowsAndStaysFiniteEverywhere() {
        SectionPolar p = SectionPolar.of(N4412);
        for (double d = -200; d <= 200; d += 3.7) {
            for (double re : new double[]{0, 10, 1e3, 1e5, 1e7, 1e10, Double.NaN}) {
                for (double m : new double[]{0, 0.4, 0.8, 0.95, 1.1, 3, Double.NaN}) {
                    AeroCoefficients c = p.sample(Math.toRadians(d), re, m);
                    assertTrue(Double.isFinite(c.lift() + c.drag() + c.pitchingMoment()), d + " " + re + " " + m);
                }
            }
        }
    }

    @Test void compressibilityRaisesTheSlopeLowersTheMaximumAndAddsWaveDrag() {
        SectionPolar p = SectionPolar.of(N0012);
        double re = 4e6;
        double low = p.sample(Math.toRadians(2), re, 0).lift(), m5 = p.sample(Math.toRadians(2), re, 0.5).lift();
        assertEquals(1 / Math.sqrt(1 - 0.25), m5 / low, 0.02, "Prandtl-Glauert");
        // Beddoes' NACA 0012 stall: lift at stall about 1.3 at Mach 0.4 falling to about 0.6 at Mach 0.8 (Leishman, table 9.1)
        double c3 = max(p, re, 0.3), c5 = max(p, re, 0.5), c8 = max(p, re, 0.8);
        assertTrue(c3 > c5 && c5 > c8, c3 + " " + c5 + " " + c8);
        assertEquals(0.65, c8, 0.2);
        double mcr = p.criticalMachAt(Math.toRadians(2));
        assertTrue(mcr > 0.55 && mcr < 0.75, "critical Mach " + mcr);
        assertTrue(p.sample(Math.toRadians(2), re, mcr + 0.15).drag() > p.sample(Math.toRadians(2), re, mcr - 0.1).drag() + 0.008, "drag rise (Lock: 0.01 at 0.15 past the critical Mach)");
    }

    /** The first peak of the lift curve (a flat plate's lift past the stall can be larger). */
    private static double max(SectionPolar p, double re, double m) {
        double best = -1;
        for (double d = 0; d <= 25; d += 0.25) {
            double cl = p.sample(Math.toRadians(d), re, m).lift();
            if (cl < 0.97 * best) { break; }
            best = Math.max(best, cl);
        }
        return best;
    }

    @Test void flapsShiftTheCurveAndRaiseTheMaximumBySlotAndExtension() {
        SectionPolar p = SectionPolar.of(N2412);
        double re = 3e6, clean = p.sample(0, re, 0).lift();
        // Glauert: tau(0.25) = 0.609
        assertEquals(0.609, SectionPolar.flapEffectiveness(0.25), 0.002);
        double plain = p.flapped(0, re, 0, SectionPolar.FlapType.PLAIN, 0.25, Math.toRadians(10)).lift();
        assertEquals(p.liftSlope(re) * 0.609 * Math.toRadians(10), plain - clean, 0.05, "small deflection: thin-airfoil increment");
        double[] maxima = new double[SectionPolar.FlapType.values().length];
        for (SectionPolar.FlapType t : SectionPolar.FlapType.values()) {
            double best = -1;
            for (double d = -10; d <= 25; d += 0.5) { best = Math.max(best, p.flapped(Math.toRadians(d), re, 0, t, 0.25, Math.toRadians(40)).lift()); }
            maxima[t.ordinal()] = best;
            assertTrue(best > p.maxLift(re) + 0.4, t + " raises the maximum: " + best);
            assertTrue(p.flapped(0, re, 0, t, 0.25, Math.toRadians(40)).pitchingMoment() < p.sample(0, re, 0).pitchingMoment() - 0.1, "nose-down moment");
        }
        assertTrue(maxima[SectionPolar.FlapType.FOWLER.ordinal()] > maxima[SectionPolar.FlapType.SLOTTED.ordinal()]);
        assertTrue(maxima[SectionPolar.FlapType.SLOTTED.ordinal()] > maxima[SectionPolar.FlapType.PLAIN.ordinal()]);
        assertTrue(p.flapped(0, re, 0, SectionPolar.FlapType.SPLIT, 0.25, Math.toRadians(40)).drag()
                > p.flapped(0, re, 0, SectionPolar.FlapType.PLAIN, 0.25, Math.toRadians(40)).drag(), "split flaps drag more");
    }
}

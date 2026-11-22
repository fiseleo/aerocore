package dev.mcmod.aerocore.api.lifting;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.frame.FrameMaterial;
import dev.mcmod.aerocore.api.frame.Section;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** The lifting calculations against makers' data, the standards' tables, hand moment balances and a published load chart. */
class LiftingTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;
    private static final WireRope.Construction IWRC = WireRope.Construction.SIX_STRAND_IWRC;
    private static final WireRope.Construction RR = WireRope.Construction.ROTATION_RESISTANT_35X7;
    private static final FrameMaterial S960 = new FrameMaterial(7850, 210e9, 81e9, 960e6);

    @Test void ropeStrengthAndMassMatchTheCatalogues() {
        // Pfeifer 6x36WS-WC (EN 12385-4), 20 mm: 252 / 279 / 308 kN in 1770 / 1960 / 2160, 164 kg per 100 m
        assertEquals(252e3, WireRope.minBreakingForceN(IWRC, 0.020, WireRope.GRADE_1770_PA), 0.005 * 252e3);
        assertEquals(279e3, WireRope.minBreakingForceN(IWRC, 0.020, WireRope.GRADE_1960_PA), 0.005 * 279e3);
        assertEquals(308e3, WireRope.minBreakingForceN(IWRC, 0.020, WireRope.GRADE_2160_PA), 0.005 * 308e3);
        assertEquals(1.64, WireRope.kgPerM(IWRC, 0.020), 0.01 * 1.64);
        // the same sheet at 40 mm: 1120 kN in 1960, 6.54 kg/m
        assertEquals(1120e3, WireRope.minBreakingForceN(IWRC, 0.040, WireRope.GRADE_1960_PA), 0.005 * 1120e3);
        assertEquals(6.54, WireRope.kgPerM(IWRC, 0.040), 0.01 * 6.54);
        // Katradis 35(W)x7, 1960: 282 kN and 1.82 kg/m at 20 mm, 70.6 kN at 10 mm
        assertEquals(282e3, WireRope.minBreakingForceN(RR, 0.020, WireRope.GRADE_1960_PA), 0.005 * 282e3);
        assertEquals(70.6e3, WireRope.minBreakingForceN(RR, 0.010, WireRope.GRADE_1960_PA), 0.005 * 70.6e3);
        assertEquals(1.82, WireRope.kgPerM(RR, 0.020), 0.01 * 1.82);
        // metallic area and the spinning loss: the wires' summed strength is a quarter more than the rope's
        assertEquals(0.586 * Math.PI / 4 * 4e-4, WireRope.metalAreaM2(IWRC, 0.020), 1e-15);
        double k = WireRope.spinningLossFactor(IWRC);
        assertTrue(k > 0.72 && k < 0.82, "spinning loss " + k);
        assertEquals(WireRope.minBreakingForceN(IWRC, 0.02, 1960e6), k * WireRope.metalAreaM2(IWRC, 0.02) * 1960e6, 1e-6);
        assertThrows(IllegalArgumentException.class, () -> WireRope.minBreakingForceN(IWRC, Double.NaN, 1960e6));
        assertThrows(IllegalArgumentException.class, () -> WireRope.minBreakingForceN(IWRC, 0.02, -1));
    }

    @Test void theWorkingLoadAndTheRequiredDiameterFollowIso4308() {
        // WLL = MBF / Zp
        double mbf = WireRope.minBreakingForceN(IWRC, 0.020, WireRope.GRADE_1960_PA);
        double wll = WireRope.workingLoadLimitN(IWRC, 0.020, WireRope.GRADE_1960_PA, WireRope.Group.M5);
        assertEquals(mbf / 4.5, wll, 1e-9);
        assertEquals(mbf / 9.0, WireRope.workingLoadLimitN(mbf, WireRope.Group.M8.zp()), 1e-9);
        // round trip: the diameter for that tension is the rope's
        assertEquals(0.020, WireRope.requiredDiameterM(IWRC, WireRope.GRADE_1960_PA, wll, WireRope.Group.M5), 1e-12);
        // ISO 4308-1 Table 1: the selection factor C (mm per sqrt(N)) for 6x36WS-IWRC, R0 1770, K' 0.356
        double[] c = {0.071, 0.073, 0.075, 0.080, 0.085, 0.094, 0.106, 0.120};
        WireRope.Group[] groups = WireRope.Group.values();
        for (int i = 0; i < groups.length; i++) {
            double s = 50e3;
            double selection = 1000 * WireRope.requiredDiameterM(IWRC, WireRope.GRADE_1770_PA, s, groups[i]) / Math.sqrt(s);
            assertEquals(c[i], selection, 0.0006, groups[i].name());
        }
        // a heavier duty asks for more: Zp, drums and sheaves all grow from M1 to M8; stationary ropes ask less
        for (int i = 1; i < groups.length; i++) {
            assertTrue(groups[i].zp() > groups[i - 1].zp());
            assertTrue(groups[i].drumRatio() > groups[i - 1].drumRatio());
            assertTrue(groups[i].sheaveRatio() > groups[i - 1].sheaveRatio());
            assertTrue(groups[i].stationaryZp() >= groups[i - 1].stationaryZp());
        }
        for (WireRope.Group g : groups) { assertTrue(g.stationaryZp() < g.zp()); }
        // ISO 4308-1 Table 2: M5 with a 20 mm rope, drum 18 d and sheave 20 d
        assertEquals(0.360, WireRope.minDrumDiameterM(WireRope.Group.M5, IWRC, 0.020), 1e-12);
        assertEquals(0.400, WireRope.minSheaveDiameterM(WireRope.Group.M5, IWRC, 0.020), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> WireRope.workingLoadLimitN(mbf, 0.5));
    }

    @Test void ropeStretchIsTensionTimesLengthOverEA() {
        // 100 m of 20 mm 6x36 IWRC under 50 kN at 100 kN/mm^2 over 184 mm^2 of steel: 0.272 m
        double area = 0.586 * Math.PI / 4 * 4e-4;
        assertEquals(50e3 * 100 / (100e9 * area), WireRope.stretchM(IWRC, 0.020, WireRope.MODULUS_PA, 100, 50e3), 1e-12);
        assertEquals(0.2716, WireRope.stretchM(IWRC, 0.020, WireRope.MODULUS_PA, 100, 50e3), 1e-4);
        assertEquals(100e9 * area / 100, WireRope.stiffnessNPerM(IWRC, 0.020, WireRope.MODULUS_PA, 100), 1e-6);
        // twice the length, half the stiffness
        assertEquals(0.5, WireRope.stiffnessNPerM(IWRC, 0.020, 100e9, 200) / WireRope.stiffnessNPerM(IWRC, 0.020, 100e9, 100), 1e-12);
        assertEquals(0, WireRope.stretchM(IWRC, 0.020, 100e9, 100, 0), 0);
    }

    @Test void bendingLifeFollowsFeyrer() {
        // by hand, 6x36 IWRC (Feyrer's 8x36 WS IWRC sZ row), 16 mm (f_d = 1) over a 400 mm sheave, S/d^2 = 140 N/mm^2,
        // grade 1960, bending length 60 d: f_l = 1.54 / (2.54 - (57.5 / 5.5)^-0.14)
        double fl = 1.54 / (2.54 - Math.pow(57.5 / 5.5, -0.14));
        double lgN = 0.584 + (0.377 - 1.750 * Math.log10(25)) * (Math.log10(140) - 0.4 * Math.log10(1960.0 / 1770)) + 6.232 * Math.log10(25) + Math.log10(fl);
        double n = WireRope.bendsToDiscard(IWRC, 0.016, 0.400, 140 * 256, WireRope.GRADE_1960_PA, 0.96);
        assertEquals(Math.pow(10, lgN), n, 1e-9 * n);
        assertEquals(65_900, n, 100);
        // the ISO 4308 groups buy life: at each group's least sheave and full Zp loading, more bends from M1 to M8
        double previous = 0;
        for (WireRope.Group g : WireRope.Group.values()) {
            double d = 0.020, tension = WireRope.workingLoadLimitN(IWRC, d, WireRope.GRADE_1960_PA, g);
            double bends = WireRope.bendsToDiscard(IWRC, d, WireRope.minSheaveDiameterM(g, IWRC, d), tension, WireRope.GRADE_1960_PA, 60 * d);
            assertTrue(bends > previous, g.name());
            previous = bends;
        }
        // M5 lands in the tens of thousands of bends
        double m5 = WireRope.bendsToDiscard(IWRC, 0.020, 0.400, WireRope.workingLoadLimitN(IWRC, 0.020, WireRope.GRADE_1960_PA, WireRope.Group.M5), WireRope.GRADE_1960_PA, 1.2);
        assertTrue(m5 > 1e4 && m5 < 1e5, "M5 bends " + m5);
        // more tension, fewer bends; a bigger sheave, more; no tension, never
        assertTrue(WireRope.bendsToDiscard(RR, 0.016, 0.4, 40e3, 1960e6, 1) < WireRope.bendsToDiscard(RR, 0.016, 0.4, 20e3, 1960e6, 1));
        assertTrue(WireRope.bendsToDiscard(RR, 0.016, 0.5, 40e3, 1960e6, 1) > WireRope.bendsToDiscard(RR, 0.016, 0.4, 40e3, 1960e6, 1));
        assertEquals(Double.POSITIVE_INFINITY, WireRope.bendsToDiscard(RR, 0.016, 0.4, 0, 1960e6, 1));
        // outside the regression: thin ropes, short bends, tiny sheaves
        assertThrows(IllegalArgumentException.class, () -> WireRope.bendsToDiscard(IWRC, 0.005, 0.2, 1e3, 1960e6, 1));
        assertThrows(IllegalArgumentException.class, () -> WireRope.bendsToDiscard(IWRC, 0.016, 0.4, 1e3, 1960e6, 0.1));
        assertThrows(IllegalArgumentException.class, () -> WireRope.bendsToDiscard(RR, 0.016, 0.048, 1e3, 1960e6, 1));
    }

    @Test void reevingMatchesAMakersHookBlocksAndLosesToItsSheaves() {
        // Liebherr LTM 1040-2.1: 34 kN line pull, hook blocks of 3, 7 and 11 parts rated 10.1, 22.8 and 34.9 t
        int[] parts = {3, 7, 11};
        double[] rated = {10.1e3, 22.8e3, 34.9e3};
        for (int i = 0; i < parts.length; i++) {
            double kg = Reeving.hookLoadN(parts[i], 34e3, Reeving.SHEAVE_ROLLING, 0) / G;
            assertEquals(rated[i], kg, 0.02 * rated[i], parts[i] + " parts");
        }
        // four parts lift a little under twice what two do; every sheave costs
        double two = Reeving.hookLoadN(2, 10e3, Reeving.SHEAVE_ROLLING, 0), four = Reeving.hookLoadN(4, 10e3, Reeving.SHEAVE_ROLLING, 0);
        assertTrue(four / two > 1.9 && four / two < 2.0, "four over two parts " + four / two);
        assertEquals(1, Reeving.efficiency(1, 0.98), 0);
        assertEquals((1 - Math.pow(0.98, 4)) / (4 * 0.02), Reeving.efficiency(4, 0.98), 1e-15);
        for (int n = 2; n < 20; n++) { assertTrue(Reeving.efficiency(n, 0.98) < Reeving.efficiency(n - 1, 0.98)); }
        assertTrue(Reeving.efficiency(8, Reeving.SHEAVE_PLAIN) < Reeving.efficiency(8, Reeving.SHEAVE_ROLLING));
        assertEquals(1, Reeving.efficiency(8, 1), 0);
        // a guide sheave costs one more eta
        assertEquals(0.98 * four, Reeving.hookLoadN(4, 10e3, 0.98, 1), 1e-9);
        // round trip, hook speed and the rope to store
        assertEquals(10e3, Reeving.linePullN(4, four, 0.98, 0), 1e-9);
        assertEquals(0.25, Reeving.hookSpeedMps(1.0, 4), 0);
        assertEquals(120, Reeving.storedRopeM(30, 4), 0);
        assertEquals(4 * 30 + 15 + 2 * Math.PI * 0.4, Reeving.reevedLengthM(4, 30, 15, 0.4), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> Reeving.efficiency(0, 0.98));
        assertThrows(IllegalArgumentException.class, () -> Reeving.efficiency(4, 1.02));
    }

    @Test void outerLayersPullLessAndRunFaster() {
        // a 400 mm barrel, 16 mm rope, 800 mm between the flanges at a 16 mm pitch: 50 turns a layer
        int turns = Winch.turnsPerLayer(0.8, 0.016);
        assertEquals(50, turns);
        assertEquals(0.416, Winch.layerDiameterM(0.4, 0.016, 1), 1e-12);
        assertEquals(0.416 + Math.sqrt(3) * 0.016, Winch.layerDiameterM(0.4, 0.016, 2), 1e-12);
        assertEquals(0.416 + 2 * 0.016, Winch.layerDiameterM(0.4, 0.016, 2, 1), 1e-12);
        // 1000 N m through 30:1 at 95 %: 137 kN on the first layer, less on each one out; the rope runs faster
        double previousPull = Double.POSITIVE_INFINITY, previousSpeed = 0, previousLength = 0;
        for (int layer = 1; layer <= 6; layer++) {
            double d = Winch.layerDiameterM(0.4, 0.016, layer);
            double pull = Winch.linePullN(1000, 30, 0.95, d), speed = Winch.ropeSpeedMps(150, 30, d);
            double length = Winch.layerLengthM(0.4, 0.016, turns, layer);
            assertTrue(pull < previousPull && speed > previousSpeed && length > previousLength);
            assertEquals(1000, Winch.motorTorqueNm(pull, 30, 0.95, d), 1e-9);
            previousPull = pull;
            previousSpeed = speed;
            previousLength = length;
        }
        assertEquals(137_019, Winch.linePullN(1000, 30, 0.95, 0.416), 1);
        assertEquals(1.04, Winch.ropeSpeedMps(150, 30, 0.416), 1e-12);
        // capacity: the layers add up, and the stored rope says which layer is working
        double first = Winch.layerLengthM(0.4, 0.016, turns, 1);
        assertEquals(50 * Math.PI * 0.416, first, 1e-12);
        assertEquals(first + Winch.layerLengthM(0.4, 0.016, turns, 2) + Winch.layerLengthM(0.4, 0.016, turns, 3), Winch.capacityM(0.4, 0.016, turns, 3), 1e-9);
        assertEquals(1, Winch.layerFor(0.4, 0.016, turns, 0));
        assertEquals(1, Winch.layerFor(0.4, 0.016, turns, first));
        assertEquals(2, Winch.layerFor(0.4, 0.016, turns, first + 1));
        assertEquals(3, Winch.layerFor(0.4, 0.016, turns, Winch.capacityM(0.4, 0.016, turns, 2) + 1));
        assertThrows(IllegalArgumentException.class, () -> Winch.turnsPerLayer(0.01, 0.016));
        assertThrows(IllegalArgumentException.class, () -> Winch.layerDiameterM(0.4, 0.016, 0));
    }

    /** A 30 t machine centred on a square of pads 6 m across. */
    private static Stability.Crane squareMachine() {
        List<Stability.Point> pads = List.of(new Stability.Point(3, 3), new Stability.Point(-3, 3), new Stability.Point(-3, -3), new Stability.Point(3, -3));
        return new Stability.Crane(pads, List.of(new Stability.Mass(30_000, 0, 0)), new Stability.Point(0, 0), List.of());
    }

    @Test void aSquareOfOutriggersBalancesLikeAHandMomentSum() {
        Stability.Crane crane = squareMachine();
        // 30 t 3 m inside the edge against the load 7 m outside it at 10 m
        assertEquals(30_000 * 3 / 7.0, Stability.tippingLoadKg(crane, 10, 0), 1e-9);
        // the same over every side
        for (int i = 0; i < 4; i++) { assertEquals(30_000 * 3 / 7.0, Stability.tippingLoadKg(crane, 10, i * Math.PI / 2), 1e-6); }
        // over a pad the load is 10 / sqrt(2) - 3 outside both edges there
        assertEquals(30_000 * 3 / (10 / Math.sqrt(2) - 3), Stability.tippingLoadKg(crane, 10, Math.PI / 4), 1e-6);
        // the moments about the governing edge with 5 t at 10 m
        List<Stability.Mass> placed = new ArrayList<>(Stability.placed(crane, 0));
        placed.add(new Stability.Mass(5_000, 10, 0));
        Stability.Moments m = Stability.governing(crane.support(), placed);
        assertEquals(5_000 * G * 7, m.overturningNm(), 1e-6);
        assertEquals(30_000 * G * 3, m.stabilisingNm(), 1e-6);
        assertEquals(3, m.from().xM(), 0);
        assertEquals(3, m.to().xM(), 0);
        assertEquals(30_000 * 3 / (5_000 * 7.0), m.ratio(), 1e-12);
        // over the supports nothing tips it
        assertEquals(Double.POSITIVE_INFINITY, Stability.tippingLoadKg(crane, 2.5, 0.3));
        // a counterweight slews with the superstructure: 10 t 4 m behind the slewing centre helps over the front
        Stability.Crane ballasted = crane.withUpper(new Stability.Mass(10_000, -4, 0));
        assertEquals((30_000 * 3 + 10_000 * 7) / 7.0, Stability.tippingLoadKg(ballasted, 10, 0), 1e-9);
        // and tips the empty machine backward when it is heavy enough: no load is then safe
        Stability.Crane overBallasted = crane.withUpper(new Stability.Mass(40_000, -6, 0));
        assertEquals(0, Stability.tippingLoadKg(overBallasted, 10, 0), 0);
        // the pads' order does not matter; a line of pads is no polygon
        assertEquals(4, Stability.supportPolygon(List.of(new Stability.Point(3, 3), new Stability.Point(-3, -3), new Stability.Point(3, -3), new Stability.Point(0, 0), new Stability.Point(-3, 3))).size());
        assertThrows(IllegalArgumentException.class, () -> Stability.supportPolygon(List.of(new Stability.Point(0, 0), new Stability.Point(1, 1), new Stability.Point(2, 2))));
    }

    @Test void theLeastStableDirectionOfASquareIsOverASideNotOverAPad() {
        Stability.Worst worst = Stability.worstSlew(squareMachine(), 10);
        assertEquals(30_000 * 3 / 7.0, worst.tippingKg(), 1e-6);
        assertEquals(0, Math.sin(2 * worst.slewRad()), 1e-6);
        // over a pad it holds 72 % more: the corner is farther out than the edges either side of it
        assertTrue(Stability.tippingLoadKg(squareMachine(), 10, Math.PI / 4) > 1.7 * worst.tippingKg());
        // a long, narrow base tips over its long side
        List<Stability.Point> narrow = List.of(new Stability.Point(5, 2), new Stability.Point(-5, 2), new Stability.Point(-5, -2), new Stability.Point(5, -2));
        Stability.Crane truck = new Stability.Crane(narrow, List.of(new Stability.Mass(30_000, 0, 0)), new Stability.Point(0, 0), List.of());
        Stability.Worst side = Stability.worstSlew(truck, 10);
        assertEquals(1, Math.abs(Math.sin(side.slewRad())), 1e-6);
        assertEquals(30_000 * 2 / 8.0, side.tippingKg(), 1e-6);
    }

    @Test void ratedLoadsAreTheRulesShareOfTipping() {
        assertEquals(8_500, Stability.Rule.OUTRIGGERS.ratedKg(10_000, 2_000), 1e-9);
        assertEquals(7_500, Stability.Rule.CRAWLER.ratedKg(10_000, 2_000), 1e-9);
        assertEquals(7_500, Stability.Rule.TYRES.ratedKg(10_000, 2_000), 1e-9);
        // ISO 4305: 1.25 P + 0.1 F = tipping
        assertEquals((10_000 - 200) / 1.25, Stability.Rule.ISO_4305.ratedKg(10_000, 2_000), 1e-9);
        assertEquals((10_000 - 200) / 1.33, Stability.Rule.ISO_4305_TYRES.ratedKg(10_000, 2_000), 1e-9);
        // never negative, and nothing to rate over the supports
        assertEquals(0, Stability.Rule.ISO_4305.ratedKg(100, 2_000), 0);
        assertEquals(Double.POSITIVE_INFINITY, Stability.Rule.OUTRIGGERS.ratedKg(Double.POSITIVE_INFINITY, 0));
        assertThrows(IllegalArgumentException.class, () -> Stability.Rule.OUTRIGGERS.ratedKg(Double.NaN, 0));
        // a load chart's stability limit is that share of the least stable tipping load
        LoadChart.Rating r = LoadChart.rate360(fortyTonneMachine(), boom35(), hoist7(), duty(), 24);
        double tipping = Stability.worstSlew(LoadChart.withBoom(fortyTonneMachine(), boom35(), r.boomAngleRad()), 24).tippingKg();
        assertEquals(0.85 * tipping, r.stabilityKg(), 1e-9 * tipping);
    }

    // A two-axle 40 t all-terrain crane, after the Liebherr LTM 1040-2.1 data sheet: outriggers 6.305 m by 6.028 m, the
    // front pair 3.9155 m ahead of the slewing centre and the rear 2.3895 m behind; 6.5 t counterweight; 34 kN line pull.
    // Assumed (not on the sheet): the 13.5 t carrier's centre of gravity 0.73 m ahead of the slewing centre (24 t on the
    // road on two 12 t axles with the boom laid forward and 1.5 t of ballast), the 3.5 t turntable, cab and winch 1 m
    // behind it, the counterweight 2.6 m behind it, a 5.5 t boom with its centre of gravity at 40 % of its length, its
    // foot 1.3 m behind the slewing centre 2.7 m up, the luffing cylinder 4 m up the boom from a pivot at (0.6, 1.6) m,
    // and one equivalent box section, 450 mm wide and 600 mm high with 4 mm webs and 6 mm flanges in S960.
    private static Stability.Crane fortyTonneMachine() {
        List<Stability.Point> pads = List.of(new Stability.Point(3.9155, 3.014), new Stability.Point(3.9155, -3.014),
                new Stability.Point(-2.3895, 3.014), new Stability.Point(-2.3895, -3.014));
        return new Stability.Crane(pads, List.of(new Stability.Mass(13_500, 0.73, 0)), new Stability.Point(0, 0),
                List.of(new Stability.Mass(3_500, -1.0, 0), new Stability.Mass(6_500, -2.6, 0)));
    }

    private static LoadChart.Boom boom(double length) {
        return new LoadChart.Boom(length, 5_500, 0.40, -1.3, 2.7, 4.0, 0.6, 1.6, Section.box(0.45, 0.60, 0.004, 0.006), S960);
    }

    private static LoadChart.Boom boom35() { return boom(35.0); }

    private static LoadChart.Hoist hoist7() { return new LoadChart.Hoist(7, 34e3, Reeving.SHEAVE_ROLLING, 0); }

    private static LoadChart.Duty duty() { return new LoadChart.Duty(Stability.Rule.OUTRIGGERS, LoadChart.STRUCTURE_SAFETY, 1.0, 0); }

    @Test void aFortyTonneCranesChartIsStructureShortAndStabilityLong() {
        // the published 360-degree chart for the 35 m boom with 6.5 t counterweight, t at m
        double[] radii = {6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30};
        double[] published = {7.4, 6.9, 6.3, 5.8, 5.4, 4.4, 3.6, 3.0, 2.5, 2.1, 1.8, 1.5, 1.2};
        LoadChart.Table chart = LoadChart.chart(fortyTonneMachine(), boom35(), hoist7(), duty(), radii);
        double[] rated = chart.ratedKg();
        for (int i = 0; i < radii.length; i++) {
            double ratio = rated[i] / (1000 * published[i]);
            assertTrue(ratio > 1 / 1.5 && ratio < 1.5, radii[i] + " m: " + rated[i] + " kg against " + published[i] + " t");
            if (i > 0) { assertTrue(rated[i] < rated[i - 1], "falls with radius at " + radii[i]); }
            LoadChart.Rating r = LoadChart.rate360(fortyTonneMachine(), boom35(), hoist7(), duty(), radii[i]);
            assertEquals(rated[i], r.ratedKg(), 0);
            if (radii[i] <= 12) { assertEquals(LoadChart.Limit.STRUCTURE, r.limit(), radii[i] + " m"); }
            if (radii[i] >= 24) { assertEquals(LoadChart.Limit.STABILITY, r.limit(), radii[i] + " m"); }
            assertTrue(r.headHeightM() > 2.7 && r.headHeightM() < 2.7 + 35);
        }
        // the 30.1 m boom too, within the same factor
        double[] radii30 = {6, 10, 14, 18, 22, 26};
        double[] published30 = {10.2, 7.9, 5.4, 3.6, 2.5, 1.7};
        for (int i = 0; i < radii30.length; i++) {
            double kg = LoadChart.rate360(fortyTonneMachine(), boom(30.1), hoist7(), duty(), radii30[i]).ratedKg();
            assertTrue(kg / (1000 * published30[i]) > 1 / 1.5 && kg / (1000 * published30[i]) < 1.5, radii30[i] + " m on 30.1 m: " + kg);
        }
        // a short boom close in is held by the reeving: three parts of 34 kN
        LoadChart.Hoist three = new LoadChart.Hoist(3, 34e3, Reeving.SHEAVE_ROLLING, 0);
        LoadChart.Rating close = LoadChart.rate360(fortyTonneMachine(), boom(10.5), three, duty(), 3);
        assertEquals(LoadChart.Limit.ROPE, close.limit());
        assertEquals(LoadChart.ropeKg(three), close.ratedKg(), 0);
        // the lookup interpolates between the chart's radii and rates nothing off it
        assertEquals(0.5 * (rated[0] + rated[1]), chart.ratedKg(7), 1e-9);
        assertEquals(rated[4], chart.ratedKg(14), 1e-9);
        assertEquals(0, chart.ratedKg(5.9), 0);
        assertEquals(0, chart.ratedKg(30.1), 0);
        // out of reach
        LoadChart.Rating far = LoadChart.rate360(fortyTonneMachine(), boom35(), hoist7(), duty(), 40);
        assertEquals(LoadChart.Limit.REACH, far.limit());
        assertEquals(0, far.ratedKg(), 0);
        // the dynamic factor and a side load take from the structure, never add
        double base = LoadChart.structureKg(boom35(), hoist7(), duty(), Math.toRadians(60));
        double dynamic = LoadChart.structureKg(boom35(), hoist7(), new LoadChart.Duty(Stability.Rule.OUTRIGGERS, 1.5, LoadChart.HoistClass.HC2.dynamicFactor(0.5), 0), Math.toRadians(60));
        double side = LoadChart.structureKg(boom35(), hoist7(), new LoadChart.Duty(Stability.Rule.OUTRIGGERS, 1.5, 1.0, 0.02), Math.toRadians(60));
        assertEquals(base / 1.27, dynamic, 0.05 * base);
        assertTrue(dynamic < base && side < base);
        assertEquals(1.10 + 0.34 * 0.5, LoadChart.HoistClass.HC2.dynamicFactor(0.5), 1e-12);
    }

    @Test void aLatticeBoomOnPendantsIsAColumn() {
        // a weightless 30 m lattice boom at 60 degrees, pinned at (0, 2), its head held by pendants to a gantry at (-3, 8)
        Section chord = Section.roundTube(0.0889, 0.008);
        Section lattice = LoadChart.latticeSection(chord, 1.2, 1.2);
        assertEquals(4 * chord.areaM2(), lattice.areaM2(), 1e-15);
        assertEquals(4 * (chord.izM4() + chord.areaM2() * 0.36), lattice.izM4(), 1e-15);
        FrameMaterial s355 = new FrameMaterial(7850, 210e9, 81e9, 355e6);
        LoadChart.Boom boom = new LoadChart.Boom(30, 0, 0.5, 0, 2, 30, -3, 8, lattice, s355);
        LoadChart.Hoist single = new LoadChart.Hoist(1, 100e3, 0.98, 0);
        LoadChart.Duty duty = new LoadChart.Duty(Stability.Rule.CRAWLER, 1.5, 1.0, 0);
        double alpha = Math.toRadians(60), w = 10_000 * G;
        // the head node by hand: C e_b + P u = W z + T e_b, the hook load, the fast line and the pendant
        double ebx = Math.cos(alpha), ebz = Math.sin(alpha);
        double hx = 30 * ebx, hz = 2 + 30 * ebz, ux = -3 - hx, uz = 8 - hz, ul = Math.hypot(ux, uz);
        ux /= ul;
        uz /= ul;
        double rx = w * ebx, rz = w + w * ebz;
        double det = ebx * uz - ebz * ux;
        double compression = (rx * uz - rz * ux) / det;
        assertTrue(compression > w);
        assertEquals(compression / lattice.areaM2(), LoadChart.stressPa(boom, single, duty, alpha, 10_000), 1e-9 * compression / lattice.areaM2());
        // with its own weight a longer lattice boom holds less at the same angle
        LoadChart.Boom short30 = new LoadChart.Boom(30, 4_000, 0.5, 0, 2, 30, -3, 8, lattice, s355);
        LoadChart.Boom long60 = new LoadChart.Boom(60, 8_000, 0.5, 0, 2, 60, -3, 8, lattice, s355);
        assertTrue(LoadChart.structureKg(long60, single, duty, alpha) < LoadChart.structureKg(short30, single, duty, alpha));
    }

    @Test void aTowerCraneIsCappedByItsLoadMoment() {
        // 8 t out to 20 m, a 60 m jib from 2.5 m
        double moment = 8_000 * G * 20;
        assertEquals(20, TowerCrane.maxLoadRadiusM(8_000, moment), 1e-12);
        assertEquals(8_000, TowerCrane.ratedKg(10, 2.5, 60, 8_000, moment), 0);
        assertEquals(8_000, TowerCrane.ratedKg(20, 2.5, 60, 8_000, moment), 1e-9);
        assertEquals(4_000, TowerCrane.ratedKg(40, 2.5, 60, 8_000, moment), 1e-9);
        assertEquals(8_000 * 20 / 60.0, TowerCrane.ratedKg(60, 2.5, 60, 8_000, moment), 1e-9);
        assertEquals(0, TowerCrane.ratedKg(60.5, 2.5, 60, 8_000, moment), 0);
        assertEquals(0, TowerCrane.ratedKg(2, 2.5, 60, 8_000, moment), 0);
        LoadChart.Table chart = TowerCrane.chart(2.5, 60, 8_000, moment, 50);
        double[] radii = chart.radiiM(), rated = chart.ratedKg();
        assertEquals(60, radii[radii.length - 1], 0);
        for (int i = 0; i < radii.length; i++) {
            assertTrue(rated[i] * G * radii[i] <= moment * (1 + 1e-12) && rated[i] <= 8_000);
            if (i > 0) { assertTrue(rated[i] <= rated[i - 1]); }
        }
        // a member's elastic moment capacity: yield / safety x I / y
        Section box = Section.box(1.2, 1.6, 0.012, 0.016);
        FrameMaterial s355 = new FrameMaterial(7850, 210e9, 81e9, 355e6);
        assertEquals(355e6 / 1.5 * box.izM4() / 0.8, TowerCrane.momentCapacityNm(box, s355, 1.5), 1e-6);
        assertThrows(IllegalArgumentException.class, () -> TowerCrane.ratedKg(10, 70, 60, 8_000, moment));
    }

    @Test void windOnTheHookLoadFollowsIso4302() {
        assertEquals(250, Wind.pressurePa(Wind.IN_SERVICE_MPS), 1e-12);
        assertEquals(507.7, Wind.pressurePa(Wind.IN_SERVICE_PROCESS_MPS), 0.1);
        // 10 t of unknown form: 2.4 x 5 m^2 = 12 m^2 at 250 Pa
        assertEquals(3_000, Wind.loadForceN(10_000, 20), 1e-9);
        assertEquals(Math.atan(3_000 / (10_000 * G)), Wind.swingAngleRad(3_000, 10_000), 1e-15);
        // 10 t with 30 m^2 at c_w 1.2 has three times the standard 12 m^2: the chart's 9 m/s falls to 9 / sqrt(3)
        assertEquals(9 / Math.sqrt(3), Wind.permissibleMps(Wind.MOBILE_CHART_GUST_MPS, 10_000, 1.2, 30), 1e-12);
        // a smaller area does not raise it
        assertEquals(9, Wind.permissibleMps(9, 10_000, 1.2, 2), 0);
        assertEquals(0, Wind.loadForceN(0, 20), 0);
    }

    @Test void nothingIsNaNOverSweeps() {
        Stability.Crane machine = fortyTonneMachine();
        for (double length : new double[]{10.5, 20.3, 35}) {
            LoadChart.Boom boom = boom(length);
            for (double r = 0; r <= 40; r += 0.5) {
                for (double slew = -7; slew <= 7; slew += 1.3) {
                    LoadChart.Rating rating = LoadChart.rate(machine, boom, hoist7(), duty(), r, slew);
                    assertFalse(Double.isNaN(rating.ratedKg()) || Double.isNaN(rating.stabilityKg()) || Double.isNaN(rating.structureKg()));
                    assertFalse(Double.isNaN(rating.boomAngleRad()) || Double.isNaN(rating.headHeightM()));
                    assertTrue(rating.ratedKg() >= 0 && Double.isFinite(rating.ratedKg()));
                }
            }
            for (double angle = 0; angle <= Math.PI / 2; angle += Math.PI / 40) {
                for (double load = 0; load <= 60_000; load += 7_500) {
                    double s = LoadChart.stressPa(boom, hoist7(), duty(), angle, load);
                    assertTrue(s >= 0 && !Double.isNaN(s));
                }
            }
        }
        for (int layer = 1; layer <= 30; layer++) {
            assertTrue(Double.isFinite(Winch.linePullN(500, 20, 0.9, Winch.layerDiameterM(0.3, 0.012, layer))));
        }
        for (double ratio = 10; ratio <= 60; ratio += 2.5) {
            for (double tension = 1; tension <= 3e5; tension *= 3) {
                double n = WireRope.bendsToDiscard(IWRC, 0.02, 0.02 * ratio, tension, 1960e6, 1);
                assertTrue(n > 0 && !Double.isNaN(n));
            }
        }
        for (double slew = -10; slew <= 10; slew += 0.37) {
            for (double r = 0; r <= 30; r += 1.5) {
                double t = Stability.tippingLoadKg(machine, r, slew);
                assertTrue(t >= 0 && !Double.isNaN(t));
            }
        }
        assertThrows(IllegalArgumentException.class, () -> LoadChart.rate360(machine, boom35(), hoist7(), duty(), Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new LoadChart.Boom(35, 5_500, 1.2, -1.3, 2.7, 4, 0.6, 1.6, Section.box(0.45, 0.6, 0.004, 0.006), S960));
        assertThrows(IllegalArgumentException.class, () -> new LoadChart.Hoist(0, 34e3, 0.98, 0));
        assertThrows(IllegalArgumentException.class, () -> new LoadChart.Duty(Stability.Rule.OUTRIGGERS, 0.9, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> new LoadChart.Table(new double[]{2, 1}, new double[]{1, 1}));
    }
}

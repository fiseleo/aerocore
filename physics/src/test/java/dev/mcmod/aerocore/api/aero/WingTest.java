package dev.mcmod.aerocore.api.aero;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** The lifting line against finite-wing theory: Helmbold's and DATCOM's lift slopes, induced drag, ground effect, controls. */
class WingTest {
    private static final Airfoil N0012 = Airfoil.naca4(0, 0, 0.12), N2412 = Airfoil.naca4(0.02, 0.4, 0.12);
    private static final Wing.Air AIR = Wing.Air.SEA_LEVEL;
    private static final double V = 50;

    /** Lift and drag coefficients of a solution at an angle. */
    private static double[] coefficients(Wing w, Wing.Solution s, double alpha) {
        double q = 0.5 * AIR.densityKgM3() * V * V * w.planform().areaM2();
        Vec3d f = s.force(), pf = s.profileForce();
        double lift = f.x() * Math.sin(alpha) + f.y() * Math.cos(alpha);
        double drag = -(f.x() * Math.cos(alpha) - f.y() * Math.sin(alpha));
        double profile = -(pf.x() * Math.cos(alpha) - pf.y() * Math.sin(alpha));
        return new double[]{lift / q, drag / q, (drag - profile) / q};
    }

    private static double slope(Wing w) {
        double a1 = Math.toRadians(2), a2 = Math.toRadians(4);
        return (coefficients(w, w.solve(V, a2, 0, AIR, Map.of()), a2)[0] - coefficients(w, w.solve(V, a1, 0, AIR, Map.of()), a1)[0]) / (a2 - a1);
    }

    /** DATCOM (Helmbold for no sweep): 2 pi A / (2 + sqrt(A^2 / k^2 (1 + tan^2 sweep) + 4)), k = a0 / 2 pi. */
    private static double datcom(Wing w) {
        Wing.Planform p = w.planform();
        double a0 = w.rootPolar().liftSlope(AIR.densityKgM3() * V * p.meanChordM() / AIR.viscosityPaS()), k = a0 / (2 * Math.PI), a = p.aspectRatio();
        double t = Math.tan(Math.toRadians(p.sweepDeg()));
        return 2 * Math.PI * a / (2 + Math.sqrt(a * a / (k * k) * (1 + t * t) + 4));
    }

    @Test void liftSlopeFollowsHelmboldAndDatcom() {
        Wing rect = new Wing(new Wing.Planform(6, 1, 1, 0, 0, 0, 0, true), N0012, N0012, List.of(), 24);
        Wing taper = new Wing(new Wing.Planform(8, 1.4286, 0.5714, 0, 0, 0, 0, true), N0012, N0012, List.of(), 24);
        Wing swept = new Wing(new Wing.Planform(6, 1, 1, 30, 0, 0, 0, true), N0012, N0012, List.of(), 24);
        for (Wing w : new Wing[]{rect, taper}) { assertEquals(datcom(w), slope(w), 0.04 * datcom(w), w.planform().toString()); }
        assertEquals(datcom(swept), slope(swept), 0.08 * datcom(swept), "30 degrees of sweep");
        assertTrue(slope(swept) < slope(rect), "sweep lowers the slope");
    }

    @Test void inducedDragIsNearTheEllipticMinimum() {
        Wing taper = new Wing(new Wing.Planform(8, 1.4286, 0.5714, 0, 0, 0, 0, true), N0012, N0012, List.of(), 24);
        Wing rect = new Wing(new Wing.Planform(6, 1, 1, 0, 0, 0, 0, true), N0012, N0012, List.of(), 24);
        for (Wing w : new Wing[]{taper, rect}) {
            double a = Math.toRadians(6);
            double[] c = coefficients(w, w.solve(V, a, 0, AIR, Map.of()), a);
            double e = c[0] * c[0] / (Math.PI * w.planform().aspectRatio() * c[2]);
            // a taper of 0.4 is within 1 % of elliptic loading; a rectangular wing of aspect ratio 6 about 0.96 (Glauert)
            double expected = w == taper ? 0.99 : 0.96;
            assertEquals(expected, e, 0.03, w.planform() + " span efficiency");
        }
    }

    @Test void stallSpreadsAndLiftFallsPastTheMaximum() {
        Wing w = new Wing(new Wing.Planform(10, 1.5, 0.75, 0, 3, 0, -2, true), N2412, N2412, List.of());
        double best = 0, at = 0;
        for (double d = 0; d <= 26; d += 1) {
            double a = Math.toRadians(d);
            double cl = coefficients(w, w.solve(V, a, 0, AIR, Map.of()), a)[0];
            if (cl > best) { best = cl; at = d; }
        }
        double re = AIR.densityKgM3() * V * 1.15 / AIR.viscosityPaS();
        // a straight wing's maximum is 0.8-1.0 of its sections' (Raymer, DATCOM)
        double ratio = best / SectionPolar.of(N2412).maxLift(re);
        assertTrue(ratio > 0.75 && ratio < 1.02, "CLmax / clmax " + ratio);
        double past = coefficients(w, w.solve(V, Math.toRadians(at + 6), 0, AIR, Map.of()), Math.toRadians(at + 6))[0];
        assertTrue(past < best, "lift falls past the stall");
        assertTrue(w.solve(V, Math.toRadians(at + 6), 0, AIR, Map.of()).stalledShare() > 0.3);
    }

    @Test void groundEffectRaisesLiftAndCutsInducedDrag() {
        Wing w = new Wing(new Wing.Planform(10, 1.4, 1.0, 0, 0, 0, 0, true), N2412, N2412, List.of());
        Vec3d[] flow = new Vec3d[w.size()];
        double a = Math.toRadians(5);
        Arrays.fill(flow, new Vec3d(-V * Math.cos(a), V * Math.sin(a), 0));
        Wing.Solution free = w.solve(flow, AIR, Map.of(), Double.POSITIVE_INFINITY, null), near = w.solve(flow, AIR, Map.of(), 1.0, null);
        assertTrue(near.force().y() > 1.05 * free.force().y(), "more lift at a tenth of the span: " + near.force() + " " + free.force());
        assertTrue(coefficients(w, near, a)[2] < 0.7 * coefficients(w, free, a)[2], "less induced drag");
    }

    @Test void controlSurfacesRollAndLiftTheWing() {
        Wing w = new Wing(new Wing.Planform(11, 1.6, 1.1, 0, 3, 2, -2, true), N2412, Airfoil.naca4(0.02, 0.4, 0.10),
                List.of(new Wing.ControlSurface("flap", SectionPolar.FlapType.SLOTTED, 0.3, 0, 0.55, 30, 0, false),
                        new Wing.ControlSurface("aileron", SectionPolar.FlapType.PLAIN, 0.25, 0.6, 0.95, 20, 20, true)));
        double a = Math.toRadians(3);
        Wing.Solution clean = w.solve(V, a, 0, AIR, Map.of());
        Wing.Solution flap = w.solve(V, a, 0, AIR, Map.of("flap", 1.0));
        Wing.Solution roll = w.solve(V, a, 0, AIR, Map.of("aileron", 1.0));
        assertTrue(flap.force().y() > 1.3 * clean.force().y(), "flaps raise the lift");
        assertTrue(flap.moment().z() < clean.moment().z(), "and pitch the nose down");
        assertEquals(0, clean.moment().x(), 1e-6 * clean.force().y(), "symmetric");
        // +1 aileron: starboard trailing edge up, port down, the starboard wing drops (roll to starboard, about +x)
        assertTrue(roll.moment().x() > 0.02 * clean.force().y(), "roll to starboard: " + roll.moment());
        assertEquals(clean.force().y(), roll.force().y(), 0.05 * clean.force().y(), "ailerons barely change the lift");
    }

    @Test void structureCarriesSchrenksLoadAndWeighsLikeALightAircraftWing() {
        // a rectangular 11 m wing of 1.5 m chord (a light aircraft): ultimate 1.5 x 3.8 g at 1100 kg
        Wing.Planform p = new Wing.Planform(11, 1.5, 1.5, 0, 0, 0, 0, true);
        double lift = 1.5 * 3.8 * 1100 * 9.81;
        WingStructure.Box box = new WingStructure.Box(0.15, 0.65, 0.004, 0.0015, 0.5, 0.0006, 0.4, 0.0008);
        WingStructure.Materials alu = new WingStructure.Materials(2780, 73e9, 2780);
        WingStructure.Result r = WingStructure.analyze(p, 0.12, 0.12, box, alu, lift);
        // Schrenk on a rectangle: the mean of b/2 x 1/2 (uniform) and b/2 x 4/(3 pi) (elliptic) times the half's lift
        double half = lift / 2, arm = 5.5 * 0.5 * (0.5 + 4 / (3 * Math.PI));
        assertEquals(half, r.rootShearN(), 1e-3 * half);
        assertEquals(half * arm, r.rootMomentNm(), 0.01 * half * arm);
        assertEquals(0, r.capStressAt(), 0.05, "the root is the most stressed");
        // aluminium wings of light aircraft weigh about 8-12 kg/m^2 (Raymer, table 15.2)
        double perArea = r.massKg() / p.areaM2();
        assertTrue(perArea > 4 && perArea < 14, "wing mass per area " + perArea);
        assertTrue(r.capStressPa() < 500e6 && r.capStressPa() > 50e6, "cap stress " + r.capStressPa());
        WingStructure.Result stiff = WingStructure.analyze(p, 0.12, 0.12, box, new WingStructure.Materials(2780, 146e9, 2780), lift);
        assertEquals(r.tipDeflectionM() / 2, stiff.tipDeflectionM(), 1e-9, "deflection goes as 1/E");
        WingStructure.Result thick = WingStructure.analyze(p, 0.12, 0.12, new WingStructure.Box(0.15, 0.65, 0.008, 0.0015, 0.5, 0.0006, 0.4, 0.0008), alu, lift);
        assertEquals(r.capStressPa() / 2, thick.capStressPa(), 1e-6 * r.capStressPa());
        assertTrue(thick.capsKg() > 1.9 * r.capsKg());
    }

    @Test void warmStartsGiveTheSameLoadsAndStillAirGivesNone() {
        Wing w = new Wing(new Wing.Planform(9, 1.3, 0.9, 5, 4, 1, -2, true), N2412, N2412, List.of(), 8);
        Wing.Solution cold = w.solve(V, Math.toRadians(4), Math.toRadians(3), AIR, Map.of());
        Vec3d[] flow = new Vec3d[w.size()];
        double a = Math.toRadians(4), b = Math.toRadians(3);
        Arrays.fill(flow, new Vec3d(-Math.cos(a) * Math.cos(b), Math.sin(a) * Math.cos(b), -Math.sin(b)).scale(V));
        Wing.Solution warm = w.solve(flow, AIR, Map.of(), Double.POSITIVE_INFINITY, cold.strength());
        assertEquals(cold.force().y(), warm.force().y(), 1e-3 * cold.force().y());
        assertEquals(cold.moment().y(), warm.moment().y(), 1e-3 * Math.abs(cold.moment().y()) + 1e-6);
        Arrays.fill(flow, Vec3d.ZERO);
        Wing.Solution none = w.solve(flow, AIR, Map.of(), Double.POSITIVE_INFINITY, null);
        assertEquals(Vec3d.ZERO, none.force());
        // sideslip with dihedral rolls the wing away from the side wind (dihedral effect): wind from starboard rolls it to port
        assertTrue(cold.moment().x() < 0, "dihedral effect: " + cold.moment());
    }

    /**
     * A light aircraft's wing (NACA 2415 to 2412, slotted flaps inboard) swept through the stall, each point continued
     * from the last: past the stall the drag only rises and the lift falls off without jumps. Solved cold point by point,
     * the lifting line jumped between its post-stall solutions (the drag fell back from 0.21 to 0.18 a degree later) and
     * Newton's step alone left residuals up to 0.4.
     */
    @Test void aSweepThroughTheStallIsContinuous() {
        Wing wing = new Wing(new Wing.Planform(17.7, 2.2, 1.75, 0, 3, 0, -3, true), Airfoil.naca4(0.02, 0.4, 0.15), Airfoil.naca4(0.02, 0.4, 0.12),
                List.of(new Wing.ControlSurface("flap", SectionPolar.FlapType.SLOTTED, 0.3, 0.08, 0.6, 30, 0, false),
                        new Wing.ControlSurface("aileron", SectionPolar.FlapType.PLAIN, 0.25, 0.65, 0.95, 20, 20, true)), 12);
        double q = 0.5 * Wing.Air.SEA_LEVEL.densityKgM3() * 50 * 50 * wing.planform().areaM2();
        for (Map<String, Double> inputs : List.of(Map.<String, Double>of(), Map.of("flap", 1.0))) {
            double[] previous = null;
            double lastCl = Double.NaN, lastCd = Double.NaN, worst = 0;
            for (int d = 4; d <= 26; d++) {
                double a = Math.toRadians(d);
                Wing.Solution s = wing.solve(50, a, 0, Wing.Air.SEA_LEVEL, inputs, previous);
                previous = s.strength();
                Vec3d f = s.force();
                double cl = (f.x() * Math.sin(a) + f.y() * Math.cos(a)) / q, cd = -(f.x() * Math.cos(a) - f.y() * Math.sin(a)) / q;
                if (!Double.isNaN(lastCd)) {
                    assertTrue(cd >= lastCd - 1e-4, inputs + " drag at " + d + " deg: " + cd + " after " + lastCd);
                    assertEquals(lastCl, cl, 0.25, inputs + " lift at " + d + " deg");
                }
                worst = Math.max(worst, s.residual());
                lastCl = cl; lastCd = cd;
            }
            assertTrue(worst < 0.15, inputs + " residual " + worst);
        }
    }
}

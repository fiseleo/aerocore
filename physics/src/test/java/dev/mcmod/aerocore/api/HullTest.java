package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.ArrayList;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class HullTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, RHO = FluidState.FRESH_WATER_20C.densityKgPerM3();
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final HullDynamics.Environment CALM = point -> HullDynamics.Water.calm(0, FluidState.FRESH_WATER_20C, AIR);

    @Test void sectionClipIsExactAtAnyHeel() {
        HullSection box = HullSection.rounded(0, 1, -1, 1, 0, 4);
        HullSection.Wet half = box.clip(1, 0, 0);
        assertEquals(2, half.areaM2(), 1e-12);
        assertEquals(-0.5, half.centroidY(), 1e-12);
        assertEquals(0, half.centroidZ(), 1e-12);
        assertEquals(2, half.waterlineBreadthM(), 1e-12);
        assertEquals(1, half.draftM(), 1e-12);
        assertEquals(4, half.girthM(), 1e-12);
        // Any line through the centre of a symmetric box halves its area; at 45 degrees the waterline is the diagonal.
        HullSection.Wet heeled = box.clip(1, 1, 0);
        assertEquals(2, heeled.areaM2(), 1e-12);
        assertEquals(2 * Math.sqrt(2), heeled.waterlineBreadthM(), 1e-12);
        assertEquals(Math.sqrt(2), heeled.draftM(), 1e-12);
        HullSection ellipse = HullSection.ellipse(0, 0.5, 0, 0.3, 256);
        assertEquals(Math.PI * 0.5 * 0.3, ellipse.area(), 1e-3 * ellipse.area());
        assertEquals(0.5 * Math.sqrt(1 - 0.25), ellipse.halfBreadthAt(0.15), 2e-4);
        assertTrue(box.clip(1, 0, -2).areaM2() == 0 && !box.clip(1, 0, 5).piercing());
    }
    @Test void lewisFormReproducesTheCircleAndTheSquareSection() {
        double r = 0.4;
        double[] circle = HullManoeuvring.lewisAddedMass(2 * r, r, Math.PI * r * r / 2, RHO);
        assertEquals(RHO * Math.PI * r * r / 2, circle[0], 1e-9 * circle[0]);
        assertEquals(RHO * Math.PI * r * r / 2, circle[1], 1e-9 * circle[1]);
        // Half-square (B/T = 2, sigma = 1): sway m22 / (rho pi T^2 / 2) = 1.433 for the Lewis form.
        double[] square = HullManoeuvring.lewisAddedMass(2, 1, 2, RHO);
        assertEquals(1.433, square[0] / (RHO * Math.PI / 2), 0.002);
    }
    @Test void spheroidCoefficientsMatchLambsValues() {
        double[] k = HullManoeuvring.spheroidCoefficients(6);
        assertEquals(0.045, k[0], 0.001);
        assertEquals(0.917, k[1], 0.002);
        assertEquals(0.762, k[2], 0.003);
        assertEquals(0.5, HullManoeuvring.spheroidCoefficients(1.0001)[0], 0.001, "Sphere limit");
    }
    @Test void kirchhoffTermsGiveTheMunkMomentAndConserveIdealFluidInvariants() {
        // Neutrally buoyant prolate spheroid, L/D = 6, in ideal fluid with Lamb's added masses.
        double a = 1.5, b = 0.25, volume = 4.0 / 3 * Math.PI * a * b * b, mass = RHO * volume;
        double[] k = HullManoeuvring.spheroidCoefficients(a / b);
        double iy = mass * (a * a + b * b) / 5, ix = 0.4 * mass * b * b;
        RigidBodyProperties p = new RigidBodyProperties(mass, Vec3d.ZERO, new InertiaTensor(ix, iy, iy, 0, 0, 0));
        double[][] added = new double[6][6];
        added[0][0] = k[0] * mass; added[1][1] = added[2][2] = k[1] * mass; added[3][3] = 1e-9; added[4][4] = added[5][5] = k[2] * iy;
        // Munk couple on a translating body: (m22 - m11) u w about +y.
        Wrench munk = HullDynamics.coriolis(added, Orientation.IDENTITY, new Vec3d(2, 0, 0.3), Vec3d.ZERO);
        assertEquals((k[1] - k[0]) * mass * 2 * 0.3, munk.torque().y(), 1e-9);
        assertEquals(0, munk.force().length(), 1e-12);
        // Free motion: world impulse P = m v + R (A nu)_v and energy T = 1/2 nu^T (M + A) nu are invariants.
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, new Vec3d(1, 0, 0.2), Orientation.IDENTITY, p.inertia().applyWorld(Orientation.IDENTITY, new Vec3d(0, 0.1, 0.05)));
        double dt = 0.002;
        Vec3d impulse0 = impulse(s, p, added);
        double energy0 = energy(s, p, added);
        double maxYaw = 0;
        for (int i = 0; i < 10000; i++) {
            Wrench load = HullDynamics.kirchhoff(added, s.orientation(), s.velocity(), s.angularVelocity(p));
            s = RigidBodyDynamics.drift(RigidBodyDynamics.implicitVelocity(s, p, load, HullDynamics.rotate(added, s.orientation()), dt), p, dt);
            maxYaw = Math.max(maxYaw, Math.acos(Math.clamp(s.orientation().rotate(Vec3d.X).x(), -1, 1)));
        }
        assertTrue(maxYaw > 0.5, "The Munk moment must turn the spheroid broadside: " + maxYaw);
        assertEquals(0, impulse(s, p, added).subtract(impulse0).length(), 0.01 * impulse0.length());
        assertEquals(energy0, energy(s, p, added), 0.01 * energy0);
    }
    @Test void hullFloatsLevelAtItsDesignWaterline() {
        SlenderHull hull = kvlccLike(3.0);
        double volume = hull.volumeBelow(0), mass = RHO * volume;
        HullModel model = new HullModel(hull, HullModel.Type.SURFACE, mass, FluidState.FRESH_WATER_20C, 0);
        assertEquals(0, model.design().waterlineY(), 1e-6);
        RigidBodyProperties p = properties(hull, mass);
        RigidBodyState s = new RigidBodyState(new Vec3d(0, 0.04, 0), Vec3d.ZERO, Orientation.rotationVector(new Vec3d(0, 0, 0.02)), Vec3d.ZERO);
        double sum = 0, pitch = 0;
        int count = 0;
        for (int i = 0; i < 6000; i++) {
            s = HullDynamics.step(s, p, model, CALM, Vec3d.Y.scale(-G), List.of(), null, 0.005).state();
            if (i >= 4000) { sum += s.position().y(); pitch += Math.abs(s.orientation().rotate(Vec3d.X).y()); count++; }
        }
        // Air buoyancy on the dry hull lifts it by a few tenths of a millimetre.
        assertEquals(0, sum / count, 0.005, "Mean heave");
        assertTrue(pitch / count < 0.005, "Hull must trim level: " + pitch / count);
    }
    @Test void momentumBudgetOfOneStepCloses() {
        SlenderHull hull = kvlccLike(3.0);
        double mass = RHO * hull.volumeBelow(0);
        HullModel model = new HullModel(hull, HullModel.Type.SURFACE, mass, FluidState.FRESH_WATER_20C, 0);
        RigidBodyProperties p = properties(hull, mass);
        RigidBodyState s = new RigidBodyState(new Vec3d(0.1, 0.02, -0.05), new Vec3d(1.2, 0.1, 0.3), Orientation.rotationVector(new Vec3d(0.05, 0.3, 0.02)),
                p.inertia().applyWorld(Orientation.IDENTITY, new Vec3d(0.02, 0.1, -0.03)));
        List<AppliedForce> push = List.of(new AppliedForce(AppliedForce.Kind.THRUST, new Vec3d(-1.4, -0.1, 0), new Vec3d(150, 0, 20)));
        double dt = 0.005;
        HullDynamics.Result r = HullDynamics.step(s, p, model, CALM, Vec3d.Y.scale(-G), push, null, dt);
        Wrench total = r.hydrostatics().add(r.drag()).add(r.lift()).add(r.inertia()).add(r.contact()).add(r.external());
        Vec3d momentumRate = r.state().velocity().subtract(s.velocity()).scale(mass / dt);
        Vec3d angularRate = r.state().angularMomentum().subtract(s.angularMomentum()).scale(1 / dt);
        assertEquals(0, momentumRate.subtract(total.force()).length(), 1e-6 * (1 + total.force().length()));
        assertEquals(0, angularRate.subtract(total.torque()).length(), 1e-6 * (1 + total.torque().length()));
        assertTrue(r.dragPowerW() > 0, "Drag and lift must dissipate");
    }
    @Test void virtualCaptiveDerivativesMatchClarkeAndKvlcc2() {
        // KVLCC2 model particulars (L 7.0, B 1.27, T 0.46 m, C_B 0.81); measured hull derivatives in the L^2 prime system
        // (Yasukawa and Yoshimura 2015, via ShipMMG.jl): Y'v -0.0207, N'v -0.0090, Y'r 0.0040 (with the added-mass Coriolis
        // term), N'r -0.0032. The lift element is calibrated to Clarke's Y'v and N'v, so those must match Clarke exactly;
        // Y'r and N'r are predictions of the strip added mass, Kirchhoff terms and lift element.
        SlenderHull hull = kvlccLike(7.0);
        // Small perturbations: the quadratic cross-flow drag then adds nothing measurable to the linear derivatives.
        double mass = RHO * hull.volumeBelow(0), u = 1.0, eps = 1e-4;
        HullModel model = new HullModel(hull, HullModel.Type.SURFACE, mass, FluidState.FRESH_WATER_20C, 0);
        SlenderHull.Particulars d = model.design();
        assertEquals(0.81, d.blockCoefficient(), 0.02, "Test hull should have KVLCC2's fullness");
        RigidBodyProperties p = properties(hull, mass);
        double length = d.lengthM(), half = 0.5 * RHO;
        Wrench vPlus = captive(model, p, new Vec3d(u, 0, eps), Vec3d.ZERO), vMinus = captive(model, p, new Vec3d(u, 0, -eps), Vec3d.ZERO);
        Wrench rPlus = captive(model, p, new Vec3d(u, 0, 0), new Vec3d(0, eps, 0)), rMinus = captive(model, p, new Vec3d(u, 0, 0), new Vec3d(0, -eps, 0));
        // Clarke convention: Y starboard (our z), N and r positive bow to starboard (minus our y components); COM at midships.
        double yv = (vPlus.force().z() - vMinus.force().z()) / (2 * eps) / (half * length * length * u);
        double nv = -(vPlus.torque().y() - vMinus.torque().y()) / (2 * eps) / (half * Math.pow(length, 3) * u);
        double yr = -(rPlus.force().z() - rMinus.force().z()) / (2 * eps) / (half * Math.pow(length, 3) * u);
        double nr = (rPlus.torque().y() - rMinus.torque().y()) / (2 * eps) / (half * Math.pow(length, 4) * u);
        HullManoeuvring.Clarke clarke = HullManoeuvring.Clarke.of(length, d.beamM(), d.draftM(), d.blockCoefficient());
        // At the time of writing: Y'v -0.02567, N'v -0.00892, Y'r 0.00578, N'r -0.00258 (C_B 0.807).
        assertEquals(clarke.yv(), yv, 0.01 * Math.abs(clarke.yv()), "Y'v");
        assertEquals(clarke.nv(), nv, 0.01 * Math.abs(clarke.nv()), "N'v");
        assertEquals(-0.0090, nv, 0.15 * 0.0090, "N'v against the measurement");
        assertEquals(-0.0207, yv, 0.30 * 0.0207, "Y'v against the measurement (Clarke itself is +24 %)");
        assertTrue(yr > 0 && Math.abs(yr - 0.0040) < 0.5 * 0.0040, "Y'r " + yr + " (measured 0.0040, Clarke " + clarke.yr() + ")");
        assertEquals(-0.0032, nr, 0.35 * 0.0032, "N'r (Clarke " + clarke.nr() + ")");
    }
    @Test void wigleyHullTotalResistanceMatchesTowingTankData() {
        // Kajitani et al. (1983) Wigley models, via Collier and Molyneux (NRC SR-2006-20): C_T = 4.51e-3 and 4.71e-3 at
        // Fn 0.267 (model length not stated in the table; 2.5-6 m geosims). C_T = (1 + k) C_F + C_W on wetted area.
        double length = 4, beam = length / 10, draft = length / 16;
        List<HullSection> sections = new ArrayList<>();
        int stations = 41;
        for (int i = 0; i < stations; i++) {
            double x = -length / 2 + length * i / (stations - 1), b = Math.max(0.01 * beam / 2, beam / 2 * (1 - 4 * x * x / (length * length)));
            List<double[]> points = new ArrayList<>();
            points.add(new double[]{0.1 * draft, b});
            // Parabolic sections (convex), the keel point shared by both sides.
            for (int k = 0; k <= 16; k++) { double depth = draft * k / 16; points.add(new double[]{-depth, b * (1 - depth * depth / (draft * draft))}); }
            for (int k = 15; k >= 0; k--) { double depth = draft * k / 16; points.add(new double[]{-depth, -b * (1 - depth * depth / (draft * draft))}); }
            points.add(new double[]{0.1 * draft, -b});
            sections.add(new HullSection(x, points));
        }
        SlenderHull hull = new SlenderHull(sections);
        HullModel model = new HullModel(hull, HullModel.Type.SURFACE, RHO * hull.volumeBelow(0), FluidState.FRESH_WATER_20C, 0);
        double speed = 0.267 * Math.sqrt(G * model.design().lengthM());
        double total = model.resistanceCoefficient(speed);
        assertEquals(1.09, model.formFactor(), 0.03, "Holtrop-Mennen 1 + k for the Wigley hull");
        assertTrue(total > 4.2e-3 && total < 5.0e-3, "C_T = " + total + " against 4.51e-3 to 4.71e-3 measured");
    }

    private static Vec3d impulse(RigidBodyState s, RigidBodyProperties p, double[][] added) {
        Orientation inverse = s.orientation().inverse();
        Vec3d v = inverse.rotate(s.velocity()), w = inverse.rotate(s.angularVelocity(p));
        double[] nu = {v.x(), v.y(), v.z(), w.x(), w.y(), w.z()}, pa = new double[3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 6; j++) { pa[i] += added[i][j] * nu[j]; }
        return s.velocity().scale(p.massKg()).add(s.orientation().rotate(new Vec3d(pa[0], pa[1], pa[2])));
    }
    private static double energy(RigidBodyState s, RigidBodyProperties p, double[][] added) {
        Orientation inverse = s.orientation().inverse();
        Vec3d v = inverse.rotate(s.velocity()), w = inverse.rotate(s.angularVelocity(p));
        double[] nu = {v.x(), v.y(), v.z(), w.x(), w.y(), w.z()};
        double fluid = 0;
        for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) { fluid += 0.5 * nu[i] * added[i][j] * nu[j]; }
        return s.kineticEnergy(p) + fluid;
    }
    /** KVLCC2-like full hull: constant draft T = L/15.2, half-beam (B/2)(1 - |s|^4.3), small bilge radius, 41 stations. */
    static SlenderHull kvlccLike(double length) {
        double scale = length / 7.0, halfBeam = 0.635 * scale, draft = 0.46 * scale, top = 0.3 * scale;
        List<HullSection> sections = new ArrayList<>();
        int stations = 41;
        for (int i = 0; i < stations; i++) {
            double s = -1 + 2.0 * i / (stations - 1), w = Math.max(0.02, 1 - Math.pow(Math.abs(s), 4.3));
            sections.add(HullSection.rounded(s * length / 2, halfBeam * w, -draft, top, 0.08 * scale * w, 6));
        }
        return new SlenderHull(sections);
    }
    static RigidBodyProperties properties(SlenderHull hull, double mass) {
        double length = hull.length();
        SlenderHull.Particulars d = hull.particulars(0);
        return new RigidBodyProperties(mass, Vec3d.ZERO, InertiaTensor.box(mass, new Vec3d(length, 1.2 * d.draftM(), d.beamM())));
    }
    private static Wrench captive(HullModel model, RigidBodyProperties p, Vec3d velocity, Vec3d omega) {
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, velocity, Orientation.IDENTITY, p.inertia().applyWorld(Orientation.IDENTITY, omega));
        return HullDynamics.captiveLoad(s, p, model, CALM, Vec3d.Y.scale(-G));
    }
}

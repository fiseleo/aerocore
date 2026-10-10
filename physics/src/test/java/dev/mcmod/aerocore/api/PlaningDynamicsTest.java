package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Calm-water validation against Fridsma (1969), Davidson Laboratory R-1275 (DTIC AD0708694): 9 in beam prismatic
 * models with the elliptical bow, L/b 5, VCG 0.294 b, towed horizontally through the CG at constant speed, free to
 * heave and pitch. Table 1 gives running trim and R/Delta; Fig. 11 (digitised, about +-0.1 deg) gives trim against speed.
 */
class PlaningDynamicsTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, BEAM = 9 * 0.0254;
    private static final FluidState WATER = FluidState.FRESH_WATER_20C, AIR = new FluidState(1.204, 1.81e-5, 343, Vec3d.ZERO);

    /** Steady towed result: trim (deg), CG rise over rest (beams), R / Delta. */
    record Run(double trimDeg, double riseBeams, double resistanceRatio, double staticTrimDeg, double wettedBeams, double frictionRatio, double trimRangeDeg) {}

    /**
     * Fridsma model: L/b 5, deadrise (deg), C_Delta = Delta / (w b^3), LCG in % L aft of the bow, gyradius in % L,
     * speed V/sqrt(L) in knots / sqrt(ft).
     */
    static Run tow(double deadriseDeg, double loading, double lcgPercent, double gyradiusPercent, double speedCoefficient) {
        double length = 5 * BEAM;
        PlaningHull hull = PlaningHull.fridsma(length, BEAM, Math.toRadians(deadriseDeg), 0.625 * BEAM, 48);
        double mass = loading * WATER.densityKgPerM3() * Math.pow(BEAM, 3);
        Vec3d cg = new Vec3d(length * (1 - lcgPercent / 100), 0.294 * BEAM, 0);
        double k = gyradiusPercent / 100 * length;
        RigidBodyProperties p = new RigidBodyProperties(mass, cg, new InertiaTensor(mass * Math.pow(0.4 * BEAM, 2), mass * k * k, mass * k * k, 0, 0, 0));
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
        double knots = speedCoefficient * Math.sqrt(length / 0.3048), speed = knots * 0.514444;
        RigidBodyState rest = settle(hull, p, calm, 0, 0.02, 3);
        double staticTrim = trim(rest);
        RigidBodyState towed = new RigidBodyState(rest.position(), new Vec3d(speed, 0, 0), rest.orientation(), Vec3d.ZERO);
        double dt = 5e-4, time = 8, average = 1.5, trimSum = 0, riseSum = 0, resistanceSum = 0, wetSum = 0, frictionSum = 0, low = 99, high = -99;
        int steps = (int) (time / dt), samples = 0;
        for (int i = 0; i < steps; i++) {
            PlaningDynamics.Prepared prepared = PlaningDynamics.prepare(towed, p, hull, calm, Vec3d.Y.scale(-G), List.of(), dt);
            towed = constrain(PlaningDynamics.finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null).state(), p, speed);
            if (i * dt >= time - average) {
                Wrench fluid = prepared.hydrostatics().add(prepared.hydrodynamic()).add(prepared.friction());
                resistanceSum += -fluid.force().x();
                wetSum += prepared.diagnostics().wettedLengthM(); frictionSum += prepared.diagnostics().frictionN();
                trimSum += trim(towed); low = Math.min(low, trim(towed)); high = Math.max(high, trim(towed));
                riseSum += towed.position().y() - rest.position().y();
                samples++;
            }
        }
        return new Run(trimSum / samples, riseSum / samples / BEAM, resistanceSum / samples / (mass * G), staticTrim, wetSum / samples / BEAM, frictionSum / samples / (mass * G), high - low);
    }

    /** Free floating in still water (heave and pitch only) until at rest. */
    private static RigidBodyState settle(PlaningHull hull, RigidBodyProperties p, HullDynamics.Environment calm, double speed, double dt0, double seconds) {
        RigidBodyState s = new RigidBodyState(new Vec3d(0, p.centerOfMassLocal().y() - 0.4 * hull.depth(), 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        double dt = 1e-3;
        for (int i = 0; i < seconds / dt; i++) {
            s = constrain(PlaningDynamics.step(s, p, hull, calm, Vec3d.Y.scale(-G), List.of(), dt).state(), p, speed);
            // Light artificial damping only while settling the start state, removed before any measurement.
            s = new RigidBodyState(s.position(), s.velocity().scale(0.995), s.orientation(), s.angularMomentum().scale(0.995));
        }
        return new RigidBodyState(s.position(), Vec3d.ZERO, s.orientation(), Vec3d.ZERO);
    }

    /** Towing-tank constraint: surge speed fixed, no sway, roll or yaw. */
    private static RigidBodyState constrain(RigidBodyState s, RigidBodyProperties p, double speed) {
        Vec3d omega = s.angularVelocity(p);
        Vec3d pitchOnly = new Vec3d(0, 0, omega.z());
        Vec3d forward = s.orientation().rotate(Vec3d.X);
        double pitch = Math.atan2(forward.y(), forward.x());
        Orientation upright = Orientation.rotationVector(Vec3d.Z.scale(pitch));
        return new RigidBodyState(new Vec3d(s.position().x(), s.position().y(), 0), new Vec3d(speed, s.velocity().y(), 0), upright,
                p.inertia().applyWorld(upright, pitchOnly));
    }

    private static double trim(RigidBodyState s) {
        Vec3d forward = s.orientation().rotate(Vec3d.X);
        return Math.toDegrees(Math.atan2(forward.y(), forward.x()));
    }

    @Test void fridsmaCalmWaterRunningTrimAndResistance() {
        // {deadrise, C_Delta, LCG %L, gyradius %L, V/sqrt(L), trim, R/Delta, static trim, trim tolerance} from Table 1
        // (configurations A, B, E, F, G).
        double[][] table = {
                {20, 0.608, 59.0, 25.1, 4, 4, 0.158, 0.86, 0.5},
                {20, 0.608, 62.0, 25.5, 6, 4, 0.206, 1.50, 0.5},
                {20, 0.608, 65.5, 26.2, 4, 6, 0.156, 2.30, 0.5},
                {20, 0.912, 58.0, 20.4, 4, 6, 0.150, 1.00, 0.5},
                {20, 0.912, 58.0, 20.4, 6, 5, 0.178, 1.00, 0.5},
        };
        StringBuilder report = new StringBuilder();
        for (double[] c : table) {
            Run run = tow(c[0], c[1], c[2], c[3], c[4]);
            report.append(String.format(Locale.ROOT, "C %.3f LCG %.1f V/sqrtL %.0f: trim %.2f (Fridsma %.1f, oscillation %.2f), R/D %.3f (%.3f), static trim %.2f (%.2f), rise %.3f b%n",
                    c[1], c[2], c[4], run.trimDeg(), c[5], run.trimRangeDeg(), run.resistanceRatio(), c[6], run.staticTrimDeg(), c[7], run.riseBeams()));
            assertEquals(c[7], run.staticTrimDeg(), 0.15, "static trim " + report);
            assertEquals(c[5], run.trimDeg(), c[8], "running trim " + report);
            assertTrue(run.resistanceRatio() > 0.8 * c[6] && run.resistanceRatio() < 1.2 * c[6], "resistance " + report);
            assertTrue(run.trimRangeDeg() < 0.3, "Fridsma ran steadily here: " + report);
        }
        // Fig. 11 (digitised): LCG 60 %L, C_Delta 0.608 -- trim 4.36 and 3.78 deg, CG rise 0.061 and 0.110 b at V/sqrt(L) 4 and 6;
        // LCG 70 %L porpoises from V/sqrt(L) about 5.85.
        Run four = tow(20, 0.608, 60, 25, 4), six = tow(20, 0.608, 60, 25, 6), aft = tow(20, 0.608, 70, 26.5, 6.2);
        Run aft55 = tow(20, 0.608, 70, 26.5, 5.5);
        report.append(String.format(Locale.ROOT, "LCG 60: V/sqrtL 4 trim %.2f (4.36) rise %.3f (0.061); 6: trim %.2f (3.78) rise %.3f (0.110); LCG 70 oscillation at V/sqrtL 5.5 %.2f, 6.2 %.2f deg%n",
                four.trimDeg(), four.riseBeams(), six.trimDeg(), six.riseBeams(), aft55.trimRangeDeg(), aft.trimRangeDeg()));
        System.out.println(report);
        assertEquals(0.061, four.riseBeams(), 0.03, report.toString());
        assertEquals(0.110, six.riseBeams(), 0.03, report.toString());
        assertEquals(4.36, four.trimDeg(), 0.5, report.toString());
        assertEquals(3.78, six.trimDeg(), 0.5, report.toString());
        // Porpoising onset between 5.5 and 6.2 as in the tank (5.85).
        assertTrue(aft55.trimRangeDeg() < 0.05, "steady at V/sqrt(L) 5.5: " + report);
        assertTrue(aft.trimRangeDeg() > 0.1, "porpoising at V/sqrt(L) 6.2: " + report);
    }

    @Test void testBoatGetsOnThePlaneUnderItsOwnPropeller() {
        // Self-propelled in calm water (surge, heave and pitch free): the 0.35 m screw at 35 rev/s must bring the
        // 1048 kg demonstrator onto the plane without porpoising.
        RigidBodyProperties p = TestPlaningBoat.properties();
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
        RigidBodyState s = settle(TestPlaningBoat.HULL, p, calm, 0, 0.02, 4);
        double dt = 2e-3, minTrim = 99, maxTrim = -99;
        for (int i = 0; i < 40 / dt; i++) {
            PropellerSolver.Load screw = PropellerSolver.load(s, p, TestPlaningBoat.PROPELLER, 35, WATER);
            List<AppliedForce> loads = List.of(new AppliedForce(AppliedForce.Kind.THRUST, screw.force().offset(), screw.force().force(), screw.force().couple()));
            s = PlaningDynamics.step(s, p, TestPlaningBoat.HULL, calm, Vec3d.Y.scale(-G), loads, dt).state();
            s = constrain(s, p, s.velocity().x());
            if (i * dt > 35) { minTrim = Math.min(minTrim, trim(s)); maxTrim = Math.max(maxTrim, trim(s)); }
        }
        double speed = s.velocity().x(), cv = speed / Math.sqrt(G * TestPlaningBoat.BEAM);
        String report = String.format(Locale.ROOT, "speed %.2f m/s (C_V %.2f), trim %.2f..%.2f deg", speed, cv, minTrim, maxTrim);
        System.out.println(report);
        assertTrue(cv > 2.5, "must plane: " + report);
        assertTrue(minTrim > 1.5 && maxTrim < 8 && maxTrim - minTrim < 1, "steady planing trim: " + report);
    }

    @Test void droppedBoatSettlesWithRadiationAndDoesNotDrift() {
        // Released 0.12 m low at rest: with the zero-speed heave radiation memory (as the game boat uses) the heave and
        // pitch oscillation decays and no surge builds up.
        RigidBodyProperties p = TestPlaningBoat.properties();
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
        RigidBodyState s = TestPlaningBoat.atModelOrigin(new Vec3d(0, -0.12, 0), Orientation.IDENTITY);
        double dt = 5e-3;
        List<Vec3d> points = HullDynamics.radiationPoints(TestPlaningBoat.HULL.hull(), 6);
        WaveRadiation.Memory radiation = new WaveRadiation.Memory(points, dt);
        for (int i = 0; i < 20 / dt; i++) {
            List<AppliedForce> loads = radiation.loadsAt(s, p, points, Vec3d.Y, WATER.densityKgPerM3());
            PlaningDynamics.Prepared pr = PlaningDynamics.prepare(s, p, TestPlaningBoat.HULL, calm, Vec3d.Y.scale(-G), loads, dt);
            PlaningDynamics.Result result = PlaningDynamics.finish(pr, pr.damped(), Wrench.ZERO, List.of(), 0, null);
            radiation.recordAt(result.state(), p, points, Vec3d.Y, HullDynamics.segmentRadii(TestPlaningBoat.HULL.hull(), 6, result.diagnostics().waterlineBreadth()));
            s = result.state();
        }
        String report = "velocity " + s.velocity() + ", trim " + trim(s);
        System.out.println(report);
        assertTrue(Math.abs(s.velocity().y()) < 0.02 && Math.abs(s.velocity().x()) < 0.08, report);
    }

    @Test void ventilationBlendLimits() {
        assertEquals(0, PlaningDynamics.ventilation(0.5));
        assertEquals(1, PlaningDynamics.ventilation(1.5));
        assertEquals(0.5, PlaningDynamics.ventilation(1.0), 1e-12);
    }
}

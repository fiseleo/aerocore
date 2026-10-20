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
        return tow(PlaningHull.fridsma(length, BEAM, Math.toRadians(deadriseDeg), 0.625 * BEAM, 48), length, loading, lcgPercent, gyradiusPercent, speedCoefficient);
    }

    /** The same tow for any hull of the given length (C_Delta still on Fridsma's beam). */
    static Run tow(PlaningHull hull, double length, double loading, double lcgPercent, double gyradiusPercent, double speedCoefficient) {
        double mass = loading * WATER.densityKgPerM3() * Math.pow(BEAM, 3);
        double knots = speedCoefficient * Math.sqrt(length / 0.3048);
        return towSI(hull, mass, length * (1 - lcgPercent / 100), 0.294 * BEAM, gyradiusPercent / 100 * length, knots * 0.514444);
    }

    /** A tow in SI: mass, LCG forward of the transom, VCG above the keel line, pitch gyradius, speed (m/s). */
    static Run towSI(PlaningHull hull, double mass, double lcgM, double vcgM, double gyradiusM, double speed) {
        Vec3d cg = new Vec3d(lcgM, vcgM, 0);
        double k = gyradiusM;
        RigidBodyProperties p = new RigidBodyProperties(mass, cg, new InertiaTensor(mass * Math.pow(0.4 * BEAM, 2), mass * k * k, mass * k * k, 0, 0, 0));
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
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

    /**
     * A body strung from modules to Fridsma's proportions (an elliptical bow one beam long, then the prism; the bow's
     * keel rises only to half depth, Fridsma's to the deck) runs at about the same trim and resistance as his hull.
     */
    @Test void aBodyShapedLikeFridsmasHullPlanesLikeIt() {
        double length = 5 * BEAM;
        dev.mcmod.aerocore.api.body.SectionShape vee = new dev.mcmod.aerocore.api.body.SectionShape.Chined(BEAM, 0.625 * BEAM, 0, 20, 0);
        dev.mcmod.aerocore.api.body.BodyShape body = new dev.mcmod.aerocore.api.body.BodyShape(vee.scaled(0.02), List.of(
                new dev.mcmod.aerocore.api.body.BodyModule("bow", BEAM, vee, 2, 0, Double.POSITIVE_INFINITY),
                dev.mcmod.aerocore.api.body.BodyModule.straight("prism", 4 * BEAM, vee)));
        assertTrue(body.hardChined());
        PlaningHull hull = body.planingHull();
        assertEquals(Math.toRadians(20), hull.deadrise(0), 1e-6, "the prism's deadrise");
        assertEquals(BEAM / 2, hull.chineHalfBeam(0), 1e-9, "and chine");
        Run fridsma = tow(20, 0.608, 60, 25, 4), modules = tow(hull, length, 0.608, 60, 25, 4);
        String report = "Fridsma's hull " + fridsma + ", the modules' " + modules;
        assertEquals(fridsma.trimDeg(), modules.trimDeg(), 0.5, report);
        assertEquals(fridsma.resistanceRatio(), modules.resistanceRatio(), 0.15 * fridsma.resistanceRatio(), report);
    }

    /**
     * Sajedi, Ghadimi et al. (Scientia Iranica B 28(4), 2021, tables 1 and 7), model B: 2.64 m, chine beam 0.551 m,
     * deadrise 24 deg, 86 kg, LCG 0.79 m forward of the transom, a 25 mm step 0.77 m from the stern; running trim and
     * R / Delta (from their power column over the speed) at 5, 7 and 9 m/s (C_V 2.2 to 3.9). Depth, bow, VCG and
     * gyradius are not given and are taken as 0.3 m, an elliptical bow of 0.6 m, 0.15 m and a quarter of the length.
     * The step must also take effect only at speed: at 2 m/s the stepped and unstepped hulls run alike.
     */
    @Test void aSteppedHullRunsAsSajedisModel() {
        double beam = 0.551;
        dev.mcmod.aerocore.api.body.SectionShape vee = new dev.mcmod.aerocore.api.body.SectionShape.Chined(beam, 0.3, 0, 24, 0);
        java.util.function.DoubleFunction<PlaningHull> hull = step -> new dev.mcmod.aerocore.api.body.BodyShape(vee.scaled(0.03), List.of(
                new dev.mcmod.aerocore.api.body.BodyModule("bow", 0.6, vee, 2, 0, Double.POSITIVE_INFINITY),
                dev.mcmod.aerocore.api.body.BodyModule.straight("forebody", 1.27, vee),
                new dev.mcmod.aerocore.api.body.BodyModule("afterbody", 0.77, step > 0 ? vee.raised(step) : vee, 1, step, Double.POSITIVE_INFINITY))).planingHull();
        PlaningHull stepped = hull.apply(0.025), plain = hull.apply(0);
        assertEquals(1, stepped.steps().length);
        assertEquals(0.77, stepped.steps()[0], 1e-9);
        double[][] measured = {{5, 6.44, 0.160}, {7, 4.86, 0.160}, {9, 3.55, 0.183}};
        StringBuilder report = new StringBuilder();
        for (double[] m : measured) {
            Run run = towSI(stepped, 86, 0.79, 0.15, 0.25 * 2.64, m[0]);
            report.append(String.format("%n%.0f m/s: trim %.2f (%.2f), R/D %.3f (%.3f)", m[0], run.trimDeg(), m[1], run.resistanceRatio(), m[2]));
            // at 5 m/s (C_V 2.2) the step is still wet: the unstepped hull's pre-planing trim, which 2D+t over-predicts here
            assertEquals(m[1], run.trimDeg(), m[0] < 6 ? 2.5 : 1.5, report.toString());
            assertEquals(m[2], run.resistanceRatio(), 0.3 * m[2], report.toString());
            assertTrue(run.trimRangeDeg() < 0.3, "steady: " + report);
        }
        Run slowStepped = towSI(stepped, 86, 0.79, 0.15, 0.25 * 2.64, 2), slowPlain = towSI(plain, 86, 0.79, 0.15, 0.25 * 2.64, 2);
        // (the raised afterbody still floats a little lower aft: a degree more trim, some more resistance)
        assertEquals(slowPlain.resistanceRatio(), slowStepped.resistanceRatio(), 0.25 * slowPlain.resistanceRatio(), "a wet step: " + slowStepped + " / " + slowPlain);
        assertEquals(slowPlain.trimDeg(), slowStepped.trimDeg(), 1.5, "a wet step: " + slowStepped + " / " + slowPlain);
        // faster still it keeps running steadily, its trim falling
        Run eleven = towSI(stepped, 86, 0.79, 0.15, 0.25 * 2.64, 11), nine = towSI(stepped, 86, 0.79, 0.15, 0.25 * 2.64, 9);
        assertTrue(eleven.trimDeg() < nine.trimDeg() && eleven.trimRangeDeg() < 0.3, "at 11 m/s: " + eleven + " / 9 m/s: " + nine);
    }

    /** Savitsky's prediction: trim (deg), resistance without and with the whisker spray (N), mean wetted length and L_C in beams. */
    record Savitsky(double trimDeg, double resistanceN, double withSprayN, double lambda, double chineBeams) {}

    /**
     * Savitsky's (1964, Marine Technology 1(1)) prismatic planing equations: C_Lbeta = C_L0 - 0.0065 beta C_L0^0.6,
     * C_L0 = tau^1.1 (0.012 lambda^0.5 + 0.0055 lambda^2.5 / C_V^2), centre of pressure
     * l_p / (lambda B) = 0.75 - 1 / (5.21 C_V^2 / lambda^2 + 2.39), friction (ITTC-1957, as the model) on lambda B^2 / cos(beta)
     * at his mean bottom velocity, towed through the CG with the friction B/4 tan(beta) below the keel's CG height; the trim
     * from the moment balance. The whisker spray drag of Savitsky, DeLorme and Datla (2007, Marine Technology 44(1)) is
     * added as in the model, at his trim and with the chines wet (lambda's own geometry).
     */
    static Savitsky savitsky(double beam, double deadriseDeg, double mass, double lcg, double vcg, double speed) {
        double rho = WATER.densityKgPerM3(), nu = WATER.dynamicViscosityPaS() / rho, beta = Math.toRadians(deadriseDeg);
        double weight = mass * G, cv = speed / Math.sqrt(G * beam), clb = weight / (0.5 * rho * speed * speed * beam * beam);
        double cl0 = clb;
        for (int i = 0; i < 200; i++) { cl0 = clb + 0.0065 * deadriseDeg * Math.pow(cl0, 0.6); }
        double lo = 0.3, hi = 20, lambda = 0, resistance = 0;
        for (int it = 0; it < 80; it++) {
            double tau = 0.5 * (lo + hi), t11 = Math.pow(tau, 1.1), a = 1e-4, b = 50;
            for (int i = 0; i < 80; i++) {
                double l = 0.5 * (a + b);
                if (t11 * (0.012 * Math.sqrt(l) + 0.0055 * Math.pow(l, 2.5) / (cv * cv)) > cl0) { b = l; } else { a = l; }
            }
            lambda = 0.5 * (a + b);
            double trim = Math.toRadians(tau), dynamic = 0.012 * Math.sqrt(lambda) * t11;
            double bottom = speed * Math.sqrt(Math.max(1 - (dynamic - 0.0065 * deadriseDeg * Math.pow(dynamic, 0.6)) / (lambda * Math.cos(trim)), 0.01));
            double friction = 0.5 * rho * bottom * bottom * HullResistance.ittc1957(bottom * lambda * beam / nu) * lambda * beam * beam / Math.cos(beta);
            double normal = (weight + friction * Math.sin(trim)) / Math.cos(trim);
            double lp = lambda * beam * (0.75 - 1 / (5.21 * cv * cv / (lambda * lambda) + 2.39));
            resistance = normal * Math.sin(trim) + friction * Math.cos(trim);
            if ((lp - lcg) * normal - (vcg - 0.25 * beam * Math.tan(beta)) * friction > 0) { lo = tau; } else { hi = tau; }
        }
        double tau = 0.5 * (lo + hi), trim = Math.toRadians(tau);
        double alpha = Math.atan(Math.PI * Math.tan(trim) / (2 * Math.tan(beta))), edge = 2 * alpha;
        double path = beam / (2 * Math.cos(beta) * Math.sin(edge));
        double spray = 0.5 * rho * speed * speed * HullResistance.ittc1957(speed * path / nu) * beam * beam * Math.cos(edge) / (4 * Math.sin(edge) * Math.cos(beta));
        return new Savitsky(tau, resistance, resistance + spray, lambda, lambda - Math.tan(beta) / (2 * Math.PI * Math.tan(trim)));
    }

    /**
     * Light and heavy hard-chine hulls against Savitsky (1964) with the whisker spray (Savitsky, DeLorme and Datla 2007), C_V
     * 2 to 5, C_Delta 0.15 to 0.6: running trim within 0.7 deg, resistance within 15 %, steady. A 2 m beam, 20 deg prism
     * (L/B 8, so the wetted length stays on the prism), VCG 0.3 B; the points are those inside Savitsky's range (trim 2 to
     * 15 deg, lambda up to 4, chines wet) and clear of porpoising. Before 2026-10-12 the light ones ran up to 0.9 deg flat and
     * 38 % heavy. The second set is the hdfv runabout (6.3 m, chine beam 2 m, deadrise 26 deg at the bow to 19 at the
     * transom, 1226 kg, LCG 2.5 m, C_Delta 0.15) on its own curved bow, which reported the gap: 2.7, 1.8, 1.4, 1.0 deg
     * against Savitsky's 3.7, 2.9, 2.4, 1.9 at 9, 12.4, 14.6 and 18 m/s, and up to 1.7 times his resistance.
     */
    @Test void hullsPlaneAsSavitskyPredicts() {
        double beam = 2, beta = 20, rho = WATER.densityKgPerM3();
        PlaningHull prism = PlaningHull.fridsma(8 * beam, beam, Math.toRadians(beta), 0.6 * beam, 64);
        // {C_Delta, LCG in beams, C_V}
        double[][] points = {{0.15, 1.25, 2}, {0.15, 1.25, 3}, {0.15, 1.6, 2}, {0.15, 1.6, 3}, {0.3, 1.6, 2}, {0.3, 1.6, 3}, {0.3, 1.6, 4},
                {0.3, 1.6, 5}, {0.45, 1.6, 2}, {0.45, 1.6, 3}, {0.45, 1.6, 4}, {0.45, 1.6, 5}, {0.6, 2, 3}, {0.6, 2, 4}, {0.6, 2, 5}};
        dev.mcmod.aerocore.api.body.SectionShape bow = new dev.mcmod.aerocore.api.body.SectionShape.Chined(2, 1, 0, 26, 8),
                cockpit = new dev.mcmod.aerocore.api.body.SectionShape.Chined(2, 1, 0, 21, 8), stern = new dev.mcmod.aerocore.api.body.SectionShape.Chined(2, 1, 0, 19, 8);
        PlaningHull runabout = new dev.mcmod.aerocore.api.body.BodyShape(bow.scaled(0.05), List.of(
                new dev.mcmod.aerocore.api.body.BodyModule("bow", 1.8, bow, 2, 0, Double.POSITIVE_INFINITY),
                new dev.mcmod.aerocore.api.body.BodyModule("cockpit", 2.8, cockpit, 1, 0, 0.75),
                dev.mcmod.aerocore.api.body.BodyModule.straight("stern", 1.7, stern))).planingHull();
        double[] runaboutSpeeds = {9, 12.4, 14.6, 18};
        List<String> lines = java.util.stream.IntStream.range(0, points.length + runaboutSpeeds.length).parallel().mapToObj(k -> {
            boolean boat = k >= points.length;
            double mass, lcg, vcg, speed, deadrise;
            Run run;
            if (boat) {
                speed = runaboutSpeeds[k - points.length]; mass = 1226; lcg = 2.5; vcg = 0.55; deadrise = 19;
                run = towSI(runabout, mass, lcg, vcg, 0.25 * 6.3, speed);
            } else {
                double[] c = points[k];
                mass = c[0] * rho * beam * beam * beam; lcg = c[1] * beam; vcg = 0.3 * beam; speed = c[2] * Math.sqrt(G * beam); deadrise = beta;
                run = towSI(prism, mass, lcg, vcg, 2.5, speed);
            }
            Savitsky s = savitsky(beam, deadrise, mass, lcg, vcg, speed);
            double resistance = run.resistanceRatio() * mass * G, error = resistance / s.withSprayN() - 1;
            String line = String.format(Locale.ROOT, "%s C_Delta %.2f LCG %.2fB C_V %.2f: trim %.2f (Savitsky %.2f), R %.0f N (Savitsky %.0f, with spray %.0f: %+.0f %%), lambda %.2f, swing %.2f",
                    boat ? "runabout" : "prism", mass / (rho * beam * beam * beam), lcg / beam, speed / Math.sqrt(G * beam), run.trimDeg(), s.trimDeg(), resistance,
                    s.resistanceN(), s.withSprayN(), 100 * error, s.lambda(), run.trimRangeDeg());
            boolean pass = Math.abs(run.trimDeg() - s.trimDeg()) <= 0.7 && Math.abs(error) <= 0.15 && run.trimRangeDeg() < 0.3;
            if (!boat) { pass &= s.trimDeg() >= 2 && s.lambda() <= 4 && s.chineBeams() >= 0; }
            return (pass ? "  " : "! ") + line;
        }).toList();
        String report = String.join("\n", lines);
        System.out.println(report);
        assertTrue(lines.stream().allMatch(l -> l.startsWith("  ")), report);
    }

    /**
     * The whisker spray sheet reaches the chines only where they are wet ahead of the transom; when they are dry there it
     * ends at the transom (2026-10-12). Sinking a planing hull by a millimetre through the attitude where the chines wet just at
     * the transom changes the friction and spray by little (0.23 lost the spray there, a fifth of the resistance).
     */
    @Test void whiskerSprayIsContinuousWhereTheChinesWetAtTheTransom() {
        double beam = 2, beta = Math.toRadians(20), trim = Math.toRadians(2), speed = 16;
        PlaningHull prism = PlaningHull.fridsma(8 * beam, beam, beta, 0.6 * beam, 64);
        RigidBodyProperties p = new RigidBodyProperties(1200, new Vec3d(2.5, 0.6, 0), new InertiaTensor(1000, 7500, 7500, 0, 0, 0));
        HullDynamics.Environment calm = point -> HullDynamics.Water.calm(0, WATER, AIR);
        Orientation q = Orientation.rotationVector(Vec3d.Z.scale(trim));
        // the chines wet at the transom when Wagner's half-beam (pi/2) h cot(beta) reaches B/2 there
        double wetAtTransom = 0.5 * beam * Math.tan(beta) / (0.5 * Math.PI) * Math.cos(trim);
        double[] friction = new double[2];
        for (int k = 0; k < 2; k++) {
            double depth = wetAtTransom + (k == 0 ? -0.001 : 0.001);
            Vec3d transom = new Vec3d(0, -depth, 0);
            RigidBodyState s = new RigidBodyState(transom.add(q.rotate(p.centerOfMassLocal())), new Vec3d(speed, 0, 0), q, Vec3d.ZERO);
            friction[k] = PlaningDynamics.loads(s, p, prism, calm, Vec3d.Y.scale(-G), false).diagnostics().frictionN();
        }
        assertEquals(friction[0], friction[1], 0.03 * friction[1], "friction and spray just dry / just wet at the transom: " + friction[0] + " / " + friction[1]);
    }

    @Test void theDeadriseFactorRecoversAlongTheWettedChines() {
        double beam = 2;
        assertEquals(1 - Math.sin(Math.toRadians(20)), PlaningDynamics.liftFactor(Math.toRadians(20), 0, beam), 1e-12);
        assertEquals(1, PlaningDynamics.liftFactor(Math.toRadians(20), 3 * beam, beam), 1e-12);
        assertEquals(PlaningDynamics.MIN_LIFT_FACTOR, PlaningDynamics.liftFactor(Math.toRadians(50), -2 * beam, beam), 1e-12);
        // a flat bottom keeps the 2D+t lift
        assertEquals(1, PlaningDynamics.liftFactor(0, 0, beam), 1e-12);
    }

    @Test void ventilationBlendLimits() {
        assertEquals(0, PlaningDynamics.ventilation(0.5));
        assertEquals(1, PlaningDynamics.ventilation(1.5));
        assertEquals(0.5, PlaningDynamics.ventilation(1.0), 1e-12);
    }
}

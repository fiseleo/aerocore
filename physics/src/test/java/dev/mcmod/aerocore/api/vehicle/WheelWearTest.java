package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.damage.Wear;
import dev.mcmod.aerocore.api.ground.BoxGround;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AxlePart;
import dev.mcmod.aerocore.api.vehicle.part.LegPart;
import dev.mcmod.aerocore.api.wheel.Brake;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 0.42: axles and landing-gear legs that wear (WheelCorner.Wearing): new they step as ones that do not, to the last bit;
 * tread, linings and bump stops wear by the work the formulas say; worn tyres grip less, worn-out linings brake less,
 * blown dampers damp less, a collapsed strut carries a tenth; nothing breaks with everything used up.
 */
class WheelWearTest {
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final double G = TestTrack.G, DT = TestTrack.DT;
    private static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC), WET = Ground.flat(0, GroundContact.Surface.WET_TARMAC);
    private static final Tyre TYRE = TestParts.carTyre();
    private static final double BODY = 1200, AXLE = 100, FRONT = 1.1, REAR = 1.6, TRACK = 1.55;
    private static final double INF = Double.POSITIVE_INFINITY;
    /** Wears nothing by itself (boundless volumes and stop rating): for setting a condition and looking at what it does. */
    private static final WheelCorner.Wearing STILL = new WheelCorner.Wearing(INF, Wear.TREAD_M3_PER_J, 3, INF, Wear.BRAKE_LINING_M3_PER_J, 523.15, 60, INF, 0.5);
    /** A car's: two tyres' tread of 6.4 mm at a land ratio of 0.7, four pads of 50 cm^2 and 10 mm, stops made for 8 kN. */
    private static final WheelCorner.Wearing TYPICAL = WheelCorner.Wearing.typical(2 * WheelCorner.Wearing.treadVolumeM3(TYRE, 0.0064, 0.7),
            WheelCorner.Wearing.liningVolumeM3(0.005, 0.01, 4), 8000);

    private static double mass(boolean engine) { return BODY + 2 * AXLE + (engine ? 10 : 0); }
    private static double comX(boolean engine) { return (AXLE * FRONT - AXLE * REAR + (engine ? 10 * FRONT : 0)) / mass(engine); }
    private static double corner(boolean front, boolean engine) {
        double share = (comX(engine) + REAR) / (FRONT + REAR);
        return mass(engine) * G * (front ? share : 1 - share) / 2;
    }
    private static double hub(boolean front, boolean engine) { return TYRE.radiusM() - TYRE.deflectionM(corner(front, engine)); }
    private static double rate(double load, double hz) {
        double ride = load / G * Math.pow(2 * Math.PI * hz, 2), kt = TYRE.verticalStiffness(TYRE.deflectionM(load));
        return ride * kt / (kt - ride);
    }

    /**
     * A car as WheelTest's (1.3 Hz, a damping ratio at the wheel, double wishbones, braked on "brake", steered on "steer"),
     * its rear axle driven by an "engine" on "torque" when driven; each axle wearing or not (null).
     */
    private static Vehicle car(String name, boolean driven, double zeta, WheelCorner.Wearing front, WheelCorner.Wearing rear) {
        double f = corner(true, driven), r = corner(false, driven), kf = rate(f, 1.3), kr = rate(r, 1.3);
        double cf = 2 * zeta * Math.sqrt(kf * f / G), cr = 2 * zeta * Math.sqrt(kr * r / G);
        AxlePart.Drive drive = driven ? new AxlePart.Drive(3.5, 1, 0.05, AxlePart.Differential.OPEN, 30, 0.3, 3, 50, false, false) : null;
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4.2, 1.0, 1.7), 0, false, 1, MassSpec.given(BODY)), Mount.at(new Vec3d(0, 0.6, 0))),
                new VehicleBlueprint.Entry("front", TestParts.axle("f", TRACK, kf, f, cf, cf, Suspension.Type.DOUBLE_WISHBONE, true, null,
                        new AxlePart.Steering(0.6, 0.5, FRONT + REAR), 0, 0, null, front), Mount.at(new Vec3d(FRONT, hub(true, driven), 0)),
                        List.of(Binding.of("steer", "steer"), Binding.of("brake", "brake"))),
                new VehicleBlueprint.Entry("rear", TestParts.axle("r", TRACK, kr, r, cr, cr, Suspension.Type.DOUBLE_WISHBONE, true, drive, null, 0, 1200, null, rear),
                        Mount.at(new Vec3d(-REAR, hub(false, driven), 0)), List.of(Binding.of("brake", "brake"), Binding.of("handbrake", "handbrake")))));
        if (driven) { parts.add(new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.15, true), Mount.at(new Vec3d(FRONT, 0.45, 0)), List.of(Binding.of("torque", "torque")))); }
        List<Connection> shafts = driven ? List.of(Connection.Shaft.ports("engine", null, "rear", "input", 1, 1)) : List.of();
        return new VehicleBlueprint(name, parts, shafts).compile();
    }

    private static VehicleState parked(Vehicle v, Ground ground) { return TestTrack.settle(v, ground, AIR, Vec3d.ZERO, 0, Map.of(), 2); }

    /** Sets the car moving at a speed with its wheels (and a driven axle's input) rolling with it. */
    private static void moving(VehicleState st, double speed) {
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(speed, 0, 0), b.orientation(), Vec3d.ZERO));
        for (Vehicle.Installed i : st.vehicle().installed()) {
            if (i.part() instanceof AxlePart a) {
                int first = i.firstRotor() + (a.drive() == null ? 0 : 1);
                double re = TYRE.radiusM() - TYRE.deflectionM(4000) / 3;
                for (int k = 0; k < 2; k++) {
                    Vehicle.Rotor r = st.vehicle().rotors().get(first + k);
                    st.setTrainSpeed(r.train(), speed / re / r.ratio());
                }
                if (a.drive() != null) {
                    Vehicle.Rotor input = st.vehicle().rotors().get(i.firstRotor());
                    st.setTrainSpeed(input.train(), a.drive().finalRatio() * speed / re / input.ratio());
                }
            }
        }
    }

    /** Sets the car moving at a speed with its wheels standing (locked: sliding). */
    private static void sliding(VehicleState st, double speed) {
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(speed, 0, 0), b.orientation(), Vec3d.ZERO));
    }

    private static void step(Vehicle v, VehicleState st, Ground ground, Map<String, Double> controls, int i) {
        VehicleDynamics.step(v, st, VehicleEnvironment.land(ground, AIR), controls, Vec3d.Y.scale(-G), List.of(), null, i * DT, DT);
    }

    private static void finite(VehicleState st) {
        RigidBodyState b = st.body();
        for (Vec3d x : new Vec3d[]{b.position(), b.velocity(), b.angularMomentum()}) {
            assertTrue(Double.isFinite(x.x()) && Double.isFinite(x.y()) && Double.isFinite(x.z()), "finite: " + x);
        }
        for (double x : st.internals) { assertTrue(Double.isFinite(x)); }
    }

    // ----- new: as one that does not wear -----

    @Test void anAxleThatWearsStepsAsOneThatDoesNotWhileNew() {
        // over a kerb, driving off, steering, braking hard and on the handbrake: every path of the wheels
        Ground track = Ground.nearest(FLAT, Ground.boxes(List.of(new BoxGround.Box(new Vec3d(5, 0, -3), new Vec3d(9, 0.06, 3), GroundContact.Surface.WET_TARMAC))));
        Vehicle plain = car("plain", true, 0.3, null, null), wearing = car("wearing", true, 0.3, TYPICAL, TYPICAL);
        assertEquals(List.of(), plain.installed("front").part().conditions());
        assertEquals(List.of("tyres", "brakes", "suspension"), wearing.installed("front").part().conditions());
        assertEquals(List.of(AxlePart.TYRES, AxlePart.BRAKES, AxlePart.SUSPENSION), wearing.installed("rear").part().conditions());
        VehicleState a = parked(plain, track), b = parked(wearing, track);
        b.setUsedValues(new double[6]);
        moving(a, 6);
        moving(b, 6);
        double[] most = new double[6];
        for (int i = 0; i < 500; i++) {
            double t = i * DT;
            Map<String, Double> controls = Map.of("torque", t < 1.2 ? 250.0 : 0.0, "steer", 0.4 * Math.sin(3 * t), "brake", t > 1.5 && t < 2.1 ? 1.0 : 0.0,
                    "handbrake", t > 2.1 ? 1.0 : 0.0);
            step(plain, a, track, controls, i);
            step(wearing, b, track, controls, i);
            double[] used = b.usedValues();
            for (int k = 0; k < 6; k++) { most[k] = Math.max(most[k], used[k]); }
            // what it wore in the step is taken away again: it stays new
            b.setUsedValues(new double[6]);
            if (i % 25 == 24) { assertEquals(Json.write(a.toJson()), Json.write(b.toJson()), "step " + i); }
        }
        assertEquals(Json.write(a.toJson()), Json.write(b.toJson()));
        assertTrue(a.origin().x() > 8, "it went over the kerb: " + a.origin());
        // and it did wear in those steps: tyres and brakes of both axles
        for (int k : new int[]{0, 1, 3, 4}) { assertTrue(most[k] > 0, "condition " + k + " wore"); }
        // the diagnostics of the wearing axle tell its condition; the plain one has nothing new
        assertTrue(b.diagnostics().get("front").containsKey("tyresUsed"));
        assertTrue(!a.diagnostics().get("front").containsKey("tyresUsed") && !a.diagnostics().get("front").containsKey("brakesUsed"));
    }

    // ----- wear by the work done -----

    /**
     * Locked wheels turn all the car's motion into heat at the tyres: sum F v dt = m V^2 / 2 whatever the friction, and
     * with the severity 1 + (v / v0)^2 the tread loses A m (V^2 / 2 + V^4 / (4 v0^2)) (F = -m dv/dt, so the integral of
     * F v^3 dt is m V^4 / 4).
     */
    @Test void lockedWheelsWearTheirTreadByTheStopsEnergy() {
        double volume = 1e-3, v0 = 3, speed = 20;
        WheelCorner.Wearing w = new WheelCorner.Wearing(volume, Wear.TREAD_M3_PER_J, v0, INF, 0, 523.15, 60, INF, 0.5);
        Vehicle v = car("locked", false, 0.3, w, w);
        VehicleState st = parked(v, FLAT);
        // sliding from the first step: the wheels stand (spinning down from rolling, part of the energy would go to the brakes)
        sliding(st, speed);
        TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 1.0), 10, s -> s.body().velocity().length() < 0.02, Integer.MAX_VALUE);
        double worn = (st.used("front", AxlePart.TYRES) + st.used("rear", AxlePart.TYRES)) * volume;
        double expected = Wear.TREAD_M3_PER_J * mass(false) * (speed * speed / 2 + Math.pow(speed, 4) / (4 * v0 * v0));
        assertEquals(expected, worn, 0.03 * expected, "tread rubber worn in a locked stop from 20 m/s, m^3");
        assertTrue(st.used("front", AxlePart.TYRES) > st.used("rear", AxlePart.TYRES), "the loaded front tyres take more of it");
        assertEquals(0, st.used("front", AxlePart.BRAKES), 0, "nothing else wore");
        assertEquals(0, st.used("front", AxlePart.SUSPENSION), 0);
        // a flat spot: one such stop takes a twentieth of this tread; the same energy at gentle slip would take a 23rd of that
        assertEquals(1 + speed * speed / (2 * v0 * v0), expected / (Wear.TREAD_M3_PER_J * mass(false) * speed * speed / 2), 1e-9);
    }

    /**
     * A stop on the brakes (no wheel locks): the car's kinetic energy and its wheels' goes into the linings but for what
     * the tyres' slip and their rolling resistance take; the linings wear by exactly their share.
     */
    @Test void aBrakingStopWearsTheLiningsByItsEnergy() {
        double lining = 2e-4, tread = 1e-3, speed = 20;
        // severity 1 (a reference speed out of reach), so the tread's wear is the slip work itself
        WheelCorner.Wearing w = new WheelCorner.Wearing(tread, Wear.TREAD_M3_PER_J, 1e9, lining, Wear.BRAKE_LINING_M3_PER_J, 523.15, 60, INF, 0.5);
        Vehicle v = car("braked", false, 0.3, w, w);
        VehicleState st = parked(v, FLAT);
        moving(st, speed);
        double x0 = st.origin().x(), wheels = 0;
        for (Vehicle.Installed i : v.installed()) {
            if (i.part() instanceof AxlePart a) { for (int k = 0; k < 2; k++) { wheels += 0.5 * a.left().inertiaKgM2() * Math.pow(st.rotorSpeed(i.firstRotor() + k), 2); } }
        }
        TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 0.25), 20, s -> s.body().velocity().length() < 0.02, Integer.MAX_VALUE);
        double distance = st.origin().x() - x0;
        double brakes = (st.used("front", AxlePart.BRAKES) + st.used("rear", AxlePart.BRAKES)) * lining / Wear.BRAKE_LINING_M3_PER_J;
        double slip = (st.used("front", AxlePart.TYRES) + st.used("rear", AxlePart.TYRES)) * tread / Wear.TREAD_M3_PER_J;
        double rolling = 2 * (TYRE.rollingResistanceN(GroundContact.Surface.TARMAC, corner(true, false), speed / 2)
                + TYRE.rollingResistanceN(GroundContact.Surface.TARMAC, corner(false, false), speed / 2)) * distance;
        double kinetic = 0.5 * mass(false) * speed * speed + wheels;
        assertEquals(kinetic, brakes + slip + rolling, 0.02 * kinetic, "brakes " + brakes + " J, tyre slip " + slip + " J, rolling " + rolling + " J");
        assertTrue(brakes > 0.9 * kinetic, "nearly all of it at the brakes: " + brakes / kinetic);
        assertTrue(slip > 0.01 * kinetic && slip < 0.06 * kinetic, "a few percent at the tyres (slip of a few percent): " + slip / kinetic);
        // a lining that is past its knee from the start wears at least twice as fast (the rotors warm some 20 K in the stop)
        WheelCorner.Wearing hot = new WheelCorner.Wearing(tread, Wear.TREAD_M3_PER_J, 1e9, lining, Wear.BRAKE_LINING_M3_PER_J, WheelCorner.AIR_K - 60, 60, INF, 0.5);
        Vehicle h = car("braked hot", false, 0.3, hot, hot);
        VehicleState hs = parked(h, FLAT);
        moving(hs, speed);
        TestTrack.drive(h, hs, FLAT, AIR, (t, s) -> Map.of("brake", 0.25), 20, s -> s.body().velocity().length() < 0.02, Integer.MAX_VALUE);
        double ratio = hs.used("front", AxlePart.BRAKES) / st.used("front", AxlePart.BRAKES);
        assertTrue(ratio > 2 && ratio < 2.8, "past the knee by 60 to 85 K: " + ratio);
    }

    /** The car's mean deceleration (m/s^2) from a speed over the second after the first 0.3 s, with controls and the axles' conditions set. */
    private static double deceleration(Vehicle v, Ground ground, double speed, Map<String, Double> controls, int condition, double used) {
        VehicleState st = parked(v, ground);
        for (String axle : new String[]{"front", "rear"}) { st.setUsed(axle, condition, used); }
        moving(st, speed);
        TestTrack.drive(v, st, ground, AIR, (t, s) -> controls, 0.3, s -> false, Integer.MAX_VALUE);
        double before = st.body().velocity().x();
        TestTrack.drive(v, st, ground, AIR, (t, s) -> controls, 1.0, s -> false, Integer.MAX_VALUE);
        finite(st);
        return before - st.body().velocity().x();
    }

    /**
     * The pull of the car's locked wheels over what new tyres' would be at the same loads (sliding from a speed on a
     * surface, over a second): the share of their grip the tyres have. Wheel by wheel, since less grip also moves less
     * load forward, and a tyre's friction falls with its load.
     */
    private static double lockedGrip(Vehicle v, GroundContact.Surface surface, double speed, double used) {
        Ground ground = Ground.flat(0, surface);
        VehicleState st = parked(v, ground);
        for (String axle : new String[]{"front", "rear"}) { st.setUsed(axle, WheelCorner.TYRE, used); }
        sliding(st, speed);
        TestTrack.drive(v, st, ground, AIR, (t, s) -> Map.of("brake", 1.0), 0.3, s -> false, Integer.MAX_VALUE);
        double pull = 0, fresh = 0;
        for (TestTrack.Sample s : TestTrack.drive(v, st, ground, AIR, (t, x) -> Map.of("brake", 1.0), 1.0, x -> false, 1)) {
            for (String axle : new String[]{"front", "rear"}) {
                for (String side : new String[]{"left", "right"}) {
                    Map<String, Double> d = s.diagnostics().get(axle);
                    pull -= d.get(side + "ForceN");
                    fresh -= TYRE.forces(surface, d.get(side + "LoadN"), -1, 0, 0, s.speedMps()).longitudinalN();
                }
            }
        }
        finite(st);
        return pull / fresh;
    }

    @Test void wornTreadGripsLessMostlyInTheWetAndAFlatTyreDrags() {
        Vehicle v = car("grip", false, 0.3, STILL, STILL);
        GroundContact.Surface dry = GroundContact.Surface.TARMAC, wet = GroundContact.Surface.WET_TARMAC;
        // the locked wheels' pull is the friction's: on dry tarmac the tread hardly matters, in the wet it is most of it
        assertEquals(1.00, lockedGrip(v, dry, 20, 0), 0.01, "dry, new");
        assertEquals(0.95, lockedGrip(v, dry, 20, 0.5), 0.01, "dry, half worn");
        assertEquals(0.90, lockedGrip(v, dry, 20, 0.999), 0.01, "dry, at the end of the tread");
        assertEquals(1.00, lockedGrip(v, wet, 15, 0), 0.01, "wet, new");
        assertEquals(0.80, lockedGrip(v, wet, 15, 0.5), 0.01, "wet, half worn");
        assertEquals(0.60, lockedGrip(v, wet, 15, 0.999), 0.01, "wet, at the end of the tread");
        // through the carcass: flat
        assertEquals(0.40, lockedGrip(v, dry, 20, 1), 0.01, "dry, flat");
        assertEquals(0.40, lockedGrip(v, wet, 15, 1), 0.01, "wet, flat");
        // and the car stops accordingly
        Map<String, Double> locked = Map.of("brake", 1.0);
        double fresh = deceleration(v, FLAT, 20, locked, WheelCorner.TYRE, 0), half = deceleration(v, FLAT, 20, locked, WheelCorner.TYRE, 0.5);
        double bald = deceleration(v, FLAT, 20, locked, WheelCorner.TYRE, 0.999), burst = deceleration(v, FLAT, 20, locked, WheelCorner.TYRE, 1);
        assertTrue(fresh > half && half > bald && bald > burst, fresh + " > " + half + " > " + bald + " > " + burst + " m/s^2");
        assertEquals(0.90, bald / fresh, 0.03, "a bald tyre's dry stop");
        double wetFresh = deceleration(v, WET, 15, locked, WheelCorner.TYRE, 0), wetBald = deceleration(v, WET, 15, locked, WheelCorner.TYRE, 0.999);
        assertEquals(0.60, wetBald / wetFresh, 0.05, "a bald tyre's wet stop");
        assertTrue(burst / fresh > 0.38 && burst / fresh < 0.48, "flat (less load goes forward, and lightly loaded tyres grip better): " + burst / fresh);
        // coasting, a flat tyre's rolling resistance is several times an inflated one's
        double coast = deceleration(v, FLAT, 20, Map.of(), WheelCorner.TYRE, 0), flat = deceleration(v, FLAT, 20, Map.of(), WheelCorner.TYRE, 1);
        assertEquals(WheelCorner.FLAT_ROLLING, flat / coast, 0.1 * WheelCorner.FLAT_ROLLING, "coasting on flat tyres " + flat + " against " + coast + " m/s^2");
        // the rule itself
        assertEquals(1, WheelCorner.treadGrip(Tyre.Ground.WET, 0), 0);
        assertEquals(0.8, WheelCorner.treadGrip(Tyre.Ground.SNOW, 0.5), 1e-12);
        assertEquals(0.95, WheelCorner.treadGrip(Tyre.Ground.ICE, 0.5), 1e-12);
        assertEquals(0.4, WheelCorner.treadGrip(Tyre.Ground.LOOSE, 1), 0);
    }

    @Test void liningsBrakeAsNewUntilNearlyGoneThenMetalOnMetal() {
        Vehicle v = car("linings", false, 0.3, STILL, STILL);
        Map<String, Double> pedal = Map.of("brake", 0.25);
        double coast = deceleration(v, FLAT, 20, Map.of(), WheelCorner.BRAKE, 0), fresh = deceleration(v, FLAT, 20, pedal, WheelCorner.BRAKE, 0) - coast;
        assertTrue(fresh > 2.5, "a quarter of the pedal: " + fresh + " m/s^2");
        assertEquals(1.0, (deceleration(v, FLAT, 20, pedal, WheelCorner.BRAKE, 0.5) - coast) / fresh, 1e-9, "half worn: as new");
        assertEquals(1.0, (deceleration(v, FLAT, 20, pedal, WheelCorner.BRAKE, 0.8) - coast) / fresh, 1e-9, "at the knee: as new");
        assertEquals(0.65, (deceleration(v, FLAT, 20, pedal, WheelCorner.BRAKE, 0.9) - coast) / fresh, 0.02, "past it");
        assertEquals(0.30, (deceleration(v, FLAT, 20, pedal, WheelCorner.BRAKE, 1) - coast) / fresh, 0.02, "the backing plates");
        assertEquals(1, WheelCorner.liningShare(0.8), 0);
        assertEquals(0.3, WheelCorner.liningShare(1), 1e-12);
    }

    /**
     * The damping ratio of a car's heave after a push down (the first two troughs' decrement): WheelTest's symmetric car
     * on trailing arms, so heave is a mode of its own and nothing but the dampers damps it.
     */
    private static double heaveDamping(double used) {
        double load = mass(false) * G / 4, k = rate(load, 1.3), c = 2 * 0.25 * Math.sqrt(k * load / G), height = TYRE.radiusM() - TYRE.deflectionM(load);
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4.2, 1.0, 1.7), 0, false, 1, MassSpec.given(BODY)), Mount.at(new Vec3d(0, 0.6, 0)))));
        for (String id : new String[]{"front", "rear"}) {
            parts.add(new VehicleBlueprint.Entry(id, TestParts.axle(id, TRACK, k, load, c, c, Suspension.Type.TRAILING_ARM, true, null, null, 0, 0, null, STILL),
                    Mount.at(new Vec3d(id.equals("front") ? 1.35 : -1.35, height, 0))));
        }
        Vehicle v = new VehicleBlueprint("ride", parts, List.of()).compile();
        VehicleState st = parked(v, FLAT);
        for (String axle : new String[]{"front", "rear"}) { st.setUsed(axle, WheelCorner.SUSPENSION, used); }
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(0, -0.25, 0), b.orientation(), Vec3d.ZERO));
        List<TestTrack.Sample> trace = TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of(), 12, s -> false, 1);
        finite(st);
        // the body's centre of mass: its height's troughs
        List<Double> troughs = new ArrayList<>();
        double settled = trace.get(trace.size() - 1).origin().y();
        for (int i = 1; i < trace.size() - 1 && troughs.size() < 2; i++) {
            double y0 = trace.get(i - 1).origin().y(), y1 = trace.get(i).origin().y(), y2 = trace.get(i + 1).origin().y();
            if (y1 < y0 && y1 <= y2 && settled - y1 > 1e-5) { troughs.add(settled - y1); }
        }
        assertEquals(2, troughs.size(), "it bounces");
        double decrement = Math.log(troughs.get(0) / troughs.get(1));
        return decrement / Math.sqrt(4 * Math.PI * Math.PI + decrement * decrement);
    }

    @Test void blownDampersLetTheCarBounce() {
        double fresh = heaveDamping(0), half = heaveDamping(0.5), blown = heaveDamping(1);
        assertTrue(fresh > 0.15 && fresh < 0.35, "a quarter of critical at the wheels: " + fresh);
        assertEquals(0.6, WheelCorner.damperShare(0.5), 1e-12);
        assertEquals(0.2, WheelCorner.damperShare(1), 1e-12);
        // a corner's linear model (WheelTest's: tyre in series with spring and damper) with 0.6 and 0.2 of the damper. The
        // step's own damping (backward Euler: about omega dt / 2 = 0.02 of critical at 1.3 Hz) is on top of every one of
        // them, so the model is compared by what the damper's loss takes away
        double load = mass(false) * G / 4, m = load / G, kt = TYRE.verticalStiffness(TYRE.deflectionM(load)), kw = rate(load, 1.3), damper = 2 * 0.25 * Math.sqrt(kw * load / G);
        double model = modeDamping(m, damper, kt, kw), lostHalf = model - modeDamping(m, 0.6 * damper, kt, kw), lostBlown = model - modeDamping(m, 0.2 * damper, kt, kw);
        assertEquals(lostHalf, fresh - half, 0.1 * lostHalf, "half used: 0.6 of the damper (of " + model + ")");
        assertEquals(lostBlown, fresh - blown, 0.1 * lostBlown, "blown: a fifth of the damper");
        assertEquals(0.2, modeDamping(m, 0.2 * damper, kt, kw) / model, 0.01, "the mode's damping goes as the damper's here");
        assertTrue(blown / fresh > 0.2 && blown / fresh < 0.32, "about a fifth of the damping, and the step's: " + blown / fresh);
        // the step itself: a strut with no spring pushed at a steady rate carries its damper's force, times the share
        Suspension strut = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.coil(0, 0), Suspension.Damper.linear(4000, 4000), 2.0, 0, 0);
        for (double share : new double[]{1, 0.6, 0.2}) {
            double c = 0, own = 0, force = 0, rate = 0.5;
            for (int i = 0; i * DT < 0.6; i++) {
                Suspension.Step s = strut.step(TYRE, rate * i * DT, rate, c, own, 0, DT, 1, share, 1);
                double next = s.compression(rate, DT);
                own = (next - c) / DT;
                c = next;
                force = s.forceN() + s.slopeNsPerM() * rate;
            }
            assertEquals(share * 4000 * rate, force, 0.01 * share * 4000 * rate, "damper share " + share);
        }
        // and a spring's share scales the spring and its stop alike
        Suspension coil = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.coil(50000, 1000), Suspension.Damper.linear(4000, 4000), 0.2, 0, 0.1);
        Suspension.Step whole = coil.step(TYRE, 0.26, 0, 0.25, 0, 0, DT, 1), tenth = coil.step(TYRE, 0.26, 0, 0.25, 0, 0, DT, 1, 1, 0.1);
        Suspension.Step same = coil.step(TYRE, 0.26, 0, 0.25, 0, 0, DT, 1, 1, 1);
        assertEquals(whole, same, "shares of 1 are the step itself");
        assertEquals(0.1 * whole.springForce(), tenth.springForce(), 1e-9 * whole.springForce());
        assertEquals(0.1 * whole.suspensionRate(), tenth.suspensionRate(), 1e-9 * whole.suspensionRate());
        assertEquals(coil.force(0.25) - coil.spring().force(0.2), coil.stopForce(0.25, 1), 1e-9, "the stop's own force");
    }

    /** The damping ratio of the ride mode of m s^3 c + m (k_t + k_w) s^2 + k_t c s + k_t k_w = 0 (Newton from the undamped root). */
    private static double modeDamping(double m, double c, double kt, double kw) {
        double a = m * c, b = m * (kt + kw), cc = kt * c, d = kt * kw, re = 0, im = Math.sqrt(d / b);
        for (int i = 0; i < 100; i++) {
            double s2r = re * re - im * im, s2i = 2 * re * im, s3r = s2r * re - s2i * im, s3i = s2r * im + s2i * re;
            double pr = a * s3r + b * s2r + cc * re + d, pi = a * s3i + b * s2i + cc * im;
            double dr = 3 * a * s2r + 2 * b * re + cc, di = 3 * a * s2i + 2 * b * im;
            double den = dr * dr + di * di;
            re -= (pr * dr + pi * di) / den;
            im -= (pi * dr - pr * di) / den;
        }
        return -re / Math.hypot(re, im);
    }

    // ----- a leg -----

    private static final double LEG_RATE = 50000, LEG_TRAVEL = 0.2;
    private static final Tyre AIRCRAFT = new Tyre(new Tyre.Spec(Tyre.Construction.AIRCRAFT, 0.16, 0.91, 0.1524, 290e3, 290e3, Tyre.Tread.AIRCRAFT_RIB, 1));

    /** A leg on a coil strut (50 kN/m, 0.2 m, no preload), braked, fixed, castering or steered; wearing or not. */
    private static PartDesign leg(String name, LegPart.Steer steer, double side, WheelCorner.Wearing wearing, LegPart.Retraction retraction) {
        Suspension strut = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.coil(LEG_RATE, 0), Suspension.Damper.linear(3000, 6000), LEG_TRAVEL, 0, 0.1);
        WheelCorner wheel = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, 0, 0), side, 0, AIRCRAFT, 0.05, strut,
                new Brake(Brake.Type.DISC, 600, 2, 0.15, 0.03), 0));
        return new TestParts.Fixture(name, "wear leg " + steer + " " + side + " " + wearing + " " + retraction, d -> {
            LegPart leg = new LegPart(d, wheel, steer, 0.6, 0.08, null, MassProperties.point(15, Vec3d.ZERO), List.of(), retraction);
            return wearing == null ? leg : leg.wearing(wearing);
        });
    }

    /** One leg under a mass that puts a load on it (N), let down and settled. */
    private static VehicleState onOneLeg(Vehicle[] out, double loadN, WheelCorner.Wearing wearing, double strutUsed, double seconds) {
        Vehicle v = new VehicleBlueprint("one leg", List.of(new VehicleBlueprint.Entry("leg", leg("leg", LegPart.Steer.FIXED, 0, wearing, null), Mount.ORIGIN),
                new VehicleBlueprint.Entry("mass", TestParts.box("m", new Vec3d(0.5, 0.5, 0.5), 0, false, 1, MassSpec.given(loadN / G - 15)), Mount.at(new Vec3d(0, 0.5, 0)))),
                List.of()).compile();
        out[0] = v;
        VehicleState st = v.newState(new Vec3d(0, AIRCRAFT.radiusM() - 0.1, 0), Orientation.IDENTITY, DT);
        if (strutUsed > 0) { st.setUsed("leg", WheelCorner.SUSPENSION, strutUsed); }
        // on its stop the strut hardly moves and the mass rings on the tyre: stilled now and then for the first half
        int steps = (int) Math.round(seconds / DT);
        for (int i = 0; i < steps; i++) {
            if (i < steps / 2 && i % 20 == 0) { st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO)); }
            upright(v, st, i);
        }
        return st;
    }

    /** A step of the one-legged rig held upright (it would fall over: only its height is free). */
    private static void upright(Vehicle v, VehicleState st, int i) {
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(new Vec3d(0, b.position().y(), 0), new Vec3d(0, b.velocity().y(), 0), Orientation.IDENTITY, Vec3d.ZERO));
        step(v, st, FLAT, Map.of(), i);
    }

    @Test void aStrutPastItsStopsRatingIsDamagedAndACollapsedOneCarriesATenth() {
        Vehicle[] v = new Vehicle[1];
        // 13 kN on a strut whose spring gives 10 kN at full travel: the stop carries 3 kN, half as much again as it is made for
        WheelCorner.Wearing w = new WheelCorner.Wearing(INF, 0, 3, INF, 0, 523.15, 60, 2000, 100);
        VehicleState st = onOneLeg(v, 13000, w, 0, 6);
        assertEquals(LEG_TRAVEL + 3000 / (Suspension.BUMP_STOP * LEG_RATE), st.internal("leg", 0), 2e-4,
                "resting on its stop " + st.diagnostics().get("leg") + " " + st.body().position() + " " + st.body().velocity() + " mass " + v[0].properties().massKg());
        double before = st.used("leg", LegPart.STRUT);
        for (int i = 0; i < 400; i++) { upright(v[0], st, i); }
        assertEquals((3000.0 / 2000 - 1) * 2 / 100, st.used("leg", LegPart.STRUT) - before, 0.02 * 0.01, "(stop force / rated - 1) dt / life over 2 s");
        // within its rating nothing: 11.5 kN leaves the stop 1.5 kN
        assertEquals(0, onOneLeg(v, 11500, w, 0, 6).used("leg", LegPart.STRUT), 0);
        // 5 kN: new and half used it stands at 0.1 m; collapsed it sinks until a tenth of spring and stop carry it
        WheelCorner.Wearing still = STILL;
        double fresh = onOneLeg(v, 5000, still, 0, 6).body().position().y(), half = onOneLeg(v, 5000, still, 0.5, 6).body().position().y();
        VehicleState down = onOneLeg(v, 5000, still, 1, 6);
        finite(down);
        double sunk = LEG_TRAVEL + (5000 / WheelCorner.COLLAPSED_SPRING - LEG_RATE * LEG_TRAVEL) / (Suspension.BUMP_STOP * LEG_RATE);
        assertEquals(5000 / LEG_RATE, onOneLeg(v, 5000, still, 0, 6).internal("leg", 0), 2e-4, "the spring alone");
        assertEquals(fresh, half, 1e-4, "a damaged strut stands as high (its damping only)");
        assertEquals(sunk, down.internal("leg", 0), 5e-4, "collapsed: " + sunk + " m");
        assertEquals(fresh - (sunk - 5000 / LEG_RATE), down.body().position().y(), 1e-3, "and the aircraft sits that much lower");
        assertTrue(Math.abs(down.body().velocity().y()) < 1e-3, "at rest");
    }

    /** A tricycle: two braked main legs behind the centre of mass, a castering nose leg, 900 kg. */
    private static Vehicle tricycle(String name, WheelCorner.Wearing wearing, boolean retractable) {
        LegPart.Retraction r = retractable ? new LegPart.Retraction(1.5, Vec3d.Z, 1.5, 0.08, 0.8) : null;
        List<Binding> nose = new ArrayList<>(), left = new ArrayList<>(List.of(Binding.of("brake", "left_brake"))), right = new ArrayList<>(List.of(Binding.of("brake", "right_brake")));
        if (retractable) { for (List<Binding> b : List.of(nose, left, right)) { b.add(Binding.of("retract", "gear")); } }
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(5, 1, 1), 0, false, 1, MassSpec.given(855)), Mount.at(new Vec3d(0, 0.9, 0))),
                new VehicleBlueprint.Entry("nose", leg("n", LegPart.Steer.CASTERING, 0, wearing, r), Mount.at(new Vec3d(1.8, 0, 0)), nose),
                new VehicleBlueprint.Entry("left", leg("l", LegPart.Steer.FIXED, -1, wearing, r), Mount.at(new Vec3d(-0.4, 0, -1.2)), left),
                new VehicleBlueprint.Entry("right", leg("r", LegPart.Steer.FIXED, 1, wearing, r), Mount.at(new Vec3d(-0.4, 0, 1.2)), right)),
                List.of()).compile();
    }

    @Test void aLegThatWearsStepsAsOneThatDoesNotWhileNew() {
        WheelCorner.Wearing w = WheelCorner.Wearing.typical(WheelCorner.Wearing.treadVolumeM3(AIRCRAFT, 0.004, 0.8), WheelCorner.Wearing.liningVolumeM3(0.002, 0.008, 2), 6000);
        for (boolean retractable : new boolean[]{false, true}) {
            Vehicle plain = tricycle("plain " + retractable, null, retractable), wearing = tricycle("wearing " + retractable, w, retractable);
            assertEquals(List.of(), plain.installed("left").part().conditions());
            assertEquals(List.of(LegPart.TYRES, LegPart.BRAKES, LegPart.STRUT), wearing.installed("left").part().conditions());
            // a fixed leg's inputs and internal states are those it always had (0.30)
            assertEquals(plain.installed("left").part().internalStates(), wearing.installed("left").part().internalStates());
            assertEquals(plain.installed("nose").part().inputs(), wearing.installed("nose").part().inputs());
            // a landing with drift: touching down at 30 m/s and 2 m/s sink, crabbed, braking unevenly, then the gear told to come up
            VehicleState a = plain.newState(new Vec3d(0, AIRCRAFT.radiusM() + 0.15, 0), Orientation.IDENTITY, DT), b = wearing.newState(a.origin(), Orientation.IDENTITY, DT);
            for (VehicleState s : new VehicleState[]{a, b}) { s.setBody(new RigidBodyState(s.body().position(), new Vec3d(30, -2, 1.5), Orientation.IDENTITY, Vec3d.ZERO)); }
            double[] most = new double[9];
            for (int i = 0; i < 450; i++) {
                double t = i * DT;
                Map<String, Double> controls = Map.of("left_brake", t > 0.5 ? 0.8 : 0.0, "right_brake", t > 0.7 ? 0.5 : 0.0, "gear", t > 1.6 ? 1.0 : 0.0);
                step(plain, a, FLAT, controls, i);
                step(wearing, b, FLAT, controls, i);
                double[] used = b.usedValues();
                for (int k = 0; k < 9; k++) { most[k] = Math.max(most[k], used[k]); }
                b.setUsedValues(new double[9]);
                if (i % 25 == 24) { assertEquals(Json.write(a.toJson()), Json.write(b.toJson()), "step " + i); }
            }
            assertEquals(Json.write(a.toJson()), Json.write(b.toJson()));
            int left = wearing.firstCondition(wearing.installed("left").index());
            assertTrue(most[left] > 0 && most[left + 1] > 0, "the braked main wheel's tyre and brake wore");
        }
    }

    // ----- everything used up -----

    @Test void nothingBreaksWithEverythingUsedUp() {
        Ground track = Ground.nearest(Ground.split(0, GroundContact.Surface.WET_TARMAC, GroundContact.Surface.GRAVEL),
                Ground.boxes(List.of(new BoxGround.Box(new Vec3d(8, 0, -3), new Vec3d(12, 0.08, 3), GroundContact.Surface.ICE))));
        Vehicle v = car("used up", true, 0.3, TYPICAL, TYPICAL);
        VehicleState st = parked(v, track);
        double[] all = new double[v.conditions()];
        java.util.Arrays.fill(all, 1);
        st.setUsedValues(all);
        moving(st, 12);
        for (int i = 0; i < 1200; i++) {
            double t = i * DT;
            step(v, st, track, Map.of("torque", t < 2 ? 300.0 : 0.0, "steer", 0.8 * Math.sin(2 * t), "brake", t > 3 && t < 5 ? 1.0 : 0.0, "handbrake", t > 5 ? 1.0 : 0.0), i);
            finite(st);
        }
        assertEquals(1, st.worstUsed(), 0);
        assertTrue(st.body().position().y() < 2 && st.body().position().y() > 0, "still on the road: " + st.body().position());
        // an aircraft landing on legs used up: struts collapsed, tyres flat, linings gone
        WheelCorner.Wearing w = WheelCorner.Wearing.typical(1e-4, 1e-5, 6000);
        for (boolean retractable : new boolean[]{false, true}) {
            Vehicle plane = tricycle("used up " + retractable, w, retractable);
            VehicleState ps = plane.newState(new Vec3d(0, AIRCRAFT.radiusM() + 0.15, 0), Orientation.IDENTITY, DT);
            double[] ones = new double[plane.conditions()];
            java.util.Arrays.fill(ones, 1);
            ps.setUsedValues(ones);
            ps.setBody(new RigidBodyState(ps.body().position(), new Vec3d(30, -2, 1.5), Orientation.IDENTITY, Vec3d.ZERO));
            for (int i = 0; i < 1000; i++) {
                double t = i * DT;
                step(plane, ps, FLAT, Map.of("left_brake", t > 0.5 ? 1.0 : 0.0, "right_brake", t > 0.7 ? 0.6 : 0.0, "gear", t > 3 ? 1.0 : 0.0), i);
                finite(ps);
            }
        }
    }

    @Test void theWearingsNumbersAreChecked() {
        // a 205/55R16: 166 mm of tread on a 1.99 m circumference, 6.4 mm at 0.7: about 1.5 litres a tyre
        assertEquals(1.48e-3, WheelCorner.Wearing.treadVolumeM3(TYRE, 0.0064, 0.7), 0.02e-3);
        assertEquals(0.2 * 2 * Math.PI * 0.3 * 0.008 * 0.7, WheelCorner.Wearing.treadVolumeM3(0.2, 0.3, 0.008, 0.7), 1e-15);
        assertEquals(2e-4, WheelCorner.Wearing.liningVolumeM3(0.005, 0.01, 4), 1e-18);
        // at Wear's rates that tread lasts tens of thousands of km at 2 kJ of slip work a km
        double km = 1.48e-3 / Wear.TREAD_M3_PER_J / 2000;
        assertTrue(km > 20000 && km < 60000, km + " km");
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> new WheelCorner.Wearing(0, 1e-11, 3, 1, 1e-14, 500, 60, 1000, 1));
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> new WheelCorner.Wearing(1, -1, 3, 1, 1e-14, 500, 60, 1000, 1));
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> new WheelCorner.Wearing(1, 1e-11, 0, 1, 1e-14, 500, 60, 1000, 1));
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> new WheelCorner.Wearing(1, 1e-11, 3, 1, 1e-14, 500, 60, 0, 1));
        // an axle given none again wears nothing
        AxlePart axle = (AxlePart) car("checked", false, 0.3, TYPICAL, null).installed("front").part();
        assertEquals(TYPICAL, axle.wearing());
        assertEquals(List.of(), axle.wearing(null).conditions());
        assertEquals(3, axle.wearing(null).wearing(STILL).conditions().size());
    }
}

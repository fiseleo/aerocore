package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.ContactSolver;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AxlePart;
import dev.mcmod.aerocore.api.vehicle.part.MassPart;
import dev.mcmod.aerocore.api.wheel.Brake;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * 0.36: a tractor (8 t, 4x2) and a semi-trailer (20 t, a tandem) on a fifth wheel, stepped by TestTrack.convoy: the
 * articulation in a slow full-lock turn against the low-speed geometry, the points held together, reversing into the
 * jackknife stop, and the trailer's brakes alone stopping both.
 */
class ConvoyTest {
    static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    static final double G = TestTrack.G, TRACK = 2.05, WHEELBASE = 3.8, FIFTH_X = 0.3, PIN_Y = 1.25, TANDEM_FRONT = -7.4, TANDEM_REAR = -8.7;
    static final double MAX_STEER = 0.5, JACKKNIFE = Math.toRadians(80);
    static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC);
    /** A 495/45R22.5 super single. */
    static final Tyre TYRE = new Tyre(new Tyre.Spec(Tyre.Construction.RADIAL, 0.495, 0.45, 22.5 * 0.0254, 900e3, 900e3, Tyre.Tread.ROAD, 1));
    static final Coupling FIFTH = Coupling.fifthWheel(new Vec3d(FIFTH_X, PIN_Y, 0), new Vec3d(0, PIN_Y, 0), Math.toRadians(12), JACKKNIFE);

    /** A truck axle of super singles on leaf springs (1.5 Hz on its corner load, 0.3 of critical), braked, steered or not. */
    static PartDesign axle(String name, double cornerLoadN, AxlePart.Steering steering) {
        double mass = cornerLoadN / G, rate = mass * Math.pow(2 * Math.PI * 1.5, 2), c = 2 * 0.3 * Math.sqrt(rate * mass), design = 0.1, travel = 0.2;
        MassProperties m = new MassProperties(450, Vec3d.ZERO, 450 * TRACK * TRACK / 12, 450 * TRACK * TRACK / 12, 20, 0, 0, 0);
        return new TestParts.Fixture(name, "truck axle " + cornerLoadN + " " + steering, d -> {
            Suspension s = new Suspension(Suspension.Geometry.of(Suspension.Type.SOLID_LEAF, TRACK, TYRE.radiusM(), 0, -1, 0),
                    Suspension.Spring.coil(rate, Math.max(cornerLoadN - rate * design, 0)), Suspension.Damper.linear(c, c), travel, 0, design);
            Brake brake = new Brake(Brake.Type.VENTED_DISC, 15000, 35, 0.43, 0.2);
            WheelCorner left = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, -design, -TRACK / 2), -1, TRACK, TYRE, 12, s, brake, 0));
            WheelCorner right = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, -design, TRACK / 2), 1, TRACK, TYRE, 12, s, brake, 0));
            return new AxlePart(d, left, right, null, steering, 0, 0, null, m, List.of());
        });
    }
    static PartDesign box(String name, double kg, Vec3d size) {
        MassProperties m = new MassProperties(kg, Vec3d.ZERO, kg * (size.y() * size.y() + size.z() * size.z()) / 12,
                kg * (size.x() * size.x() + size.z() * size.z()) / 12, kg * (size.x() * size.x() + size.y() * size.y()) / 12, 0, 0, 0);
        return new TestParts.Fixture(name, "truck box " + kg + " " + size, d -> new MassPart(d, m, List.of()));
    }
    static VehicleBlueprint.Entry at(String id, PartDesign design, double x, List<Binding> bindings) {
        return new VehicleBlueprint.Entry(id, design, Mount.at(new Vec3d(x, TYRE.radiusM() - 0.03, 0)), bindings);
    }

    /** The tractor: its origin on the ground under the rear axle, the front axle 3.8 m ahead, laden by the kingpin. */
    static Vehicle tractor() {
        double pin = 6019 * G, front = (4000 * G + pin * FIFTH_X / WHEELBASE) / 2, rear = (4000 * G + pin * (1 - FIFTH_X / WHEELBASE)) / 2;
        return new VehicleBlueprint("tractor", List.of(
                new VehicleBlueprint.Entry("cab", box("cab", 7100, new Vec3d(5.8, 1.6, 2.4)), Mount.at(new Vec3d(1.9, 1.2, 0))),
                at("front", axle("front", front, new AxlePart.Steering(MAX_STEER, 1, WHEELBASE)), WHEELBASE, List.of(Binding.of("brake", "brake"), Binding.of("steer", "steer"))),
                at("rear", axle("rear", rear, null), 0, List.of(Binding.of("brake", "brake")))), List.of()).compile();
    }

    /** The trailer: its origin on the ground under the kingpin, a tandem 7.4 and 8.7 m behind, 20 t in all. */
    static Vehicle trailer() {
        double axle = 13981 * G / 4;
        return new VehicleBlueprint("trailer", List.of(
                new VehicleBlueprint.Entry("deck", box("deck", 19100, new Vec3d(12.5, 1.2, 2.5)), Mount.at(new Vec3d(-5.5, 1.6, 0))),
                at("tandem1", axle("tandem1", axle, null), TANDEM_FRONT, List.of(Binding.of("brake", "brake"))),
                at("tandem2", axle("tandem2", axle, null), TANDEM_REAR, List.of(Binding.of("brake", "brake")))), List.of()).compile();
    }

    record Rig(List<Vehicle> vehicles, List<VehicleState> states, List<ContactSolver.Joint> joints) {}

    /** The pair hitched straight at the origin facing +x and settled on their brakes. */
    static Rig rig(Coupling coupling) {
        Vehicle a = tractor(), b = trailer();
        VehicleState sa = a.newState(Vec3d.ZERO, Orientation.IDENTITY, TestTrack.DT);
        VehicleState sb = b.newState(TestTrack.hitchedOrigin(a, sa, coupling, 0), Orientation.IDENTITY, TestTrack.DT);
        Rig r = new Rig(List.of(a, b), List.of(sa, sb), List.of(new ContactSolver.Joint(0, 1, coupling)));
        TestTrack.convoy(r.vehicles(), r.states(), r.joints(), FLAT, AIR, (k, t, s) -> Map.of("brake", 1.0), 3, s -> false, Integer.MAX_VALUE);
        for (VehicleState s : r.states()) { s.setBody(new RigidBodyState(s.body().position(), Vec3d.ZERO, s.body().orientation(), Vec3d.ZERO)); }
        return r;
    }

    /** A push on the tractor (it has no engine here) holding a speed along its heading (negative: backing). */
    static List<AppliedForce> push(VehicleState s, double speed) {
        Vec3d heading = s.body().orientation().rotate(Vec3d.X);
        heading = new Vec3d(heading.x(), 0, heading.z()).normalized();
        double force = Math.clamp(28000 * 2 * (speed - s.body().velocity().dot(heading)), -60000, 60000);
        return List.of(new AppliedForce(AppliedForce.Kind.THRUST, Vec3d.ZERO, heading.scale(force)));
    }

    static double worstGap(List<TestTrack.ConvoySample> samples, double after) {
        double worst = 0;
        for (TestTrack.ConvoySample s : samples) { if (s.timeS() > after) { worst = Math.max(worst, s.joints().getFirst().gapM()); } }
        return worst;
    }

    @Test void aSlowTightTurnArticulatesAsTheGeometrySays() {
        // at full lock the kingpin's circle (7 m) is smaller than the trailer's wheelbase (8 m): it would fold; at 0.6 of it, a steady turn
        Rig r = rig(FIFTH);
        List<TestTrack.ConvoySample> samples = TestTrack.convoy(r.vehicles(), r.states(), r.joints(), FLAT, AIR, new TestTrack.ConvoyDriver() {
            @Override public Map<String, Double> controls(int k, double t, VehicleState s) { return k == 0 ? Map.of("steer", 0.6) : Map.of(); }
            @Override public List<AppliedForce> external(int k, double t, VehicleState s) { return k == 0 ? push(s, 1.5) : List.of(); }
        }, 70, s -> false, 10);
        // low speed, no slip: the rear axle on R1 = L / tan(delta); the kingpin on sqrt(R1^2 + c^2); the trailer's
        // tandem (its middle) on sqrt(Rp^2 - L2^2); the articulation atan(L2 / R2) - atan(c / R1)
        double r1 = WHEELBASE / Math.tan(0.6 * MAX_STEER), rp = Math.hypot(r1, FIFTH_X), l2 = -(TANDEM_FRONT + TANDEM_REAR) / 2;
        double r2 = Math.sqrt(rp * rp - l2 * l2), expected = Math.atan(l2 / r2) - Math.atan(FIFTH_X / r1);
        double sum = 0;
        int count = 0;
        for (TestTrack.ConvoySample s : samples) { if (s.timeS() > 55) { sum += s.joints().getFirst().yawRad(); count++; } }
        double articulation = -sum / count;
        System.out.printf("convoy tight turn: articulation %.1f deg against %.1f deg (R1 %.2f m, R2 %.2f m); worst gap %.2f mm%n",
                Math.toDegrees(articulation), Math.toDegrees(expected), r1, r2, worstGap(samples, 0.5) * 1000);
        assertEquals(expected, articulation, 0.1 * expected, "articulation (turning left, the trailer to the right of the tractor)");
        assertTrue(worstGap(samples, 0.5) < 0.002, "the kingpin stays in the fifth wheel: " + worstGap(samples, 0.5));
        assertTrue(Math.abs(samples.getLast().joints().getFirst().rollRad()) < 0.005, "roll carried");
    }

    @Test void backingFoldsTheTrailerToTheJackknifeStop() {
        Rig r = rig(FIFTH);
        List<TestTrack.ConvoySample> samples = TestTrack.convoy(r.vehicles(), r.states(), r.joints(), FLAT, AIR, new TestTrack.ConvoyDriver() {
            @Override public Map<String, Double> controls(int k, double t, VehicleState s) { return k == 0 ? Map.of("steer", 0.3) : Map.of(); }
            @Override public List<AppliedForce> external(int k, double t, VehicleState s) { return k == 0 ? push(s, -1.0) : List.of(); }
        }, 40, s -> false, 10);
        double most = 0;
        for (TestTrack.ConvoySample s : samples) { most = Math.max(most, Math.abs(s.joints().getFirst().yawRad())); }
        System.out.printf("convoy backing: articulation up to %.1f deg (stop %.0f deg); worst gap %.2f mm%n", Math.toDegrees(most), Math.toDegrees(JACKKNIFE),
                worstGap(samples, 0.5) * 1000);
        assertTrue(most > JACKKNIFE - 0.05, "folded to the stop: " + Math.toDegrees(most));
        assertTrue(most < JACKKNIFE + 0.03, "and no further: " + Math.toDegrees(most));
        assertTrue(worstGap(samples, 0.5) < 0.002, "points together: " + worstGap(samples, 0.5));
    }

    @Test void theTrailersBrakesAloneStopBoth() {
        Rig r = rig(FIFTH);
        for (VehicleState s : r.states()) { s.setBody(new RigidBodyState(s.body().position(), new Vec3d(10, 0, 0), s.body().orientation(), Vec3d.ZERO)); }
        List<TestTrack.ConvoySample> samples = TestTrack.convoy(r.vehicles(), r.states(), r.joints(), FLAT, AIR,
                (k, t, s) -> k == 1 ? Map.of("brake", 1.0) : Map.of(), 1.0, s -> false, 1);
        double tractor = r.states().get(0).body().velocity().x(), trailer = r.states().get(1).body().velocity().x();
        assertTrue(tractor < 9 && trailer < 9, "both slowed: " + tractor + ", " + trailer);
        assertEquals(trailer, tractor, 0.05, "together");
        double pull = 0;
        for (TestTrack.ConvoySample s : samples.subList(samples.size() / 2, samples.size())) { pull += s.joints().getFirst().forceN().x(); }
        assertTrue(pull > 0, "the trailer holds the tractor back through the kingpin (the force on the trailer forward)");
        assertTrue(worstGap(samples, 0.1) < 0.002, "points together: " + worstGap(samples, 0.1));
        // a forwarded brake channel: the tractor's driver brakes, the trailer's brakes work too
        Rig f = rig(FIFTH.withForwarded(List.of("brake")));
        for (VehicleState s : f.states()) { s.setBody(new RigidBodyState(s.body().position(), new Vec3d(10, 0, 0), s.body().orientation(), Vec3d.ZERO)); }
        TestTrack.convoy(f.vehicles(), f.states(), f.joints(), FLAT, AIR, (k, t, s) -> k == 0 ? Map.of("brake", 1.0) : Map.of(), 1.0, s -> false, 0);
        Rig o = rig(FIFTH);
        for (VehicleState s : o.states()) { s.setBody(new RigidBodyState(s.body().position(), new Vec3d(10, 0, 0), s.body().orientation(), Vec3d.ZERO)); }
        TestTrack.convoy(o.vehicles(), o.states(), o.joints(), FLAT, AIR, (k, t, s) -> k == 0 ? Map.of("brake", 1.0) : Map.of(), 1.0, s -> false, 0);
        assertTrue(f.states().get(0).body().velocity().x() < o.states().get(0).body().velocity().x() - 1,
                "with the trailer braking too: " + f.states().get(0).body().velocity().x() + " against " + o.states().get(0).body().velocity().x());
    }
}

package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

/** A vehicle refitted in place (0.34, a modification shop): set down at rest with what still fits aboard, the physics free to change. */
class RefitTest {
    private static final double DT = 0.005;

    /** A part with a diesel tank of a capacity and a count of internal states (each step adds 1 to every one). */
    static PartDesign tanker(String name, double capacityM3, int internals) {
        MassProperties m = MassShapes.box(new Vec3d(1, 0.4, 0.4), 1).scaled(40 / MassShapes.box(new Vec3d(1, 0.4, 0.4), 1).massKg());
        Part.Tank tank = new Part.Tank("main", capacityM3, Vec3d.ZERO, "diesel", 0.5 * capacityM3 * 840, false);
        return new TestParts.Fixture(name, "tanker " + capacityM3 + " " + internals, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<Tank> tanks() { return List.of(tank); }
            @Override public int internalStates() { return internals; }
            @Override public void loads(PartStep step) {
                step.burn(Set.of("diesel"), 0.001);
                for (int i = 0; i < internals; i++) { step.setInternal(i, step.internal(i) + 1); }
            }
        });
    }

    /** The test platform with its frame of a mass and, in place of the booster, the parts given. */
    static VehicleBlueprint platform(double frameKg, VehicleBlueprint.Entry... extra) {
        List<VehicleBlueprint.Entry> parts = new ArrayList<>();
        for (VehicleBlueprint.Entry e : RestyleTest.platform(false).parts()) {
            parts.add(e.id().equals("frame") ? new VehicleBlueprint.Entry("frame", TestParts.box("frame", new Vec3d(2, 0.2, 2), 0, false, 1, MassSpec.given(frameKg)), e.mount()) : e);
        }
        parts.addAll(List.of(extra));
        return new VehicleBlueprint("platform", parts, List.of());
    }

    private static VehicleState moving(Vehicle v) {
        VehicleState st = v.newState(new Vec3d(0, 0.1, 0), Orientation.rotationVector(new Vec3d(0.05, 0.3, -0.04)), DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(0.3, -0.2, 0.1), st.body().orientation(), new Vec3d(5, -3, 2)));
        RestyleTest.run(v, st, 0, 300);
        return st;
    }

    @Test void aRefitOfTheSameBlueprintIsPlacingItWhereItStands() {
        VehicleBlueprint bp = RestyleTest.platform(true);
        Vehicle v = bp.compile();
        VehicleState st = moving(v);
        for (int t = 0; t < v.trains(); t++) { st.setTrainSpeed(t, 10 + t); }
        st.setTank(0, "knsb", 5, 1000);

        VehicleState.Refitted r = st.refitted(bp.compile());
        VehicleState placed = st.placed(st.origin(), st.body().orientation());
        assertEquals(placed.body(), r.state().body(), "where it stands, at rest");
        assertEquals(Vec3d.ZERO, r.state().body().velocity());
        assertEquals(Vec3d.ZERO, r.state().body().angularMomentum());
        assertArrayEquals(placed.internals, r.state().internals, "internal states by part id");
        assertArrayEquals(placed.shown, r.state().shown);
        assertArrayEquals(placed.trainSpeeds, r.state().trainSpeeds);
        assertArrayEquals(placed.supply, r.state().supply);
        assertEquals(5, r.state().tankKg(0), 0);
        assertEquals("knsb", r.state().tankFuel(0));
        assertTrue(r.spilledKg().isEmpty(), "nothing spilled: " + r.spilledKg());
        assertFalse(r.state().primed, "the wave memory starts afresh");
    }

    @Test void aHeavierFrameChangesTheMassAndKeepsTheRest() {
        Vehicle v = platform(300, new VehicleBlueprint.Entry("tank", tanker("tank", 0.05, 3), Mount.at(new Vec3d(0, 0.5, 0)))).compile();
        VehicleState st = moving(v);
        Vehicle next = platform(500, new VehicleBlueprint.Entry("tank", tanker("tank", 0.05, 3), Mount.at(new Vec3d(0, 0.5, 0)))).compile();
        assertEquals(200, next.properties().massKg() - v.properties().massKg(), 1e-9);

        VehicleState.Refitted r = st.refitted(next);
        VehicleState out = r.state();
        assertSame(next, out.vehicle());
        assertEquals(st.origin().x(), out.origin().x(), 1e-9);
        assertEquals(st.origin().y(), out.origin().y(), 1e-9);
        assertEquals(st.origin().z(), out.origin().z(), 1e-9);
        assertEquals(st.body().orientation(), out.body().orientation());
        assertEquals(Vec3d.ZERO, out.body().velocity());
        for (int i = 0; i < 3; i++) { assertEquals(st.internal("tank", i), out.internal("tank", i), 0, "the tank's state " + i + " carried by id"); }
        assertTrue(st.internal("tank", 0) > 0, "the check is not empty");
        assertEquals(st.tankKg(0), out.tankKg(0), 0, "the same tank keeps its fuel");
        assertEquals("diesel", out.tankFuel(0));

        // stepping the refitted vehicle leaves the original alone
        RigidBodyState before = st.body();
        RestyleTest.run(next, out, 300, 100);
        assertEquals(before, st.body());
    }

    @Test void tanksKeepWhatFitsAndReportWhatSpills() {
        Vehicle v = platform(300,
                new VehicleBlueprint.Entry("big", tanker("big", 0.05, 1), Mount.at(new Vec3d(0, 0.5, 0))),
                new VehicleBlueprint.Entry("gone", tanker("gone", 0.01, 1), Mount.at(new Vec3d(0.5, 0.5, 0)))).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTank(0, "diesel", 40, 840);   // 40 of the 42 kg it takes
        st.setTank(1, "diesel", 6, 840);
        Vehicle next = platform(300,
                new VehicleBlueprint.Entry("big", tanker("big", 0.02, 1), Mount.at(new Vec3d(0, 0.5, 0))),
                new VehicleBlueprint.Entry("new", tanker("new", 0.01, 1), Mount.at(new Vec3d(0.5, 0.5, 0)))).compile();

        VehicleState.Refitted r = st.refitted(next);
        int big = indexOf(next, "big.main"), fresh = indexOf(next, "new.main");
        assertEquals(0.02 * 840, r.state().tankKg(big), 1e-9, "the smaller tank full at diesel's density");
        assertEquals("diesel", r.state().tankFuel(big));
        assertEquals(0, r.state().tankKg(fresh), 0, "a new tank is empty: a refit makes no fuel");
        assertEquals("", r.state().tankFuel(fresh));
        assertEquals(round(Map.of("big.main", 40 - 0.02 * 840, "gone.main", 6.0)), round(r.spilledKg()));
        assertEquals(40 - 0.02 * 840 + 6, r.spilledTotalKg(), 1e-9);

        // the caller's own densities
        VehicleState.Refitted light = st.refitted(next, fuel -> 500);
        assertEquals(0.02 * 500, light.state().tankKg(big), 1e-9);
    }

    @Test void aPartWithOtherStatesStartsAfresh() {
        Vehicle v = platform(300,
                new VehicleBlueprint.Entry("a", tanker("a", 0.05, 2), Mount.at(new Vec3d(0, 0.5, 0))),
                new VehicleBlueprint.Entry("b", tanker("b", 0.05, 1), Mount.at(new Vec3d(0.5, 0.5, 0)))).compile();
        VehicleState st = moving(v);
        assertTrue(st.internal("a", 0) > 0 && st.internal("b", 0) > 0);
        Vehicle next = platform(300,
                new VehicleBlueprint.Entry("b", tanker("b", 0.05, 1), Mount.at(new Vec3d(0.5, 0.5, 0))),
                new VehicleBlueprint.Entry("a", tanker("a", 0.05, 3), Mount.at(new Vec3d(0, 0.5, 0)))).compile();

        VehicleState out = st.refitted(next).state();
        for (int i = 0; i < 3; i++) { assertEquals(0, out.internal("a", i), 0, "three states where there were two: afresh"); }
        assertEquals(st.internal("b", 0), out.internal("b", 0), 0, "b keeps its state though it moved first");
    }

    private static int indexOf(Vehicle v, String tankId) {
        for (int k = 0; k < v.tanks().size(); k++) { if (v.tanks().get(k).id().equals(tankId)) { return k; } }
        throw new AssertionError("No tank " + tankId);
    }

    private static Map<String, Double> round(Map<String, Double> m) {
        Map<String, Double> out = new java.util.HashMap<>();
        m.forEach((k, x) -> out.put(k, Math.round(x * 1e9) / 1e9));
        return out;
    }
}

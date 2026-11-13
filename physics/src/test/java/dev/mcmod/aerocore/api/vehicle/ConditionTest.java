package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 0.42: the parts' conditions (what is worn or damaged, 0 new .. 1 failed) and the cargo in their holds: kept in the
 * state by part id and name, worn by the parts, set by the game, saved, and carried wherever the state goes.
 */
class ConditionTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final VehicleEnvironment ENV = VehicleEnvironment.calm(-1000, WATER, AIR);

    /**
     * A block that wears its first condition by perSecond in loads() and its second by half that in after(), and tells
     * what it read at the step's start; it may have holds.
     */
    static PartDesign wearer(String name, double massKg, List<String> conditions, double perSecond, List<Part.Hold> holds) {
        MassProperties m = MassShapes.box(new Vec3d(2, 1, 1), 1).scaled(massKg / MassShapes.box(new Vec3d(2, 1, 1), 1).massKg());
        return new TestParts.Fixture(name, "wearer " + massKg + " " + conditions + " " + perSecond + " " + holds, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<String> conditions() { return conditions; }
            @Override public List<Hold> holds() { return holds; }
            @Override public void loads(PartStep step) {
                step.diagnostic("first", step.used(0));
                step.diagnostic("beyond", step.used(7));
                step.wear(0, perSecond * step.dt());
                step.wear(7, 1);
            }
            @Override public void after(PartAfter step) { step.wear(1, 0.5 * perSecond * step.dt()); step.wear(7, 1); }
        });
    }

    static Vehicle vehicle(Object... idsAndParts) {
        List<VehicleBlueprint.Entry> entries = new ArrayList<>();
        for (int i = 0; i < idsAndParts.length; i += 2) { entries.add(new VehicleBlueprint.Entry((String) idsAndParts[i], (PartDesign) idsAndParts[i + 1], Mount.ORIGIN)); }
        return new VehicleBlueprint("worn", entries, List.of()).compile();
    }

    static void run(Vehicle v, VehicleState st, Vec3d gravity, List<AppliedForce> external, double seconds) {
        int n = (int) Math.round(seconds / DT);
        for (int i = 0; i < n; i++) { VehicleDynamics.step(v, st, ENV, Map.of(), gravity, external, null, i * DT, DT); }
    }

    private static VehicleState saved(Vehicle v, VehicleState st) { return VehicleState.fromJson(v, Json.parse(Json.write(st.toJson())), DT); }

    @Test void partsWearTheirConditionsUpToFailure() {
        Vehicle v = vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0.05, List.of()), "plain", wearer("plain", 50, List.of(), 0.05, List.of()));
        assertEquals(2, v.conditions());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        assertEquals(0, st.worstUsed(), 0);
        run(v, st, Vec3d.ZERO, List.of(), 10);
        assertEquals(0.5, st.used("a", 0), 1e-9);
        assertEquals(0.25, st.used("a", "brakes"), 1e-9);
        // loads() reads the value the step started with; an index the part does not have reads as new and wears nothing
        assertEquals(0.5 - 0.05 * DT, st.diagnostics().get("a").get("first"), 1e-9);
        assertEquals(0, st.diagnostics().get("a").get("beyond"), 0);
        assertEquals(0, st.used("plain", "tyres"), 0);
        assertEquals(0, st.used("nobody", "tyres"), 0);
        assertThrows(IllegalArgumentException.class, () -> st.used("nobody", 0));
        run(v, st, Vec3d.ZERO, List.of(), 30);
        assertEquals(1, st.used("a", 0), 0);
        assertEquals(1, st.worstUsed(), 0);
        assertEquals(1, st.used("a", 1), 1e-9);
    }

    @Test void theGameDamagesAndMendsBetweenAndWithinSteps() {
        Vehicle v = vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0.1, List.of()));
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setUsed("a", 0, 0.4);
        assertEquals(0.4, st.used("a", "tyres"), 0);
        assertEquals(0.7, st.addUsed("a", 0, 0.3), 1e-12);
        assertEquals(1, st.addUsed("a", 0, 5), 0);
        assertEquals(0, st.addUsed("a", 0, -9), 0);
        st.setUsed("a", 1, 7);
        assertEquals(1, st.used("a", 1), 0);
        assertThrows(IllegalArgumentException.class, () -> st.setUsed("a", 0, Double.NaN));
        assertThrows(IndexOutOfBoundsException.class, () -> st.setUsed("a", 2, 0.1));
        // damage that arrives between the two halves of a step is kept: the step adds what the part wore to it
        st.setUsed("a", 0, 0.2);
        VehicleDynamics.Prepared prepared = VehicleDynamics.prepare(v, st, ENV, Map.of(), Vec3d.ZERO, List.of(), 0, DT);
        st.setUsed("a", 0, 0.6);
        VehicleDynamics.finish(prepared, prepared.damped(), Wrench.ZERO, List.of(), 0, null);
        assertEquals(0.6 + 0.1 * DT, st.used("a", 0), 1e-12);
        // a client's copy
        double[] values = st.usedValues();
        VehicleState copy = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        copy.setUsedValues(values);
        assertEquals(st.used("a", 0), copy.used("a", 0), 0);
    }

    @Test void savedByNameAndReadIntoAChangedBlueprint() {
        Vehicle v = vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0.05, List.of()), "b", wearer("b", 100, List.of("structure"), 0, List.of()));
        VehicleState fresh = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        // new and empty: the saved state has nothing 0.41 did not write
        assertFalse(fresh.toJson().containsKey("condition"));
        assertFalse(fresh.toJson().containsKey("holds"));
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, Vec3d.ZERO, List.of(), 10);
        st.setUsed("b", 0, 0.9);
        VehicleState back = saved(v, st);
        assertEquals(0.5, back.used("a", "tyres"), 1e-9);
        assertEquals(0.25, back.used("a", "brakes"), 1e-9);
        assertEquals(0.9, back.used("b", "structure"), 0);
        // the blueprint changed since: the axle names its conditions in another order and has a new one, b is gone, c is new
        Vehicle changed = vehicle("c", wearer("c", 10, List.of("structure"), 0, List.of()),
                "a", wearer("a", 120, List.of("suspension", "brakes", "tyres"), 0.05, List.of()));
        VehicleState into = saved(changed, st);
        assertEquals(0.5, into.used("a", "tyres"), 1e-9);
        assertEquals(0.25, into.used("a", "brakes"), 1e-9);
        assertEquals(0, into.used("a", "suspension"), 0);
        assertEquals(0, into.used("c", "structure"), 0);
        // a state saved before 0.42 reads as new
        Map<String, Object> old = new java.util.LinkedHashMap<>(st.toJson());
        old.remove("condition");
        assertEquals(0, VehicleState.fromJson(v, old, DT).worstUsed(), 0);
    }

    @Test void carriedWhereverTheStateGoes() {
        List<Part.Hold> holds = List.of(new Part.Hold("boot", 0.4, new Vec3d(-1, 0, 0)));
        Vehicle v = vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0, holds), "b", wearer("b", 100, List.of("structure"), 0, List.of()));
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setUsed("a", 1, 0.3);
        st.setUsed("b", 0, 0.8);
        st.setHold(0, 75);
        // set down elsewhere
        VehicleState placed = st.placed(new Vec3d(10, 0, 0), Orientation.IDENTITY);
        assertEquals(0.3, placed.used("a", "brakes"), 0);
        assertEquals(0.8, placed.used("b", "structure"), 0);
        assertEquals(75, placed.holdKg(0), 0);
        // repainted: the same vehicle to the physics
        VehicleState restyled = st.restyled(vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0, holds), "b", wearer("b", 100, List.of("structure"), 0, List.of())));
        assertEquals(0.3, restyled.used("a", "brakes"), 0);
        assertEquals(75, restyled.holdKg(0), 0);
        // refitted: b is heavier and comes first, a has another condition; the hardware keeps its wear and the boot its load
        Vehicle next = vehicle("b", wearer("b", 300, List.of("structure"), 0, List.of()), "a", wearer("a", 100, List.of("suspension", "tyres", "brakes"), 0, holds));
        VehicleState refitted = st.refitted(next).state();
        assertEquals(0.3, refitted.used("a", "brakes"), 0);
        assertEquals(0, refitted.used("a", "suspension"), 0);
        assertEquals(0.8, refitted.used("b", "structure"), 0);
        assertEquals(75, refitted.holdKg(0), 0);
        // b cast off: what is left keeps its own
        VehicleState rest = st.separate(vehicle("a", wearer("a", 100, List.of("tyres", "brakes"), 0, holds)));
        assertEquals(0.3, rest.used("a", "brakes"), 0);
        assertEquals(75, rest.holdKg(0), 0);
        // a state built anew after a fault takes them over
        VehicleState anew = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        anew.keepHardware(st);
        assertEquals(0.8, anew.used("b", "structure"), 0);
        assertEquals(75, anew.holdKg(0), 0);
    }

    @Test void cargoWeighsAndAcceleratesWithTheVehicle() {
        List<Part.Hold> holds = List.of(new Part.Hold("bed", 2, new Vec3d(1, 0, 0)), new Part.Hold("box", 0.3, new Vec3d(0, 1, 0)));
        Vehicle v = vehicle("truck", wearer("truck", 100, List.of(), 0, holds));
        assertEquals("truck.bed", v.holds().get(0).id());
        assertEquals(100, v.properties().massKg(), 1e-9);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setHold(0, 50);
        assertEquals(50, st.cargoKg(), 0);
        run(v, st, new Vec3d(0, -9.81, 0), List.of(), 1);
        // all falls at g: no torque from the cargo's weight on its own mass
        assertEquals(-9.81, st.body().velocity().y(), 1e-6);
        assertEquals(0, st.body().angularMomentum().length(), 1e-6);
        // a force through the joint centre of mass accelerates vehicle and cargo together: 300 N on 150 kg
        VehicleState flat = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        flat.setHold(0, 50);
        run(v, flat, Vec3d.ZERO, List.of(new AppliedForce(AppliedForce.Kind.THRUST, new Vec3d(1.0 / 3, 0, 0), new Vec3d(300, 0, 0))), 1);
        assertEquals(2.0, flat.body().velocity().x(), 1e-3);
        assertEquals(0, flat.body().angularMomentum().length(), 1e-3);
        // the weight the step reports is the vehicle's and the cargo's
        VehicleDynamics.Result r = VehicleDynamics.step(v, st, ENV, Map.of(), new Vec3d(0, -9.81, 0), List.of(), null, 0, DT);
        assertEquals(-150 * 9.81, r.load(AppliedForce.Kind.WEIGHT).force().y(), 1e-9);
        // saved by hold id; an empty hold is not written
        st.setHold(1, 12.5);
        VehicleState back = saved(v, st);
        assertEquals(50, back.holdKg(0), 0);
        assertEquals(12.5, back.holdKg(1), 0);
        st.setHold(0, 0);
        assertTrue(((Map<?, ?>) st.toJson().get("holds")).containsKey("truck.box"));
        assertFalse(((Map<?, ?>) st.toJson().get("holds")).containsKey("truck.bed"));
        assertThrows(IllegalArgumentException.class, () -> st.setHold(0, -1));
    }

    @Test void aPartMayNotNameAConditionTwice() {
        assertThrows(IllegalStateException.class, () -> vehicle("a", wearer("a", 100, List.of("tyres", "tyres"), 0, List.of())));
    }
}

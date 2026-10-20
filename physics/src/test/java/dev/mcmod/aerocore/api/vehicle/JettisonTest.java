package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

/** Own tanks and casting parts off (0.28): boosters burn their own grain, and a burnt-out one leaves the vehicle. */
class JettisonTest {
    private static final double DT = 0.005;
    private static final VehicleEnvironment ENV = VehicleEnvironment.calm(-1000, dev.mcmod.aerocore.api.FluidState.FRESH_WATER_20C,
            dev.mcmod.aerocore.api.Atmosphere.atAltitude(0, Vec3d.ZERO).fluid());

    /** A block with an own tank that burns from it (and lists the vehicle's fuel too), asking to go when it runs dry. */
    static PartDesign booster(String name, double massKg, double grainKg, double kgPerS) {
        MassProperties m = MassShapes.box(new Vec3d(1, 0.3, 0.3), 1).scaled(massKg / MassShapes.box(new Vec3d(1, 0.3, 0.3), 1).massKg());
        Part.Tank grain = new Part.Tank("grain", 0.01, Vec3d.ZERO, "knsb", grainKg, true);
        return new TestParts.Fixture(name, "booster " + massKg + " " + grainKg + " " + kgPerS, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<Tank> tanks() { return List.of(grain); }
            @Override public Map<String, Mount> mountPoints() { return Map.of("fin", Mount.at(new Vec3d(-0.5, 0, 0))); }
            @Override public int internalStates() { return 1; }
            @Override public void loads(PartStep step) {
                step.burn(Set.of("knsb", "diesel"), kgPerS);
                step.setInternal(0, step.internal(0) + 1);
                if (step.fuelSupply() < 1) { step.jettison(); }
            }
        });
    }

    static VehicleBlueprint blueprint(VehicleBlueprint.Entry... parts) { return new VehicleBlueprint("stack", List.of(parts), List.of()); }

    static void run(Vehicle v, VehicleState st, double seconds) {
        int n = (int) Math.round(seconds / DT);
        for (int i = 0; i < n; i++) { VehicleDynamics.step(v, st, ENV, Map.of(), Vec3d.ZERO, List.of(), null, i * DT, DT); }
    }

    @Test void boostersBurnOnlyTheirOwnGrainAndNobodyElseDoes() {
        Vehicle v = blueprint(
                new VehicleBlueprint.Entry("core", FuelTankTest.tanker("core", 100, List.of(new Part.Tank("main", 0.1, Vec3d.ZERO, "diesel", 50))), Mount.ORIGIN),
                new VehicleBlueprint.Entry("left", booster("left", 20, 10, 1), Mount.at(new Vec3d(0, 0, -2))),
                new VehicleBlueprint.Entry("right", booster("right", 20, 5, 1), Mount.at(new Vec3d(0, 0, 2))),
                new VehicleBlueprint.Entry("engine", FuelTankTest.burner(Set.of("diesel", "knsb"), 0.5), Mount.ORIGIN)).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, 3);
        assertEquals(50 - 1.5, st.tankKg(0), 1e-6, "the engine burns the shared diesel only, though it would take grain");
        assertEquals(10 - 3, st.tankKg(1), 1e-6, "each booster its own grain, though it would take diesel");
        assertEquals(5 - 3, st.tankKg(2), 1e-6);
        run(v, st, 3);
        assertEquals(0, st.tankKg(2), 0, "the small booster ran dry");
        assertEquals(10 - 6, st.tankKg(1), 1e-6, "and did not take the other's grain");
        assertEquals(List.of("right"), st.takeJettisons(), "it asked to go");
        assertEquals(List.of(), st.takeJettisons(), "once taken, the requests are gone");
    }

    @Test void whatIsLeftKeepsItsPlaceAndMotionWithoutThePartsMass() {
        VehicleBlueprint full = blueprint(
                new VehicleBlueprint.Entry("core", FuelTankTest.tanker("core", 100, List.of(new Part.Tank("main", 0.1, Vec3d.ZERO, "diesel", 50))), Mount.ORIGIN),
                new VehicleBlueprint.Entry("booster", booster("booster", 40, 10, 1), Mount.at(new Vec3d(0, 0, 2))),
                new VehicleBlueprint.Entry("fin", FuelTankTest.tanker("fin", 5, List.of()), Mount.on("booster", "fin", Mount.at(new Vec3d(-0.2, 0, 0)))));
        Vehicle v = full.compile();
        VehicleState st = v.newState(new Vec3d(10, 20, 30), Orientation.rotationVector(new Vec3d(0, 0.3, 0)), DT);
        // moving and turning
        Vec3d omega = new Vec3d(0.2, 0.5, -0.1);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(30, 0, 5), st.body().orientation(), v.properties().inertia().applyWorld(st.body().orientation(), omega)));
        st.setTank(0, "diesel", 40, 850);
        run(v, st, 0.1);
        RigidBodyState before = st.body();
        Vec3d w = before.angularVelocity(v.properties());

        VehicleBlueprint.Split split = full.without(List.of("booster"));
        assertEquals(List.of("booster", "fin"), split.removed(), "what was attached to it goes with it");
        Vehicle rest = split.rest().compile();
        VehicleState after = st.separate(rest);
        Vehicle alone = blueprint(full.parts().getFirst()).compile();
        assertEquals(alone.properties().massKg(), rest.properties().massKg(), 1e-12);
        assertEquals(0, alone.properties().centerOfMassLocal().subtract(rest.properties().centerOfMassLocal()).length(), 1e-12);
        // the rest's centre is where that point of the old body was, moving with it, turning at the same rate
        Vec3d com = before.worldPoint(v.properties(), rest.properties().centerOfMassLocal());
        assertEquals(0, after.body().position().subtract(com).length(), 1e-9);
        assertEquals(0, after.body().velocity().subtract(before.velocity().add(w.cross(com.subtract(before.position())))).length(), 1e-9);
        assertEquals(0, after.body().angularVelocity(rest.properties()).subtract(w).length(), 1e-9);
        assertEquals(0, rest.origin(after.body()).subtract(v.origin(before)).length(), 1e-9, "the vehicle frame stays put");
        assertEquals(st.tankKg(0), after.tankKg(0), 0, "the core keeps its fuel");
        assertEquals(1, rest.tanks().size());
    }

    @Test void theGameCanSetAPartsStateBetweenSteps() {
        Vehicle v = blueprint(new VehicleBlueprint.Entry("a", booster("a", 20, 10, 0), Mount.ORIGIN)).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, 0.05);
        assertEquals(10, st.internal("a", 0), 0);
        st.setInternal("a", 0, 0);   // serviced: the part counts afresh
        run(v, st, 0.025);
        assertEquals(5, st.internal("a", 0), 0);
        assertThrows(IllegalArgumentException.class, () -> st.setInternal("a", 0, Double.NaN));
        assertThrows(IndexOutOfBoundsException.class, () -> st.setInternal("a", 1, 0));
        assertThrows(IllegalArgumentException.class, () -> st.setInternal("nobody", 0, 0));
    }

    @Test void theRestCarriesItsPartsStatesByName() {
        VehicleBlueprint full = blueprint(
                new VehicleBlueprint.Entry("a", booster("a", 20, 10, 0), Mount.ORIGIN),
                new VehicleBlueprint.Entry("b", booster("b", 20, 10, 0), Mount.at(new Vec3d(0, 0, 2))),
                new VehicleBlueprint.Entry("c", booster("c", 20, 10, 0), Mount.at(new Vec3d(0, 0, 4))));
        Vehicle v = full.compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, 0.05);   // ten steps: each part counted 10
        st.setTank(2, "knsb", 3, 1800);
        Vehicle rest = full.without(List.of("a")).rest().compile();
        VehicleState after = st.separate(rest);
        assertEquals(10, after.internal("b", 0), 0);
        assertEquals(10, after.internal("c", 0), 0);
        assertEquals(3, after.tankKg(1), 0, "c's grain is now the second tank");
    }
}

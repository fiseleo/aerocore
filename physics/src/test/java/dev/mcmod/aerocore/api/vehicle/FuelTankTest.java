package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Fuel tanks (0.26): their contents move and weigh with the vehicle, parts burn them, they save and load. */
class FuelTankTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final VehicleEnvironment ENV = VehicleEnvironment.calm(-1000, WATER, AIR);

    /** A dry block of a mass with tanks (no loads of its own). */
    static PartDesign tanker(String name, double massKg, List<Part.Tank> tanks) {
        MassProperties m = MassShapes.box(new Vec3d(2, 1, 1), 1).scaled(massKg / MassShapes.box(new Vec3d(2, 1, 1), 1).massKg());
        return new TestParts.Fixture(name, "tanker " + massKg + " " + tanks, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public List<Tank> tanks() { return tanks; }
            @Override public void loads(PartStep step) {}
        });
    }

    /** A small block that burns a fuel at a rate and shows its supply. */
    static PartDesign burner(Set<String> fuels, double kgPerS) {
        MassProperties m = MassShapes.box(new Vec3d(0.2, 0.2, 0.2), 1000);
        return new TestParts.Fixture("burner", "burner " + fuels + " " + kgPerS, d -> new Part() {
            @Override public PartDesign design() { return d; }
            @Override public MassProperties mass() { return m; }
            @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
            @Override public List<DesignIssue> issues() { return List.of(); }
            @Override public void loads(PartStep step) { step.burn(fuels, kgPerS); step.diagnostic("supply", step.fuelSupply()); }
        });
    }

    static Vehicle vehicle(PartDesign... parts) {
        List<VehicleBlueprint.Entry> entries = new ArrayList<>();
        for (int i = 0; i < parts.length; i++) { entries.add(new VehicleBlueprint.Entry("p" + i, parts[i], Mount.ORIGIN)); }
        return new VehicleBlueprint("fuel", entries, List.of()).compile();
    }

    static void run(Vehicle v, VehicleState st, Vec3d gravity, List<AppliedForce> external, double seconds) {
        int n = (int) Math.round(seconds / DT);
        for (int i = 0; i < n; i++) { VehicleDynamics.step(v, st, ENV, Map.of(), gravity, external, null, i * DT, DT); }
    }

    @Test void fuelFallsAndAcceleratesWithTheVehicle() {
        Vehicle v = vehicle(tanker("t", 100, List.of(new Part.Tank("main", 0.1, new Vec3d(1, 0, 0), "diesel", 50))));
        assertEquals(100, v.properties().massKg(), 1e-9);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, new Vec3d(0, -9.81, 0), List.of(), 1);
        // all falls at g: no torque from the tank's weight on its own mass
        assertEquals(-9.81, st.body().velocity().y(), 1e-6);
        assertEquals(0, st.body().angularMomentum().length(), 1e-6);
        // a force through the joint centre of mass accelerates dry mass and fuel together: 300 N on 150 kg
        VehicleState flat = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        Vec3d centre = new Vec3d(1.0 / 3, 0, 0);
        run(v, flat, Vec3d.ZERO, List.of(new AppliedForce(AppliedForce.Kind.THRUST, centre, new Vec3d(300, 0, 0))), 1);
        assertEquals(2.0, flat.body().velocity().x(), 1e-3);
        assertEquals(0, flat.body().angularMomentum().length(), 1e-3);
    }

    @Test void partsBurnFuelUntilTheTanksRunDry() {
        Vehicle v = vehicle(tanker("t", 100, List.of(new Part.Tank("main", 0.1, new Vec3d(1, 0, 0), "diesel", 50))), burner(Set.of("diesel"), 1));
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, Vec3d.ZERO, List.of(), 10);
        assertEquals(40, st.tankKg(0), 1e-6);
        assertEquals(1, st.fuelSupply(1), 1e-12);
        run(v, st, Vec3d.ZERO, List.of(), 45);
        assertEquals(0, st.tankKg(0), 0);
        assertEquals("", st.tankFuel(0));
        assertEquals(0, st.fuelSupply(1), 1e-12);
        assertEquals(0, st.diagnostics().get("p1").get("supply"), 1e-12);
        // the wrong fuel: nothing comes, the tank is untouched
        Vehicle wrong = vehicle(tanker("t", 100, List.of(new Part.Tank("main", 0.1, Vec3d.ZERO, "diesel", 50))), burner(Set.of("gasoline"), 1));
        VehicleState w = wrong.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(wrong, w, Vec3d.ZERO, List.of(), 1);
        assertEquals(50, w.tankKg(0), 0);
        assertEquals(0, w.fuelSupply(1), 0);
        // no tanks at all: everything is supplied (vehicles made before 0.26)
        Vehicle none = vehicle(tanker("t", 100, List.of()), burner(Set.of("diesel"), 1));
        VehicleState n = none.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(none, n, Vec3d.ZERO, List.of(), 1);
        assertEquals(1, n.fuelSupply(1), 0);
    }

    @Test void tanksDrainInProportionSaveAndRefill() {
        Vehicle v = vehicle(tanker("t", 100, List.of(new Part.Tank("left", 0.05, new Vec3d(0, 0, -1), "kerosene", 30),
                new Part.Tank("right", 0.05, new Vec3d(0, 0, 1), "kerosene", 10))), burner(Set.of("kerosene", "diesel"), 0.4));
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, Vec3d.ZERO, List.of(), 10);
        assertEquals(27, st.tankKg(0), 1e-6);
        assertEquals(9, st.tankKg(1), 1e-6);
        assertEquals(36, st.fuelKg(), 1e-6);
        assertEquals("p0.left", v.tanks().get(0).id());
        // saved and loaded
        VehicleState back = VehicleState.fromJson(v, dev.mcmod.aerocore.api.vehicle.Json.parse(dev.mcmod.aerocore.api.vehicle.Json.write(st.toJson())), DT);
        assertEquals(27, back.tankKg(0), 1e-9);
        assertEquals("kerosene", back.tankFuel(1));
        // a state saved before 0.26 (no tanks listed) keeps the design's fill
        Map<String, Object> old = new java.util.LinkedHashMap<>(st.toJson());
        old.remove("tanks");
        assertEquals(30, VehicleState.fromJson(v, old, DT).tankKg(0), 1e-9);
        // refuelling stops at the brim: 0.05 m^3 of kerosene at 800 kg/m^3 is 40 kg
        assertEquals(40, st.setTank(1, "kerosene", 100, 800), 1e-9);
        assertEquals(0, st.setTank(1, "", 100, 800), 0);
        assertTrue(st.tankFuel(1).isEmpty());
    }

    /** A floating tube body, its fuel in a tank or in its own mass. */
    static PartDesign floatingBody(boolean tank) {
        var circle = dev.mcmod.aerocore.api.body.SectionShape.Rounded.ellipse(1.2, 1.2, 0);
        var shape = new dev.mcmod.aerocore.api.body.BodyShape(circle, List.of(dev.mcmod.aerocore.api.body.BodyModule.straight("tube", 6, circle)));
        MassProperties dry = MassShapes.box(new Vec3d(6, 1.2, 1.2), 1).scaled(800 / MassShapes.box(new Vec3d(6, 1.2, 1.2), 1).massKg()).at(new Vec3d(3, 0, 0));
        Vec3d at = new Vec3d(2, -0.3, 0);
        MassProperties mass = tank ? dry : MassProperties.combine(List.of(dry, MassProperties.point(200, at)));
        List<Part.Tank> tanks = tank ? List.of(new Part.Tank("main", 0.3, at, "diesel", 200)) : List.of();
        return new TestParts.Fixture("body", "floating body " + tank, d -> new dev.mcmod.aerocore.api.vehicle.part.BodyPart(d, shape, 0,
                dev.mcmod.aerocore.api.vehicle.part.BodyPart.Hydro.DISPLACEMENT, 6, mass, Map.of(), List.of(), tanks, List.of()));
    }

    @Test void aBodyFloatsTheSameWithItsFuelInATank() {
        Vehicle tanked = vehicle(floatingBody(true)), solid = vehicle(floatingBody(false));
        assertEquals("p0.main", tanked.tanks().get(0).id());
        assertEquals(800, tanked.properties().massKg(), 1e-6);
        VehicleEnvironment water = VehicleEnvironment.calm(0, WATER, AIR);
        VehicleState a = tanked.newState(new Vec3d(-3, 0, 0), Orientation.IDENTITY, DT), b = solid.newState(new Vec3d(-3, 0, 0), Orientation.IDENTITY, DT);
        for (int i = 0; i < (int) (20 / DT); i++) {
            VehicleDynamics.step(tanked, a, water, Map.of(), new Vec3d(0, -9.81, 0), List.of(), null, i * DT, DT);
            VehicleDynamics.step(solid, b, water, Map.of(), new Vec3d(0, -9.81, 0), List.of(), null, i * DT, DT);
        }
        // where the body's own origin is (the states' positions are their dry centres of mass)
        Vec3d oa = a.body().position().subtract(a.body().orientation().rotate(tanked.properties().centerOfMassLocal()));
        Vec3d ob = b.body().position().subtract(b.body().orientation().rotate(solid.properties().centerOfMassLocal()));
        assertEquals(ob.y(), oa.y(), 1e-3, "the same draft");
        assertEquals(0, a.body().velocity().length(), 0.05, "settled");
        assertEquals(b.body().orientation().rotate(Vec3d.X).y(), a.body().orientation().rotate(Vec3d.X).y(), 1e-3, "the same trim");
    }
}

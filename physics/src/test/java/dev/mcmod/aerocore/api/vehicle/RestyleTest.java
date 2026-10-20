package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.BuoyantSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.TestPlatform;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveField;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** A vehicle restyled in place (0.31, a paint booth): it steps on exactly as it would have, and nothing else is let through. */
class RestyleTest {
    private static final double DT = 0.005;
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-Atmosphere.GRAVITY_MPS2);
    private static final dev.mcmod.aerocore.api.FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final WaveField WAVES = new WaveField(List.of(new LinearWave(0.15, 6, Math.toRadians(30), Double.POSITIVE_INFINITY, 0.4)));

    /** The test platform's four floats and frame (wave-radiation memory) with a booster burning its own grain (an internal state, a tank). */
    static VehicleBlueprint platform(boolean booster) {
        List<VehicleBlueprint.Entry> parts = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            BuoyantSphere s = TestPlatform.FLOATS.get(i);
            parts.add(new VehicleBlueprint.Entry("float" + i, TestParts.sphere("float", s.radiusM(), MassSpec.shell(Material.STEEL, 0.004)), Mount.at(s.offset())));
        }
        parts.add(new VehicleBlueprint.Entry("frame", TestParts.box("frame", new Vec3d(2, 0.2, 2), 0, false, 1, MassSpec.given(300)), Mount.at(new Vec3d(0, 0.35, 0))));
        if (booster) { parts.add(new VehicleBlueprint.Entry("booster", JettisonTest.booster("booster", 20, 50, 1), Mount.at(new Vec3d(0, 0.5, 0)))); }
        return new VehicleBlueprint("platform", parts, List.of());
    }

    static void run(Vehicle v, VehicleState st, int from, int steps) {
        for (int i = from; i < from + steps; i++) {
            double t = i * DT;
            VehicleDynamics.step(v, st, VehicleEnvironment.of(TestPlatform.wavyWater(0, WAVES, t), point -> HullDynamics.Water.calm(0, WATER, AIR), WAVES),
                    Map.of(), GRAVITY, List.of(), null, t, DT);
        }
    }

    @Test void aRestyledVehicleStepsOnExactlyAsItWould() {
        VehicleBlueprint bp = platform(true);
        Vehicle v = bp.compile();
        VehicleState st = v.newState(new Vec3d(0, 0.1, 0), Orientation.rotationVector(new Vec3d(0.05, 0.3, -0.04)), DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(0.3, -0.2, 0.1), st.body().orientation(), new Vec3d(5, -3, 2)));
        run(v, st, 0, 300);

        // the same blueprint compiled again: a new vehicle, the same to the physics (as a repainted one is)
        Vehicle again = bp.compile();
        assertNotSame(v, again);
        VehicleState restyled = st.restyled(again);
        assertSame(again, restyled.vehicle());
        assertEquals(st.body(), restyled.body(), "the body as it was");

        run(v, st, 300, 300);
        run(again, restyled, 300, 300);
        assertEquals(st.body(), restyled.body(), "position, velocity, orientation and momentum to the last bit, radiation memory included");
        for (int k = 0; k < st.vehicle().tanks().size(); k++) {
            assertEquals(st.tankKg(k), restyled.tankKg(k), 0, "tank " + k);
            assertEquals(st.tankFuel(k), restyled.tankFuel(k));
        }
        assertArrayEquals(st.internals, restyled.internals, "internal states");
        assertArrayEquals(st.shown, restyled.shown, "shown values");
        assertTrue(Arrays.stream(st.internals).anyMatch(x -> x != 0), "the booster counted its steps (the check is not empty)");
    }

    @Test void theOriginalIsNotChangedByTheCopy() {
        VehicleBlueprint bp = platform(true);
        Vehicle v = bp.compile();
        VehicleState st = v.newState(new Vec3d(0, 0.1, 0), Orientation.IDENTITY, DT);
        run(v, st, 0, 100);
        RigidBodyState before = st.body();
        double[] internals = st.internals.clone();
        VehicleState restyled = st.restyled(bp.compile());
        run(restyled.vehicle(), restyled, 100, 100);
        assertEquals(before, st.body(), "stepping the copy leaves the original where it was");
        assertArrayEquals(internals, st.internals);
    }

    @Test void onlyTheSameVehicleToThePhysicsIsLetThrough() {
        VehicleState st = platform(true).compile().newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        IllegalArgumentException fewer = assertThrows(IllegalArgumentException.class, () -> st.restyled(platform(false).compile()));
        assertTrue(fewer.getMessage().contains("mass"), "a part less is other mass properties: " + fewer.getMessage());

        // the same mass in another part: the parts differ
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(platform(false).parts());
        parts.add(new VehicleBlueprint.Entry("ballast", JettisonTest.booster("booster", 20, 50, 1), Mount.at(new Vec3d(0, 0.5, 0))));
        Vehicle renamed = new VehicleBlueprint("platform", parts, List.of()).compile();
        assertEquals(st.vehicle().properties(), renamed.properties(), "same mass properties");
        IllegalArgumentException other = assertThrows(IllegalArgumentException.class, () -> st.restyled(renamed));
        assertTrue(other.getMessage().contains("parts"), other.getMessage());
    }
}

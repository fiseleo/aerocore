package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

/** A vehicle set down somewhere else (0.32, an assembly room rolling it out): at rest there, with what is aboard kept. */
class PlaceTest {
    private static final double DT = 0.005;

    @Test void aPlacedVehicleKeepsItsTanksStatesAndShafts() {
        Vehicle v = RestyleTest.platform(true).compile();
        VehicleState st = v.newState(new Vec3d(0, 0.1, 0), Orientation.rotationVector(new Vec3d(0.05, 0.3, -0.04)), DT);
        st.setBody(new RigidBodyState(st.body().position(), new Vec3d(0.3, -0.2, 0.1), st.body().orientation(), new Vec3d(5, -3, 2)));
        RestyleTest.run(v, st, 0, 300);
        for (int t = 0; t < v.trains(); t++) { st.setTrainSpeed(t, 10 + t); }
        st.setTank(0, st.tankFuel(0).isEmpty() ? "test" : st.tankFuel(0), 0.5 * st.tankKg(0), 1000);
        assertTrue(st.primed, "the wave memory has taken radii");

        Vec3d origin = new Vec3d(40, 3, -12);
        Orientation q = Orientation.rotationVector(new Vec3d(0, 1.2, 0));
        VehicleState placed = st.placed(origin, q);
        assertSame(v, placed.vehicle());
        assertEquals(v.newState(origin, q, DT).body(), placed.body(), "where newState would set it down, at rest");
        assertEquals(Vec3d.ZERO, placed.body().velocity());
        assertEquals(Vec3d.ZERO, placed.body().angularMomentum());
        for (int k = 0; k < v.tanks().size(); k++) {
            assertEquals(st.tankKg(k), placed.tankKg(k), 0, "tank " + k);
            assertEquals(st.tankFuel(k), placed.tankFuel(k));
        }
        assertArrayEquals(st.internals, placed.internals, "internal states");
        assertArrayEquals(st.shown, placed.shown, "shown values");
        assertArrayEquals(st.trainSpeeds, placed.trainSpeeds, "shaft speeds");
        assertArrayEquals(st.supply, placed.supply);
        assertTrue(Arrays.stream(st.internals).anyMatch(x -> x != 0), "the booster counted its steps (the check is not empty)");
        assertFalse(placed.primed, "the wave memory starts afresh");

        // stepping the placed vehicle leaves the original alone
        RigidBodyState before = st.body();
        double[] internals = st.internals.clone();
        RestyleTest.run(v, placed, 300, 100);
        assertEquals(before, st.body());
        assertArrayEquals(internals, st.internals);
    }

    @Test void aFreshVehiclePlacedIsANewState() {
        Vehicle v = RestyleTest.platform(true).compile();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        Vec3d origin = new Vec3d(1, 2, 3);
        VehicleState placed = st.placed(origin, Orientation.IDENTITY);
        VehicleState fresh = v.newState(origin, Orientation.IDENTITY, DT);
        assertEquals(fresh.body(), placed.body());
        assertArrayEquals(fresh.internals, placed.internals);
        for (int k = 0; k < v.tanks().size(); k++) { assertEquals(fresh.tankKg(k), placed.tankKg(k), 0); }
    }
}

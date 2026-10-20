package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AxlePart;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Air springs with a levelling valve (0.33): the ride heights hold under any load; coil axles are as they were. */
class AirSpringTest {
    static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC);
    static final double G = TestTrack.G, TRACK = 1.55, DESIGN = 0.12, TRAVEL = 0.2, AREA = 0.0079, VOLUME = 0.003;
    static final Suspension.Levelling VALVE = new Suspension.Levelling(0.04, 0.05, 0.15, 0.15, 0.004);
    static final Tyre TYRE = TestParts.carTyre();

    /** An air axle sized for a corner load at the design compression (its air at droop scaled back from there). */
    static PartDesign axle(String id, double cornerLoadN) {
        double designAbs = cornerLoadN / AREA + 101325, droop = designAbs * Math.pow((VOLUME - AREA * DESIGN) / VOLUME, Suspension.AIR_POLYTROPIC);
        Suspension.Spring spring = Suspension.Spring.air(AREA, VOLUME, droop);
        double c = 2 * 0.3 * Math.sqrt(spring.rate(DESIGN) * cornerLoadN / G);
        MassProperties m = new MassProperties(100, Vec3d.ZERO, 100 * TRACK * TRACK / 12, 100 * TRACK * TRACK / 12, 5, 0, 0, 0);
        return new TestParts.Fixture(id, "air axle " + id + " " + cornerLoadN, d -> {
            Suspension s = new Suspension(Suspension.Geometry.of(Suspension.Type.TRAILING_ARM, TRACK, TYRE.radiusM(), 0, -1, 0), spring,
                    Suspension.Damper.linear(c, c), TRAVEL, 0, DESIGN, null, VALVE);
            WheelCorner left = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, -DESIGN, -TRACK / 2), -1, TRACK, TYRE, 0.8, s, TestParts.carBrake(), 0));
            WheelCorner right = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, -DESIGN, TRACK / 2), 1, TRACK, TYRE, 0.8, s, TestParts.carBrake(), 0));
            return new AxlePart(d, left, right, null, null, 0, 0, null, m, List.of());
        });
    }

    static Vehicle car(double bodyKg) {
        double corner = (bodyKg + 200) * G / 4;
        return new VehicleBlueprint("air", List.of(
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4.2, 1.0, 1.7), 0, false, 1, MassSpec.given(bodyKg)), Mount.at(new Vec3d(0, 0.6, 0))),
                new VehicleBlueprint.Entry("front", axle("front", corner), Mount.at(new Vec3d(1.35, TYRE.radiusM() - TYRE.deflectionM(corner), 0)),
                        List.of(Binding.of("brake", "brake"), Binding.of("ride_height", "ride_height"))),
                new VehicleBlueprint.Entry("rear", axle("rear", corner), Mount.at(new Vec3d(-1.35, TYRE.radiusM() - TYRE.deflectionM(corner), 0)),
                        List.of(Binding.of("brake", "brake"), Binding.of("ride_height", "ride_height")))), List.of()).compile();
    }

    /** Each wheel's compression after holding a ride height for some seconds. */
    static double[] held(Vehicle v, VehicleState st, double level, double seconds) {
        TestTrack.drive(v, st, FLAT, AIR, (t, s) -> Map.of("brake", 1.0, "ride_height", level), seconds, s -> false, Integer.MAX_VALUE);
        double[] c = new double[4];
        int k = 0;
        for (Vehicle.Installed i : v.installed()) {
            if (!(i.part() instanceof AxlePart)) { continue; }
            c[k++] = st.internals[i.firstInternal()];
            c[k++] = st.internals[i.firstInternal() + WheelCorner.STATES];
        }
        return c;
    }

    @Test void coilAxlesAreAsTheyWere() {
        PartDesign coil = TestParts.axle("c", TRACK, 30000, 4000, 2000, 2000, Suspension.Type.DOUBLE_WISHBONE, true, null, null, 0, 0);
        Part p = coil.compile();
        assertEquals(2 * WheelCorner.STATES, p.internalStates());
        assertFalse(p.inputs().contains("ride_height"));
        Part air = axle("a", 4000).compile();
        assertEquals(2 * WheelCorner.STATES + 2, air.internalStates());
        assertTrue(air.inputs().contains("ride_height"));
        // the scaled spring is the spring at scale 1
        Suspension.Spring s = Suspension.Spring.air(AREA, VOLUME, 4e5);
        assertEquals(s.force(0.1), s.force(0.1, 1), 0);
        assertEquals(1.5 * 4e5 * AREA * Math.pow(VOLUME / (VOLUME - AREA * 0.1), Suspension.AIR_POLYTROPIC) - 101325 * AREA, s.force(0.1, 1.5), 1e-9);
    }

    @Test void theValveHoldsEachLevelWhateverTheLoad() {
        for (double body : new double[]{1200, 1600}) {
            Vehicle v = car(body);
            VehicleState st = TestTrack.settle(v, FLAT, AIR, Vec3d.ZERO, 0, Map.of("brake", 1.0, "ride_height", 0.5), 1);
            for (double level : new double[]{0.5, 1, 0, 0.5}) {
                double target = VALVE.target(DESIGN, level);
                for (double c : held(v, st, level, 20)) {
                    assertEquals(target, c, VALVE.deadbandM() + 0.003, body + " kg at level " + level);
                }
            }
        }
    }

    @Test void theBodyRisesInSecondsAndTheValveRests() {
        Vehicle v = car(1200);
        VehicleState st = TestTrack.settle(v, FLAT, AIR, Vec3d.ZERO, 0, Map.of("brake", 1.0, "ride_height", 0.5), 6);
        double low = st.origin().y();
        held(v, st, 1, 20);
        double high = st.origin().y();
        assertEquals(VALVE.highM(), high - low, 0.012, "the body rises by the high level's height");
        int air = v.installed("front").firstInternal() + 2 * WheelCorner.STATES;
        double before = st.internals[air];
        held(v, st, 1, 2);
        assertEquals(before, st.internals[air], 1e-12, "held, the valve neither fills nor vents");
    }
}

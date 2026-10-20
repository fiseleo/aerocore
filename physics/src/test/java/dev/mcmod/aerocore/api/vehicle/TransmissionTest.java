package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.drive.TorqueConverter;
import dev.mcmod.aerocore.api.vehicle.part.TransferPart;
import dev.mcmod.aerocore.api.vehicle.part.TransmissionPart;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Gearboxes and transfer cases (0.24): what the hardware does with its inputs. */
class TransmissionTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final VehicleEnvironment SPACE = VehicleEnvironment.calm(-100, FluidState.FRESH_WATER_20C, AIR);
    private static final double[] SIX = {3.6, 2.1, 1.4, 1.0, 0.8, 0.65};

    private static PartDesign box(String key, TransmissionPart.Spec spec) {
        return new TestParts.Fixture("box", "box " + key, d -> new TransmissionPart(d, spec, MassProperties.point(40, Vec3d.ZERO), List.of()));
    }

    private static TransmissionPart.Spec spec(TransmissionPart.Kind kind) {
        return switch (kind) {
            case MANUAL -> new TransmissionPart.Spec(kind, SIX, -3.5, 0.97, 400, 60, null, 0, 0.3, 0, 0, 0.02, 0.01, 0.01, false);
            case AUTOMATIC -> new TransmissionPart.Spec(kind, SIX, -3.4, 0.95, 600, 0, new TorqueConverter(2.0, 12.29), 600, 0.4, 0, 0, 0.1, 0.03, 0.01, false);
            case DUAL_CLUTCH -> new TransmissionPart.Spec(kind, SIX, -3.5, 0.96, 450, 60, null, 0, 0.15, 0, 0, 0.02, 0.01, 0.01, false);
            case CVT -> new TransmissionPart.Spec(kind, new double[]{2.4, 0.45}, -1, 0.9, 0, 0, new TorqueConverter(2.0, 12.29), 400, 0.4, 0.5, 300, 0.1, 0.03, 0.01, false);
        };
    }

    /**
     * An engine (0.2 kg m^2, "torque" on "torque") through a part to a load: a 4 kg m^2 flywheel standing for the
     * vehicle, with resistance c x speed (external), on a body held still.
     */
    private static Vehicle rig(String key, PartDesign middle, List<Binding> bindings, String in, String out, double inertia, double resistance) {
        PartDesign load = TestParts.gears("l", "load " + inertia + " " + resistance, List.of(new Part.Rotor("", Vec3d.X, inertia, false)), List.of(), List.of(), 0, 0,
                s -> s.rotorTorque(0, -resistance * s.rotorSpeed(0), -resistance, true), a -> {});
        return new VehicleBlueprint("rig " + key, List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.2, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("box", middle, Mount.ORIGIN, bindings),
                new VehicleBlueprint.Entry("load", load, Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(Connection.Shaft.ports("engine", null, "box", in, 1, 1), Connection.Shaft.ports("box", out, "load", null, 1, 1))).compile();
    }

    private static List<Binding> bind(String... roles) {
        List<Binding> b = new ArrayList<>();
        for (String r : roles) { b.add(Binding.of(r, r)); }
        return b;
    }

    /** Steps with controls from the state; returns the last result. */
    private static VehicleDynamics.Result run(Vehicle v, VehicleState st, double seconds, Function<VehicleState, Map<String, Double>> controls) {
        VehicleDynamics.Result r = null;
        int steps = (int) Math.round(seconds / DT);
        for (int i = 0; i < steps; i++) {
            st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
            r = VehicleDynamics.step(v, st, SPACE, controls.apply(st), Vec3d.ZERO, List.of(), null, i * DT, DT);
        }
        return r;
    }

    private static double speed(VehicleState st, String id, int k) { return st.rotorSpeed(st.vehicle().installed(id).firstRotor() + k); }

    @Test void aManualBoxSlipsItsClutchSynchronisesAndGrindsWithoutTheClutch() {
        Vehicle v = rig("manual", box("manual", spec(TransmissionPart.Kind.MANUAL)), bind("gear", "clutch"), "input", "output", 4, 0.2);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        // engine governed at 200 rad/s (at most 150 N m), first gear, the clutch let out over 1.5 s
        Function<VehicleState, Double> governor = s -> Math.clamp(5 * (200 - speed(s, "engine", 0)), 0, 150.0);
        run(v, st, 1, s -> Map.of("torque", governor.apply(s), "gear", 0.0, "clutch", 1.0));
        double[] t = {0};
        VehicleDynamics.Result r = run(v, st, 3, s -> {
            t[0] += DT;
            return Map.of("torque", governor.apply(s), "gear", 1.0, "clutch", Math.max(0, 1 - t[0] / 1.5));
        });
        assertEquals(1, r.diagnostics().get("box").get("gear"), 0, "in first");
        assertEquals(speed(st, "box", 0), speed(st, "box", 1), 1e-6, "the clutch has locked");
        assertEquals(3.6 * speed(st, "load", 0), speed(st, "box", 1), 1e-6 * speed(st, "box", 1), "first gear's ratio");
        // clutch pressed, lever to second: the cone brings the primary shaft (0.01 kg m^2 and the clutch disc) to the gear's speed
        double before = speed(st, "box", 1), target = 2.1 * speed(st, "load", 0);
        int steps = 0;
        do {
            r = run(v, st, DT, s -> Map.of("torque", 0.0, "gear", 2.0, "clutch", 1.0));
            steps++;
        } while (r.diagnostics().get("box").get("gear") != 2 && steps < 400);
        double expected = 0.01 * (before - target) / 60;
        assertEquals(expected, steps * DT, 0.5 * expected + 2 * DT, "synchronising time I dw / T");
        // lever to third with the clutch still engaged: the cone has to swing the engine as well, which keeps pulling: it grinds
        r = run(v, st, 1, s -> Map.of("torque", governor.apply(s), "gear", 3.0, "clutch", 0.0));
        assertEquals(0, r.diagnostics().get("box").get("gear"), 0, "not in gear");
        assertEquals(2, r.diagnostics().get("box").get("state"), 0, "grinding");
    }

    /**
     * A vehicle mod's gearbox: its own rule (here: "go" above 0.5 means first gear with the clutch let out) turns the
     * driver's input into the box's commands through PartStep.override, then the hardware reads them.
     */
    @Test void aWrappingPartCommandsTheBoxThroughOverride() {
        PartDesign wrapped = new TestParts.Fixture("ruled", "ruled box", d -> {
            TransmissionPart inner = new TransmissionPart(d, spec(TransmissionPart.Kind.MANUAL), MassProperties.point(40, Vec3d.ZERO), List.of());
            return new Part() {
                @Override public PartDesign design() { return d; }
                @Override public MassProperties mass() { return inner.mass(); }
                @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
                @Override public List<DesignIssue> issues() { return List.of(); }
                @Override public List<Rotor> rotors() { return inner.rotors(); }
                @Override public List<Link> links() { return inner.links(); }
                @Override public List<String> inputs() { return List.of("go"); }
                @Override public int internalStates() { return TransmissionPart.STATES + 1; }
                @Override public double[] initialInternals() { return java.util.Arrays.copyOf(inner.initialInternals(), TransmissionPart.STATES + 1); }
                @Override public int shownValues() { return inner.shownValues(); }
                @Override public void loads(PartStep step) {
                    boolean go = step.input("go", 0) > 0.5;
                    step.override("gear", go ? 1 : 0);
                    step.override("clutch", 0);
                    step.setInternal(TransmissionPart.STATES, inner.engaged(step));   // the rule's own memory: what it saw
                    inner.loads(step);
                }
            };
        });
        Vehicle v = rig("ruled", wrapped, bind("go"), "input", "output", 4, 0.2);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        // the "gear" channel the driver sends is not what the box reads: only the rule's override is
        VehicleDynamics.Result r = run(v, st, 0.5, s -> Map.of("torque", 0.0, "go", 0.0, "gear", 3.0));
        assertEquals(0, r.diagnostics().get("box").get("gear"), 0, "neutral while not going");
        r = run(v, st, 0.5, s -> Map.of("torque", 0.0, "go", 1.0, "gear", 3.0));
        assertEquals(1, r.diagnostics().get("box").get("gear"), 0, "first gear by the rule");
        assertEquals(1, st.internals[v.installed("box").firstInternal() + TransmissionPart.STATES], 0, "engaged() reads it back");
    }

    @Test void anAutomaticStallsAtItsConverterAndShiftsClutchToClutch() {
        // stall: in drive against a load too heavy to move, 200 N m holds the pump at K sqrt(200)
        Vehicle stall = rig("auto stall", box("auto", spec(TransmissionPart.Kind.AUTOMATIC)), bind("gear", "lockup"), "input", "output", 1e7, 0);
        VehicleState ss = stall.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(stall, ss, 3, s -> Map.of("torque", 200.0, "gear", 1.0));
        assertEquals(12.29 * Math.sqrt(200), speed(ss, "box", 0), 0.01 * 12.29 * Math.sqrt(200), "stall speed");
        Vehicle v = rig("auto", box("auto", spec(TransmissionPart.Kind.AUTOMATIC)), bind("gear", "lockup"), "input", "output", 4, 0.2);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        // in park the turbine turns free and the output stays
        run(v, st, 1, s -> Map.of("torque", 50.0, "gear", -2.0));
        assertEquals(0, speed(st, "box", 2), 1e-9, "park");
        assertTrue(speed(st, "box", 1) > 10, "the turbine turns with the pump");
        // drive in first, then shift to second: the output keeps being pushed while the elements hand over
        run(v, st, 3, s -> Map.of("torque", 120.0, "gear", 1.0, "lockup", 0.0));
        double first = speed(st, "box", 1) / speed(st, "box", 2);
        assertEquals(3.6, first, 1e-6, "turbine and output at first's ratio");
        double before = speed(st, "load", 0);
        double[] lowest = {Double.POSITIVE_INFINITY};
        for (int i = 0; i < 80; i++) {
            run(v, st, DT, s -> Map.of("torque", 120.0, "gear", 2.0, "lockup", 0.0));
            lowest[0] = Math.min(lowest[0], speed(st, "load", 0) - before);
        }
        assertEquals(2.1, speed(st, "box", 1) / speed(st, "box", 2), 1e-6, "in second at the end of the shift time");
        assertTrue(lowest[0] > -0.05, "no drop in the vehicle's speed through the shift: " + lowest[0]);
        assertTrue(speed(st, "load", 0) > before, "still accelerating");
        // park at speed ratchets: the pawl does not catch
        run(v, st, 0.2, s -> Map.of("torque", 0.0, "gear", -2.0));
        assertTrue(Math.abs(speed(st, "box", 2)) > TransmissionPart.PAWL, "rolling on in park above the pawl's speed");
    }

    @Test void aDualClutchBoxHandsOverWithoutABreakInDrive() {
        Vehicle v = rig("dct", box("dct", spec(TransmissionPart.Kind.DUAL_CLUTCH)), bind("gear", "clutch"), "input", "output", 4, 0.2);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        Function<VehicleState, Double> governor = s -> Math.clamp(5 * (300 - speed(s, "engine", 0)), 0, 150.0);
        run(v, st, 4, s -> Map.of("torque", governor.apply(s), "gear", 1.0, "clutch", 1.0));
        assertEquals(1, run(v, st, DT, s -> Map.of("torque", governor.apply(s), "gear", 1.0, "clutch", 1.0)).diagnostics().get("box").get("gear"), 0);
        // second is ready on the even shaft (preselected): the handover takes the shift time and the load never slows
        double before = speed(st, "load", 0);
        int steps = 0;
        double lowest = Double.POSITIVE_INFINITY;
        VehicleDynamics.Result r;
        do {
            r = run(v, st, DT, s -> Map.of("torque", governor.apply(s), "gear", 2.0, "clutch", 1.0));
            lowest = Math.min(lowest, speed(st, "load", 0) - before);
            steps++;
        } while ((r.diagnostics().get("box").get("gear") != 2 || r.diagnostics().get("box").get("state") != 0) && steps < 400);
        assertEquals(0.15, steps * DT, 0.03, "handover in the shift time");
        assertTrue(lowest > -1e-3, "no break in drive: " + lowest);
        run(v, st, 2, s -> Map.of("torque", governor.apply(s), "gear", 2.0, "clutch", 1.0));
        assertEquals(2.1, speed(st, "box", 2) / speed(st, "box", 3), 1e-6, "even shaft in second");
    }

    @Test void aCvtMovesItsRatioAtItsRateAndItsBeltSlipsAboveCapacity() {
        Vehicle v = rig("cvt", box("cvt", spec(TransmissionPart.Kind.CVT)), bind("gear", "ratio", "lockup"), "input", "output", 4, 0.2);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, 1, s -> Map.of("torque", 60.0, "gear", 1.0, "ratio", 0.45, "lockup", 1.0));
        assertEquals(2.4 - 0.5 * 1, r.diagnostics().get("box").get("ratio"), 1e-9, "at most 0.5 per second toward the target");
        r = run(v, st, 4, s -> Map.of("torque", 60.0, "gear", 1.0, "ratio", 0.45, "lockup", 1.0));
        assertEquals(0.45, r.diagnostics().get("box").get("ratio"), 1e-9);
        assertEquals(0.45, speed(st, "box", 1) / speed(st, "box", 2), 1e-6, "the belt holds below its capacity");
        // locked converter, 400 N m at the primary pulley against a 300 N m belt: it slips
        Vehicle slipping = rig("cvt slip", box("cvt", spec(TransmissionPart.Kind.CVT)), bind("gear", "ratio", "lockup"), "input", "output", 40, 0.2);
        VehicleState s2 = slipping.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(slipping, s2, 0.5, s -> Map.of("torque", 400.0, "gear", 1.0, "ratio", 2.4, "lockup", 1.0));
        assertTrue(speed(s2, "box", 1) / speed(s2, "box", 2) > 2.4 * 1.05, "belt slipping: " + speed(s2, "box", 1) / speed(s2, "box", 2));
    }

    private static PartDesign transfer(String key, TransferPart.Spec spec) {
        return new TestParts.Fixture("transfer", "transfer " + key, d -> new TransferPart(d, spec, MassProperties.point(30, Vec3d.ZERO), List.of()));
    }

    /** Engine to a transfer case to two loads (front and rear) of their own inertia and resistance. */
    private static Vehicle fourByFour(String key, TransferPart.Spec spec, List<Binding> bindings, double frontResistance, double rearResistance) {
        PartDesign front = TestParts.gears("f", "front load " + frontResistance, List.of(new Part.Rotor("", Vec3d.X, 2, false)), List.of(), List.of(), 0, 0,
                s -> s.rotorTorque(0, -frontResistance * s.rotorSpeed(0), -frontResistance, true), a -> {});
        PartDesign rear = TestParts.gears("r", "rear load " + rearResistance, List.of(new Part.Rotor("", Vec3d.X, 2, false)), List.of(), List.of(), 0, 0,
                s -> s.rotorTorque(0, -rearResistance * s.rotorSpeed(0), -rearResistance, true), a -> {});
        return new VehicleBlueprint("4x4 " + key, List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.2, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("transfer", transfer(key, spec), Mount.ORIGIN, bindings),
                new VehicleBlueprint.Entry("front", front, Mount.ORIGIN), new VehicleBlueprint.Entry("rear", rear, Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(Connection.Shaft.ports("engine", null, "transfer", "input", 1, 1), Connection.Shaft.ports("transfer", "front", "front", null, 1, 1),
                        Connection.Shaft.ports("transfer", "rear", "rear", null, 1, 1))).compile();
    }

    @Test void transferCasesSplitLockAndCouple() {
        // part-time: 2H drives the rear only; 4H locks the axles; 4L engages only near standstill and gears down 2.72
        TransferPart.Spec part = new TransferPart.Spec(TransferPart.Kind.PART_TIME, 2.72, 1, 0.5, TransferPart.Centre.OPEN, 0, 0, false, 0, 0, false, 0.05, 0.05, 0.05, false);
        Vehicle pt = fourByFour("part", part, bind("mode"), 1, 3);
        VehicleState st = pt.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(pt, st, 1, s -> Map.of("torque", 60.0, "mode", 0.0));
        assertEquals(0, speed(st, "front", 0), 1e-9, "2H: the front is not driven");
        run(pt, st, 1, s -> Map.of("torque", 60.0, "mode", 1.0));
        assertEquals(speed(st, "front", 0), speed(st, "rear", 0), 1e-6, "4H: locked together");
        assertTrue(speed(st, "rear", 0) > TransferPart.RANGE_CHANGE, "moving: " + speed(st, "rear", 0));
        run(pt, st, 0.2, s -> Map.of("torque", 60.0, "mode", 2.0));
        assertEquals(1, st.shown("transfer", 0), 0, "too fast for low range");
        run(pt, st, 6, s -> Map.of("torque", 0.0, "mode", 1.0));
        run(pt, st, 0.1, s -> Map.of("torque", 0.0, "mode", 2.0));
        assertEquals(2, st.shown("transfer", 0), 0, "low range near standstill");
        run(pt, st, 1, s -> Map.of("torque", 20.0, "mode", 2.0));
        assertEquals(2.72, speed(st, "engine", 0) / speed(st, "rear", 0), 1e-6, "4L gears down 2.72");
        // full-time 40/60: an open centre differential sends 40 % forward whatever the speeds, so equal axles turn at
        // 40 : 60; a Torsen (bias 3) holds the even split equal axles need, so they turn alike
        TransferPart.Spec open = new TransferPart.Spec(TransferPart.Kind.FULL_TIME, 1, 1, 0.4, TransferPart.Centre.OPEN, 3, 0, true, 0, 0, false, 0.05, 0.05, 0.05, false);
        Vehicle fo = fourByFour("open", open, bind("lock"), 1, 1);
        VehicleState so0 = fo.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(fo, so0, 20, s -> Map.of("torque", 50.0));
        assertEquals(0.4 / 0.6, speed(so0, "front", 0) / speed(so0, "rear", 0), 0.01, "open: speeds follow the 40/60 torque split");
        TransferPart.Spec full = new TransferPart.Spec(TransferPart.Kind.FULL_TIME, 1, 1, 0.4, TransferPart.Centre.TORSEN, 3, 0, true, 0, 0, false, 0.05, 0.05, 0.05, false);
        Vehicle ft = fourByFour("full", full, bind("lock"), 1, 1);
        VehicleState sf = ft.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(ft, sf, 20, s -> Map.of("torque", 50.0));
        assertEquals(1, speed(sf, "front", 0) / speed(sf, "rear", 0), 1e-6, "Torsen: within its bias it holds the axles together");
        // on-demand: the rear is driven; the front only through the coupling, so it lags the rear, then follows
        TransferPart.Spec demand = new TransferPart.Spec(TransferPart.Kind.ON_DEMAND, 1, 1, 0.5, TransferPart.Centre.OPEN, 0, 0, false, 20, 800, false, 0.05, 0.05, 0.05, false);
        Vehicle od = fourByFour("demand", demand, bind("couple"), 1, 1);
        VehicleState so = od.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(od, so, 0.5, s -> Map.of("torque", 100.0));
        double lag = speed(so, "rear", 0) - speed(so, "front", 0);
        assertTrue(lag > 0 && speed(so, "front", 0) > 0, "the front follows through the slip-driven coupling: lag " + lag);
        run(od, so, 0.5, s -> Map.of("torque", 100.0, "couple", 1.0));
        assertEquals(speed(so, "rear", 0), speed(so, "front", 0), 1e-6 * speed(so, "rear", 0), "preloaded: locked");
    }
}

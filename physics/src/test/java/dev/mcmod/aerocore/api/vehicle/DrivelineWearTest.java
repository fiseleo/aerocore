package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.ElectricMotor;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.damage.Wear;
import dev.mcmod.aerocore.api.drive.TorqueConverter;
import dev.mcmod.aerocore.api.vehicle.part.MotorPart;
import dev.mcmod.aerocore.api.vehicle.part.TransferPart;
import dev.mcmod.aerocore.api.vehicle.part.TransmissionPart;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 0.42: gearboxes, transfer cases and motors that wear: new they step as ones that do not, to the last bit; clutches wear
 * by their slip work and gears and motors by Wear.running at the torque and speed they run at; a worn clutch holds less,
 * worn gears lose more and at the end drive nothing, a motor's torque goes with its winding; nothing breaks used up.
 */
class DrivelineWearTest {
    private static final double DT = 0.005, INF = Double.POSITIVE_INFINITY;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final VehicleEnvironment SPACE = VehicleEnvironment.calm(-100, FluidState.FRESH_WATER_20C, AIR);
    private static final double[] SIX = {3.6, 2.1, 1.4, 1.0, 0.8, 0.65};
    /** A box's: 5000 h at 150 N m and 300 rad/s, two facings of 200 cm^2 with half a millimetre to wear. */
    private static final TransmissionPart.Wearing TYPICAL = TransmissionPart.Wearing.typical(5000 * 3600.0, 150, 300, 2 * 0.02 * 0.0005);
    /** Wears nothing by itself: for setting a condition and looking at what it does. */
    private static final TransmissionPart.Wearing STILL = new TransmissionPart.Wearing(INF, 150, 300, 1, 3, INF, 0);

    private static TransmissionPart.Spec spec(TransmissionPart.Kind kind) {
        return switch (kind) {
            case MANUAL -> new TransmissionPart.Spec(kind, SIX, -3.5, 0.97, 400, 60, null, 0, 0.3, 0, 0, 0.02, 0.01, 0.01, false);
            case AUTOMATIC -> new TransmissionPart.Spec(kind, SIX, -3.4, 0.95, 600, 0, new TorqueConverter(2.0, 12.29), 600, 0.4, 0, 0, 0.1, 0.03, 0.01, false);
            case DUAL_CLUTCH -> new TransmissionPart.Spec(kind, SIX, -3.5, 0.96, 450, 60, null, 0, 0.15, 0, 0, 0.02, 0.01, 0.01, false);
            case CVT -> new TransmissionPart.Spec(kind, new double[]{2.4, 0.45}, -1, 0.9, 0, 0, new TorqueConverter(2.0, 12.29), 400, 0.4, 0.5, 300, 0.1, 0.03, 0.01, false);
        };
    }

    private static PartDesign box(TransmissionPart.Kind kind, TransmissionPart.Wearing wearing) {
        return new TestParts.Fixture("box", "wear box " + kind + " " + wearing, d -> {
            TransmissionPart box = new TransmissionPart(d, spec(kind), MassProperties.point(40, Vec3d.ZERO), List.of());
            return wearing == null ? box : box.wearing(wearing);
        });
    }

    private static PartDesign load(String name, double inertia, double resistance) {
        return TestParts.gears(name, "wear load " + inertia + " " + resistance, List.of(new Part.Rotor("", Vec3d.X, inertia, false)), List.of(), List.of(), 0, 0,
                s -> s.rotorTorque(0, -resistance * s.rotorSpeed(0), -resistance, true), a -> {});
    }

    /** TransmissionTest's rig: an engine (0.2 kg m^2, "torque") through a part to a load of an inertia and a resistance c x speed, on a body held still. */
    private static Vehicle rig(String key, PartDesign middle, List<String> roles, double inertia, double resistance) {
        List<Binding> bindings = new ArrayList<>();
        for (String r : roles) { bindings.add(Binding.of(r, r)); }
        return new VehicleBlueprint("wear rig " + key, List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.2, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("box", middle, Mount.ORIGIN, bindings),
                new VehicleBlueprint.Entry("load", load("l", inertia, resistance), Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(Connection.Shaft.ports("engine", null, "box", "input", 1, 1), Connection.Shaft.ports("box", "output", "load", null, 1, 1))).compile();
    }

    private static VehicleDynamics.Result step(Vehicle v, VehicleState st, Map<String, Double> controls, int i) {
        st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
        return VehicleDynamics.step(v, st, SPACE, controls, Vec3d.ZERO, List.of(), null, i * DT, DT);
    }

    private static void run(Vehicle v, VehicleState st, double seconds, Map<String, Double> controls) {
        for (int i = 0; i < Math.round(seconds / DT); i++) { step(v, st, controls, i); }
    }

    private static double speed(VehicleState st, String id, int k) { return st.rotorSpeed(st.vehicle().installed(id).firstRotor() + k); }

    private static void spin(VehicleState st, String id, double radPerS) {
        Vehicle.Rotor r = st.vehicle().rotors().get(st.vehicle().installed(id).firstRotor());
        st.setTrainSpeed(r.train(), radPerS / r.ratio());
    }

    private static void finite(VehicleState st) {
        for (int k = 0; k < st.vehicle().rotors().size(); k++) { assertTrue(Double.isFinite(st.rotorSpeed(k)), "rotor " + k); }
        for (double x : st.internals) { assertTrue(Double.isFinite(x)); }
    }

    /** What a driver does with each kind of box over four seconds: off the line, up and down the gears, into park or neutral. */
    private static Map<String, Double> drive(TransmissionPart.Kind kind, double t, VehicleState s) {
        double governed = Math.clamp(5 * (250 - speed(s, "engine", 0)), 0, 180.0);
        return switch (kind) {
            case MANUAL -> {
                double gear = t < 0.3 ? 0 : t < 1.6 ? 1 : t < 2.6 ? 2 : t < 3.3 ? 4 : t < 3.6 ? -1 : 0;
                double pedal = t < 0.3 ? 1 : t < 1.3 ? Math.max(0, 1 - (t - 0.3) / 0.8) : t < 1.6 ? 0 : t < 1.9 ? 1 : t < 2.6 ? 0 : t < 2.75 ? 1 : t < 3.3 ? 0.4 : 0;
                yield Map.of("torque", governed, "gear", gear, "clutch", pedal);
            }
            case AUTOMATIC -> Map.of("torque", t < 3.4 ? 160.0 : 0.0, "gear", t < 0.2 ? -2.0 : t < 1.2 ? 1 : t < 2.0 ? 2 : t < 2.6 ? 3 : t < 3.0 ? 2 : t < 3.4 ? 0 : -1.0,
                    "lockup", t > 1.5 && t < 2.8 ? 0.7 : 0.0);
            case DUAL_CLUTCH -> Map.of("torque", governed, "gear", t < 0.2 ? 0.0 : t < 1.2 ? 1 : t < 1.9 ? 2 : t < 2.6 ? 3 : t < 3.2 ? 2 : t < 3.6 ? -2.0 : -1.0,
                    "clutch", t < 0.8 ? Math.min(1, t / 0.6) : 1.0);
            case CVT -> Map.of("torque", t < 3.2 ? 350.0 : 0.0, "gear", t < 0.2 ? -2.0 : t < 3.2 ? 1 : t < 3.6 ? 0 : -1.0, "ratio", t < 1 ? 2.4 : 0.6,
                    "lockup", t > 0.8 ? 1.0 : 0.0);
        };
    }

    private static List<String> roles(TransmissionPart.Kind kind) {
        return switch (kind) {
            case MANUAL -> List.of("gear", "clutch");
            case AUTOMATIC -> List.of("gear", "lockup");
            case DUAL_CLUTCH -> List.of("gear", "clutch");
            case CVT -> List.of("gear", "ratio", "lockup");
        };
    }

    /** Steps a plain and a wearing vehicle alike, the wearing one made new again after each step; returns the most each condition wore in a step. */
    private static double[] alike(Vehicle plain, Vehicle wearing, int steps, BiFunction<Double, VehicleState, Map<String, Double>> controls) {
        VehicleState a = plain.newState(Vec3d.ZERO, Orientation.IDENTITY, DT), b = wearing.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        int n = wearing.conditions();
        assertEquals(0, plain.conditions());
        double[] most = new double[n];
        for (int i = 0; i < steps; i++) {
            Map<String, Double> c = controls.apply(i * DT, a);
            step(plain, a, c, i);
            step(wearing, b, c, i);
            double[] used = b.usedValues();
            for (int k = 0; k < n; k++) { most[k] = Math.max(most[k], used[k]); }
            b.setUsedValues(new double[n]);
            if (i % 20 == 19 || i == steps - 1) { assertEquals(Json.write(a.toJson()), Json.write(b.toJson()), "step " + i); }
        }
        return most;
    }

    // ----- new: as one that does not wear -----

    @Test void aBoxThatWearsStepsAsOneThatDoesNotWhileNew() {
        for (TransmissionPart.Kind kind : TransmissionPart.Kind.values()) {
            Vehicle plain = rig("plain " + kind, box(kind, null), roles(kind), 4, 0.2), wearing = rig("wearing " + kind, box(kind, TYPICAL), roles(kind), 4, 0.2);
            assertEquals(List.of(), plain.installed("box").part().conditions());
            assertEquals(List.of(TransmissionPart.GEARS, TransmissionPart.CLUTCH), wearing.installed("box").part().conditions());
            assertEquals(List.of("gears", "clutch"), wearing.installed("box").part().conditions());
            double[] most = alike(plain, wearing, 800, (t, s) -> drive(kind, t, s));
            assertTrue(most[0] > 0, kind + ": the gears wore");
            assertTrue(most[1] > 0, kind + ": the friction elements wore");
        }
    }

    // ----- wear by the work done -----

    @Test void aSlippingClutchWearsByItsSlipWork() {
        // in first against a load too heavy to move, the pedal half up: the clutch slips at 200 N m, and with 200 N m from
        // the engine its speed stays at 100 rad/s: 20 kW into the facings
        double volume = 2e-5;
        TransmissionPart.Wearing w = new TransmissionPart.Wearing(INF, 150, 300, 1, 3, volume, Wear.CLUTCH_FACING_M3_PER_J);
        Vehicle v = rig("slip", box(TransmissionPart.Kind.MANUAL, w), List.of("gear", "clutch"), 1e9, 0);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        spin(st, "engine", 100);
        Map<String, Double> controls = Map.of("torque", 200.0, "gear", 1.0, "clutch", 0.5);
        run(v, st, 2, controls);
        assertEquals(100, speed(st, "engine", 0), 0.01, "the engine holds its speed");
        assertEquals(1, st.diagnostics().get("box").get("gear"), 0, "in first");
        double expected = 200 * 100 * 2 * Wear.CLUTCH_FACING_M3_PER_J / volume;
        assertEquals(expected, st.used("box", TransmissionPart.CLUTCH), 0.005 * expected, "torque x slip speed x time x rate / volume");
        assertEquals(0, st.used("box", TransmissionPart.GEARS), 0, "the gears stand: nothing");
        // a clutch that has caught wears no more
        Vehicle light = rig("caught", box(TransmissionPart.Kind.MANUAL, w), List.of("gear", "clutch"), 4, 0.2);
        VehicleState ls = light.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(light, ls, 6, Map.of("torque", 100.0, "gear", 1.0, "clutch", 0.0));
        assertEquals(speed(ls, "box", 0), speed(ls, "box", 1), 1e-6, "locked");
        double caught = ls.used("box", TransmissionPart.CLUTCH);
        run(light, ls, 2, Map.of("torque", 100.0, "gear", 1.0, "clutch", 0.0));
        assertEquals(caught, ls.used("box", TransmissionPart.CLUTCH), 1e-15, "no slip, no wear");
    }

    @Test void gearsWearByTheTorqueAndSpeedTheyRunAt() {
        // fourth (1:1) at a steady speed: the gears carry the engine's torque T at the speed 0.97 T / c
        double life = 1e6, resistance = 0.5, rated = 0.97 * 100 / resistance;
        TransmissionPart.Wearing w = new TransmissionPart.Wearing(life, 100, rated, 1, 3, INF, 0);
        Vehicle v = rig("running", box(TransmissionPart.Kind.MANUAL, w), List.of("gear", "clutch"), 4, resistance);
        double[] rate = new double[2];
        for (int k = 0; k < 2; k++) {
            double torque = k == 0 ? 100 : 200;
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            // the box is put in fourth at once (everything stands), and the wear of the run-up is taken away
            run(v, st, 90, Map.of("torque", torque, "gear", 4.0, "clutch", 0.0));
            st.setUsed("box", TransmissionPart.GEARS_INDEX, 0);
            run(v, st, 2, Map.of("torque", torque, "gear", 4.0, "clutch", 0.0));
            double speed = speed(st, "box", 1);
            assertEquals(0.97 * torque / resistance, speed, 5e-3 * speed, "steady at the gears' efficiency (less the primary shaft's churning, 0.4 %)");
            rate[k] = st.used("box", TransmissionPart.GEARS) / 2;
            // what the gears carry: the engine's torque less the primary shaft's churning (0.002 N m s/rad: 0.4 %)
            double carried = torque - 0.002 * speed;
            assertEquals(Wear.running(1, life, speed / rated, 1, carried / 100, 3), rate[k], 2e-3 * rate[k], "share of the life a second at " + torque + " N m");
        }
        assertEquals(1 / life, rate[0], 0.02 / life, "at its rating: a second of its life a second");
        assertEquals(16, rate[1] / rate[0], 0.3, "twice the torque at twice the speed: 2 x 2^3");
        // out of gear nothing runs
        VehicleState idle = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, idle, 2, Map.of("torque", 20.0, "gear", 0.0, "clutch", 0.0));
        assertEquals(0, idle.used("box", TransmissionPart.GEARS), 0);
    }

    // ----- what wear does -----

    /** The torque a manual box's clutch holds (N m): what slows an engine turning against a load too heavy to move. */
    private static double clutchHolds(double used) {
        Vehicle v = rig("holds", box(TransmissionPart.Kind.MANUAL, STILL), List.of("gear", "clutch"), 1e9, 0);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setUsed("box", TransmissionPart.CLUTCH_INDEX, used);
        spin(st, "engine", 400);
        run(v, st, 0.05, Map.of("torque", 0.0, "gear", 1.0, "clutch", 0.0));
        double before = speed(st, "engine", 0);
        run(v, st, 0.05, Map.of("torque", 0.0, "gear", 1.0, "clutch", 0.0));
        finite(st);
        return (0.2 + 0.02) * (before - speed(st, "engine", 0)) / 0.05;
    }

    @Test void aWornClutchHoldsLessAndSlips() {
        double fresh = clutchHolds(0);
        assertEquals(400, fresh, 1, "the design's capacity");
        assertEquals(1 - 0.85 * 0.25, clutchHolds(0.5) / fresh, 1e-3, "half worn: 0.79 (the clamp load goes late)");
        assertEquals(0.15, clutchHolds(1) / fresh, 1e-3, "worn out");
        assertEquals(0.7875, TransmissionPart.clutchShare(0.5), 1e-12);
        assertEquals(1, TransmissionPart.clutchShare(0), 0);
        // 150 N m through a clutch made for 400: new it locks, worn out (60 N m) it slips
        for (double used : new double[]{0, 1}) {
            Vehicle v = rig("slips", box(TransmissionPart.Kind.MANUAL, STILL), List.of("gear", "clutch"), 4, 0.5);
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setUsed("box", TransmissionPart.CLUTCH_INDEX, used);
            run(v, st, 20, Map.of("torque", 150.0, "gear", 4.0, "clutch", 0.0));
            double slip = speed(st, "box", 0) - speed(st, "box", 1);
            if (used == 0) { assertEquals(0, slip, 1e-6, "new: locked"); } else { assertTrue(slip > 50, "worn out: slipping by " + slip + " rad/s"); }
            finite(st);
        }
        // the other kinds' friction elements alike: an automatic's elements, a dual-clutch box's clutches, a CVT's belt
        for (TransmissionPart.Kind kind : new TransmissionPart.Kind[]{TransmissionPart.Kind.AUTOMATIC, TransmissionPart.Kind.DUAL_CLUTCH, TransmissionPart.Kind.CVT}) {
            double[] ratio = new double[2];
            for (int k = 0; k < 2; k++) {
                Vehicle v = rig("held " + kind, box(kind, STILL), roles(kind), 4, 0.5);
                VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
                st.setUsed("box", TransmissionPart.CLUTCH_INDEX, k);
                // top gear (a CVT: 1:1), locked up, 250 N m: within the new elements' capacity (600, 450, 300 N m), beyond 0.15 of it
                Map<String, Double> controls = kind == TransmissionPart.Kind.CVT ? Map.of("torque", 250.0, "gear", 1.0, "ratio", 1.0, "lockup", 1.0)
                        : Map.of("torque", 250.0, "gear", 4.0, "lockup", 1.0, "clutch", 1.0);
                run(v, st, 30, controls);
                finite(st);
                ratio[k] = speed(st, "engine", 0) / speed(st, "load", 0);
            }
            assertEquals(1, ratio[0], 0.02, kind + " new: holding at 1:1");
            assertTrue(ratio[1] > 1.5, kind + " worn out: the engine runs away from the load: " + ratio[1]);
        }
    }

    /** The load's steady speed (rad/s) behind a part with its gears so used, driven with a torque. */
    private static double steady(Vehicle v, String part, double used, Map<String, Double> controls) {
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setUsed(part, 0, used);
        run(v, st, 90, controls);
        finite(st);
        return speed(st, "load", 0);
    }

    @Test void wornGearsLoseMoreAndUsedUpDriveNothing() {
        for (TransmissionPart.Kind kind : TransmissionPart.Kind.values()) {
            // drive at 1:1 (fourth; a CVT's variator at 1): the load's steady speed is the gears' efficiency x T / c
            Vehicle v = rig("lossy " + kind, box(kind, STILL), roles(kind), 4, 0.5);
            Map<String, Double> controls = kind == TransmissionPart.Kind.CVT ? Map.of("torque", 100.0, "gear", 1.0, "ratio", 1.0, "lockup", 1.0)
                    : Map.of("torque", 100.0, "gear", 4.0, "lockup", 1.0, "clutch", kind == TransmissionPart.Kind.MANUAL ? 0.0 : 1.0);
            double fresh = steady(v, "box", 0, controls);
            assertEquals(spec(kind).efficiency() * 100 / 0.5, fresh, 0.01 * fresh, kind + " new");
            assertEquals(0.95, steady(v, "box", 0.5, controls) / fresh, 2e-3, kind + " half worn: 5 % more lost");
            assertEquals(0.90, steady(v, "box", 0.999, controls) / fresh, 2e-3, kind + " at the end: 10 %");
            assertEquals(0, steady(v, "box", 1, controls), 1e-3, kind + " used up: the load is not driven");
        }
        assertEquals(0.95, TransmissionPart.gearEfficiency(0.5), 1e-12);
        // a transfer case's gears alike
        for (TransferPart.Kind kind : TransferPart.Kind.values()) {
            Vehicle v = fourByFour("lossy " + kind, kind, new TransferPart.Wearing(INF, 100, 100, 1, 3));
            Map<String, Double> controls = Map.of("torque", 100.0, "mode", 1.0, "couple", 1.0, "lock", 1.0);
            double fresh = steady(v, "transfer", 0, controls);
            assertEquals(0.96 * 100 / (2 * 0.5), fresh, 0.01 * fresh, kind + " new: both axles driven");
            assertEquals(0.95, steady(v, "transfer", 0.5, controls) / fresh, 2e-3, kind + " half worn");
            assertEquals(0.90, steady(v, "transfer", 0.999, controls) / fresh, 2e-3, kind + " at the end");
            assertEquals(0, steady(v, "transfer", 1, controls), 1e-3, kind + " used up");
        }
    }

    // ----- transfer cases -----

    private static TransferPart.Spec transferSpec(TransferPart.Kind kind) {
        return switch (kind) {
            case FULL_TIME -> new TransferPart.Spec(kind, 2.72, 0.96, 0.4, TransferPart.Centre.TORSEN, 3, 0, true, 0, 0, false, 0.05, 0.05, 0.05, false);
            case PART_TIME -> new TransferPart.Spec(kind, 2.72, 0.96, 0.5, TransferPart.Centre.OPEN, 0, 0, false, 0, 0, false, 0.05, 0.05, 0.05, false);
            case ON_DEMAND -> new TransferPart.Spec(kind, 1, 0.96, 0.5, TransferPart.Centre.OPEN, 0, 0, false, 20, 800, false, 0.05, 0.05, 0.05, false);
        };
    }

    private static List<String> transferRoles(TransferPart.Kind kind) {
        return switch (kind) { case FULL_TIME -> List.of("range", "lock"); case PART_TIME -> List.of("mode"); case ON_DEMAND -> List.of("range", "couple"); };
    }

    /** An engine through a transfer case to a front and a rear load ("load" is the rear one) of 2 kg m^2 and 0.5 N m s/rad each. */
    private static Vehicle fourByFour(String key, TransferPart.Kind kind, TransferPart.Wearing wearing) {
        PartDesign transfer = new TestParts.Fixture("transfer", "wear transfer " + kind + " " + wearing, d -> {
            TransferPart part = new TransferPart(d, transferSpec(kind), MassProperties.point(30, Vec3d.ZERO), List.of());
            return wearing == null ? part : part.wearing(wearing);
        });
        List<Binding> bindings = new ArrayList<>();
        for (String r : transferRoles(kind)) { bindings.add(Binding.of(r, r)); }
        return new VehicleBlueprint("wear 4x4 " + key, List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.2, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("transfer", transfer, Mount.ORIGIN, bindings),
                new VehicleBlueprint.Entry("front", load("f", 2, 0.5), Mount.ORIGIN), new VehicleBlueprint.Entry("load", load("r", 2, 0.5), Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(Connection.Shaft.ports("engine", null, "transfer", "input", 1, 1), Connection.Shaft.ports("transfer", "front", "front", null, 1, 1),
                        Connection.Shaft.ports("transfer", "rear", "load", null, 1, 1))).compile();
    }

    @Test void aTransferCaseThatWearsStepsAsOneThatDoesNotWhileNewAndWearsByWhatItCarries() {
        TransferPart.Wearing w = TransferPart.Wearing.typical(8000 * 3600.0, 300, 200);
        for (TransferPart.Kind kind : TransferPart.Kind.values()) {
            Vehicle plain = fourByFour("plain " + kind, kind, null), wearing = fourByFour("wearing " + kind, kind, w);
            assertEquals(List.of(), plain.installed("transfer").part().conditions());
            assertEquals(List.of(TransferPart.GEARS), wearing.installed("transfer").part().conditions());
            // low range from rest, then high, the modes and the lock by turns, the coupling preloaded for a while
            double[] most = alike(plain, wearing, 700, (t, s) -> Map.of("torque", t < 3 ? 120.0 : -40.0, "range", t < 0.8 ? 1.0 : 0.0, "mode", t < 0.8 ? 2.0 : t < 1.6 ? 0 : 1.0,
                    "lock", t > 1.2 && t < 2.2 ? 1.0 : 0.0, "couple", t > 1.5 && t < 2.5 ? 0.6 : 0.0));
            assertTrue(most[0] > 0, kind + ": the gears wore");
        }
        // at its rating: 100 N m in at the steady speed the loads give
        double life = 1e6;
        TransferPart.Wearing rated = new TransferPart.Wearing(life, 100, 96, 1, 3);
        Vehicle v = fourByFour("rated", TransferPart.Kind.PART_TIME, rated);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, 90, Map.of("torque", 100.0, "mode", 1.0));
        st.setUsed("transfer", 0, 0);
        run(v, st, 2, Map.of("torque", 100.0, "mode", 1.0));
        assertEquals(96, speed(st, "transfer", 0), 0.2, "0.96 x 100 N m on 2 x 0.5 N m s/rad");
        assertEquals(2 / life * speed(st, "transfer", 0) / 96, st.used("transfer", TransferPart.GEARS), 0.01 * 2 / life, "two seconds of its life");
        assertEquals("gearsUsed", st.diagnostics().get("transfer").keySet().stream().filter(k -> k.endsWith("Used")).findFirst().orElse(""));
    }

    // ----- motors -----

    private static PartDesign motor(MotorPart.Wearing wearing) {
        MassProperties m = MassProperties.point(30, Vec3d.ZERO);
        return new TestParts.Fixture("motor", "wear motor " + wearing, d -> {
            MotorPart motor = new MotorPart(d, new ElectricMotor(200, 20000, 3000), 0.02, 0.9, m, List.of());
            return wearing == null ? motor : motor.wearing(wearing);
        });
    }

    /** A motor (200 N m to 100 rad/s, 20 kW beyond) turning a load of 1 kg m^2 and a resistance, on a body held still. */
    private static Vehicle motorRig(String key, MotorPart.Wearing wearing, double resistance) {
        return new VehicleBlueprint("wear motor " + key, List.of(
                new VehicleBlueprint.Entry("motor", motor(wearing), Mount.ORIGIN, List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("load", load("l", 1, resistance), Mount.ORIGIN),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)), Mount.ORIGIN)),
                List.of(new Connection.Shaft("motor", "load", 1, 1))).compile();
    }

    @Test void aMotorThatWearsStepsAsOneThatDoesNotWhileNewAndItsTorqueGoesWithItsWinding() {
        MotorPart.Wearing typical = MotorPart.Wearing.typical(20000 * 3600.0, 100, 200);
        Vehicle plain = motorRig("plain", null, 4), wearing = motorRig("wearing", typical, 4);
        assertEquals(List.of(MotorPart.MOTOR), wearing.installed("motor").part().conditions());
        double[] most = alike(plain, wearing, 600, (t, s) -> Map.of("throttle", t < 1.5 ? 1.0 : t < 2.2 ? -0.6 : 0.3));
        assertTrue(most[0] > 0, "it wore");
        // at its rating (200 N m at the 50 rad/s the load lets it reach): a second of its life a second
        double life = 1e6;
        Vehicle v = motorRig("rated", new MotorPart.Wearing(life, 50, 200, 1, 3), 4);
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        // (a long life: what it wears while running up must not slow it; taken away again before the two seconds counted)
        run(v, st, 20, Map.of("throttle", 1.0));
        st.setUsed("motor", 0, 0);
        run(v, st, 2, Map.of("throttle", 1.0));
        assertEquals(50, speed(st, "motor", 0), 0.2, "200 N m on 4 N m s/rad");
        assertEquals(2 / life, st.used("motor", MotorPart.MOTOR), 0.02 * 2 / life, "two seconds of its life");
        // half the torque at the same speed: an eighth (the load exponent 3)
        Vehicle half = motorRig("half", new MotorPart.Wearing(life, 25, 200, 1, 3), 4);
        VehicleState hs = half.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(half, hs, 20, Map.of("throttle", 0.5));
        hs.setUsed("motor", 0, 0);
        run(half, hs, 2, Map.of("throttle", 0.5));
        assertEquals(2 / life / 8, hs.used("motor", 0), 0.03 * 2 / life / 8, "100 N m at its rated 25 rad/s");
        // what is left of the winding gives its share of the torque
        MotorPart.Wearing still = new MotorPart.Wearing(INF, 50, 200, 1, 3);
        Vehicle s = motorRig("still", still, 4);
        double fresh = Double.NaN;
        for (double used : new double[]{0, 0.5, 1}) {
            VehicleState ss = s.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            ss.setUsed("motor", 0, used);
            VehicleDynamics.Result r = step(s, ss, Map.of("throttle", 1.0), 0);
            double torque = r.diagnostics().get("motor").get("torqueNm");
            if (used == 0) { fresh = torque; assertEquals(200, torque, 1e-9); }
            assertEquals(1 - used, torque / fresh, 1e-12, "torque with " + used + " used");
            assertEquals(used, r.diagnostics().get("motor").get("motorUsed"), 0);
            run(s, ss, 20, Map.of("throttle", 1.0));
            finite(ss);
            assertEquals((1 - used) * 50, speed(ss, "motor", 0), 0.2, "and the speed it reaches against the load");
        }
        assertTrue(!step(plain, plain.newState(Vec3d.ZERO, Orientation.IDENTITY, DT), Map.of("throttle", 1.0), 0).diagnostics().get("motor").containsKey("motorUsed"));
    }

    // ----- wrapped, and used up -----

    /** A part that wraps a wearing box as a vehicle mod's gearbox does: the box's conditions first, then its own; or no conditions at all. */
    private static PartDesign wrapped(boolean passesConditions) {
        return new TestParts.Fixture("ruled", "wear ruled box " + passesConditions, d -> {
            TransmissionPart inner = new TransmissionPart(d, spec(TransmissionPart.Kind.MANUAL), MassProperties.point(40, Vec3d.ZERO), List.of())
                    .wearing(new TransmissionPart.Wearing(INF, 150, 300, 1, 3, 2e-5, Wear.CLUTCH_FACING_M3_PER_J));
            List<String> conditions = new ArrayList<>(passesConditions ? inner.conditions() : List.of());
            if (passesConditions) { conditions.add("selector"); }
            return new Part() {
                @Override public PartDesign design() { return d; }
                @Override public MassProperties mass() { return inner.mass(); }
                @Override public List<dev.mcmod.aerocore.api.CollisionSphere> collision() { return List.of(); }
                @Override public List<DesignIssue> issues() { return List.of(); }
                @Override public List<Rotor> rotors() { return inner.rotors(); }
                @Override public List<Link> links() { return inner.links(); }
                @Override public List<String> inputs() { return inner.inputs(); }
                @Override public List<String> conditions() { return conditions; }
                @Override public int internalStates() { return inner.internalStates(); }
                @Override public double[] initialInternals() { return inner.initialInternals(); }
                @Override public int shownValues() { return inner.shownValues(); }
                @Override public void loads(PartStep step) { inner.loads(step); }
                @Override public void after(PartAfter step) { inner.after(step); }
            };
        });
    }

    @Test void aWrappedBoxWearsThroughItsWrappersConditions() {
        Map<String, Double> slipping = Map.of("torque", 200.0, "gear", 1.0, "clutch", 0.5);
        // the wrapper lists the box's conditions first: the box reads and wears its own
        Vehicle v = rig("ruled", wrapped(true), List.of("gear", "clutch"), 1e9, 0);
        assertEquals(List.of("gears", "clutch", "selector"), v.installed("box").part().conditions());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        spin(st, "engine", 100);
        run(v, st, 1, slipping);
        assertEquals(200 * 100 * Wear.CLUTCH_FACING_M3_PER_J / 2e-5, st.used("box", "clutch"), 1e-7);
        assertEquals(0, st.used("box", "selector"), 0, "the wrapper's own condition is its own business");
        st.setUsed("box", TransmissionPart.CLUTCH_INDEX, 1);
        VehicleState worn = st;
        double before = speed(worn, "engine", 0);
        run(v, worn, 0.1, slipping);
        assertTrue(speed(worn, "engine", 0) > before + 50, "the worn-out clutch holds 30 N m against the engine's 200: it runs away");
        // a wrapper that lists no conditions (one written before 0.42): the box is new for ever, and what it would wear goes nowhere
        Vehicle none = rig("ruled none", wrapped(false), List.of("gear", "clutch"), 1e9, 0);
        assertEquals(0, none.conditions());
        VehicleState ns = none.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        spin(ns, "engine", 100);
        run(none, ns, 1, slipping);
        assertEquals(100, speed(ns, "engine", 0), 0.01, "holding 200 N m");
        assertEquals(0, ns.worstUsed(), 0);
        assertTrue(ns.diagnostics().get("box").containsKey("clutchUsed"), "the box itself still tells its (new) condition");
    }

    @Test void nothingBreaksWithEverythingUsedUp() {
        for (TransmissionPart.Kind kind : TransmissionPart.Kind.values()) {
            Vehicle v = rig("used up " + kind, box(kind, TYPICAL), roles(kind), 4, 0.2);
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setUsedValues(new double[]{1, 1});
            for (int i = 0; i < 800; i++) { step(v, st, drive(kind, i * DT, st), i); finite(st); }
            assertEquals(0, speed(st, "load", 0), 1e-9, kind + ": nothing reached the load");
            // and with only the friction elements gone
            st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setUsedValues(new double[]{0, 1});
            for (int i = 0; i < 800; i++) { step(v, st, drive(kind, i * DT, st), i); finite(st); }
        }
        for (TransferPart.Kind kind : TransferPart.Kind.values()) {
            Vehicle v = fourByFour("used up " + kind, kind, TransferPart.Wearing.typical(1000, 100, 100));
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setUsedValues(new double[]{1});
            for (int i = 0; i < 600; i++) {
                double t = i * DT;
                step(v, st, Map.of("torque", 150.0, "range", t < 1 ? 1.0 : 0.0, "mode", t < 1 ? 2.0 : 1.0, "lock", t > 2 ? 1.0 : 0.0, "couple", 0.5), i);
                finite(st);
            }
            assertEquals(0, speed(st, "load", 0), 1e-9, kind + ": nothing reached the axles");
        }
        Vehicle m = motorRig("used up", MotorPart.Wearing.typical(100, 50, 200), 4);
        VehicleState ms = m.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        ms.setUsedValues(new double[]{1});
        for (int i = 0; i < 400; i++) { step(m, ms, Map.of("throttle", i < 200 ? 1.0 : -1.0), i); finite(ms); }
        assertEquals(0, speed(ms, "motor", 0), 1e-9, "a burnt-out motor turns nothing");
    }

    @Test void theWearingsNumbersAreChecked() {
        assertThrows(IllegalArgumentException.class, () -> new TransmissionPart.Wearing(0, 150, 300, 1, 3, 1e-5, 1e-14));
        assertThrows(IllegalArgumentException.class, () -> new TransmissionPart.Wearing(1000, 150, 300, 1, 3, 0, 1e-14));
        assertThrows(IllegalArgumentException.class, () -> new TransmissionPart.Wearing(1000, 150, 300, -1, 3, 1e-5, 1e-14));
        assertThrows(IllegalArgumentException.class, () -> new TransferPart.Wearing(1000, 0, 300, 1, 3));
        assertThrows(IllegalArgumentException.class, () -> new MotorPart.Wearing(1000, 100, 200, 1, Double.NaN));
        assertEquals(Wear.CLUTCH_FACING_M3_PER_J, TYPICAL.clutchWearM3PerJ(), 0);
        assertEquals(3, TYPICAL.loadExponent(), 0);
        TransmissionPart box = (TransmissionPart) box(TransmissionPart.Kind.MANUAL, TYPICAL).compile();
        assertEquals(TYPICAL, box.wearing());
        assertEquals(List.of(), box.wearing(null).conditions());
    }
}

package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.InertiaTensor;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.RigidBodyProperties;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** The drivetrain solve (0.24): rows between trains against analytic answers, losses, conservation and robustness. */
class DrivetrainTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final VehicleEnvironment SPACE = VehicleEnvironment.calm(-100, WATER, AIR);
    /** An after() that reports every row's torque as diagnostics "row0", "row1", ... */
    private static final Consumer<PartAfter> ROWS = a -> { for (int i = 0; i < a.rows(); i++) { a.diagnostic("row" + i, a.rowTorque(i)); } };

    private static Part.Rotor rotor(String name, double inertia, boolean driver) { return new Part.Rotor(name, Vec3d.X, inertia, driver); }

    private static VehicleBlueprint.Entry part(String id, PartDesign design) { return new VehicleBlueprint.Entry(id, design, Mount.ORIGIN); }

    /** A body too heavy to move (held still each step by run()). */
    private static VehicleBlueprint.Entry anvil() {
        return part("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(1e7)));
    }

    private static Vehicle vehicle(List<VehicleBlueprint.Entry> parts, List<Connection> connections) {
        return new VehicleBlueprint("drive", parts, connections).compile();
    }

    /** Steps without gravity; held: the body is put back at rest each step. Returns the last result. */
    private static VehicleDynamics.Result run(Vehicle v, VehicleState st, Map<String, Double> controls, int steps, boolean held) {
        VehicleDynamics.Result r = null;
        for (int i = 0; i < steps; i++) {
            if (held) { st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO)); }
            r = VehicleDynamics.step(v, st, SPACE, controls, Vec3d.ZERO, List.of(), null, i * DT, DT);
        }
        return r;
    }

    private static int train(Vehicle v, String id, int k) { return v.rotors().get(v.installed(id).firstRotor() + k).train(); }

    private static double speed(VehicleState st, String id, int k) { return st.rotorSpeed(st.vehicle().installed(id).firstRotor() + k); }

    private static double row(VehicleDynamics.Result r, String id, int i) { return r.diagnostics().get(id).get("row" + i); }

    // ----- the trains alone -----

    @Test void theSolverRepeatsTheTrainsAloneAndTheirFreewheels() {
        for (boolean freewheel : new boolean[]{false, true}) {
            List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(
                    new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.5, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                    new VehicleBlueprint.Entry("rotor", TestParts.flywheel("r", 2.0, false), Mount.at(new Vec3d(0, 1, 0))), anvil()));
            List<Connection> shaft = List.of(new Connection.Shaft("engine", "rotor", 2, 1, freewheel));
            Vehicle alone = vehicle(parts, shaft);
            // the same with a part that only adds an idle row (a brake of no capacity on its own wheel): the solver takes over
            parts.add(part("idle", TestParts.gears("i", "idle hold", List.of(rotor("", 1, false)), List.of(), List.of(), 0, 0, s -> s.hold(0, 0), a -> {})));
            Vehicle solved = vehicle(parts, shaft);
            VehicleState a = alone.newState(Vec3d.ZERO, Orientation.IDENTITY, DT), b = solved.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            double[] torques = {10, -10, 10, -3};
            for (int phase = 0; phase < torques.length; phase++) {
                for (int i = 0; i < 100; i++) {
                    VehicleDynamics.Result ra = run(alone, a, Map.of("torque", torques[phase]), 1, true);
                    VehicleDynamics.Result rb = run(solved, b, Map.of("torque", torques[phase]), 1, true);
                    for (int k = 0; k < 2; k++) {
                        assertEquals(a.rotorSpeed(k), b.rotorSpeed(k), 1e-9 * (1 + Math.abs(a.rotorSpeed(k))), "freewheel " + freewheel + " phase " + phase + " rotor " + k);
                    }
                    assertEquals(ra.load(AppliedForce.Kind.THRUST).torque().x(), rb.load(AppliedForce.Kind.THRUST).torque().x(), 1e-7, "the body's reaction");
                }
            }
            if (freewheel) { assertTrue(b.rotorSpeed(1) > b.rotorSpeed(0) / 2 + 1, "the rotor overran when the engine was braked: " + b.rotorSpeed(1)); }
        }
    }

    // ----- gears and differentials -----

    @Test void gearsCarryTorqueWithTheirEfficiencyAndTheBodyFeelsOnlyTheOutsideTorque() {
        double c = 0.5;
        Vehicle v = vehicle(List.of(new VehicleBlueprint.Entry("box", TestParts.gears("g", "gear 3 0.9 damped 0.5",
                List.of(rotor("input", 0.1, true), rotor("output", 0.3, false)), List.of(new Part.Link(0, 1)), List.of("torque"), 0, 0, s -> {
                    s.rotorTorque(0, s.input("torque", 0), 0, false);
                    s.rotorTorque(1, -c * s.rotorSpeed(1), -c, true);
                    s.gear(0, 1, 3, 0.9);
                }, ROWS), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))), anvil()), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, Map.of("torque", 20.0), 6000, true);
        // power in x 0.9 = power out: 20 x 3 w x 0.9 = 0.5 w^2
        double out = 108;
        assertEquals(out, speed(st, "box", 1), 1e-4 * out, "output speed");
        assertEquals(3 * speed(st, "box", 1), speed(st, "box", 0), 1e-9 * out, "the gear's ratio");
        assertEquals(-20, row(r, "box", 0), 1e-3, "the gear holds the input's torque");
        // steady: the vehicle feels the outside torque on the output (water on a screw), not the gearbox's inside
        assertEquals(-c * out, r.load(AppliedForce.Kind.THRUST).torque().x(), 1e-3 * c * out, "reaction");
    }

    /** An axle: input (driven by "torque"), left and right damped by the outside (c_L, c_R), joined by rows. */
    private static Vehicle axle(String key, double cl, double cr, Consumer<PartStep> rows) {
        return vehicle(List.of(new VehicleBlueprint.Entry("axle", TestParts.gears("a", "axle " + key + " " + cl + " " + cr,
                List.of(rotor("input", 0.05, true), rotor("left", 0.5, false), rotor("right", 0.5, false)), List.of(new Part.Link(0, 1), new Part.Link(0, 2)),
                List.of("torque"), 0, 0, s -> {
                    s.rotorTorque(0, s.input("torque", 0), 0, false);
                    s.rotorTorque(1, -cl * s.rotorSpeed(1), -cl, true);
                    s.rotorTorque(2, -cr * s.rotorSpeed(2), -cr, true);
                    rows.accept(s);
                }, ROWS), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))), anvil()), List.of());
    }

    @Test void differentialsSplitTorqueByTheirSharesWhateverTheSpeeds() {
        for (double share : new double[]{0.5, 0.4}) {
            Vehicle v = axle("open " + share, 1, 2, s -> s.differential(0, 1, 2, 4, share, 1));
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            run(v, st, Map.of("torque", 50.0), 4000, true);
            // the input's 50 N m times the final drive 4 splits share : 1 - share
            double left = 200 * share / 1, right = 200 * (1 - share) / 2;
            assertEquals(left, speed(st, "axle", 1), 1e-4 * left, "left at share " + share);
            assertEquals(right, speed(st, "axle", 2), 1e-4 * right, "right at share " + share);
            assertEquals(4 * (share * left + (1 - share) * right), speed(st, "axle", 0), 1e-4 * left, "the input follows the carrier");
        }
        // locked: both sides alike, the torque going where the load is
        Vehicle v = axle("locked", 1, 2, s -> { s.differential(0, 1, 2, 4, 0.5, 1); s.clutch(1, 2, 1, Double.POSITIVE_INFINITY); });
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, Map.of("torque", 50.0), 4000, true);
        assertEquals(speed(st, "axle", 1), speed(st, "axle", 2), 1e-9 * speed(st, "axle", 1), "locked sides");
        assertEquals(200.0 / 3, speed(st, "axle", 1), 1e-4 * 200 / 3);
        assertEquals(100 - 200.0 / 3, Math.abs(row(r, "axle", 1)), 1e-3, "the lock moves 33 N m from the left to the right");
    }

    @Test void limitedSlipDifferentialsBiasTorqueToTheGrippingSide() {
        // split friction: the left wheel can take 10 N m from the ground, the right 1000; the input is held near 20 rad/s
        record Case(String name, Consumer<PartStep> bias, double right) {}
        double tbr = 3;
        for (Case c : List.of(new Case("open", s -> {}, 10), new Case("torsen", s -> s.biasFriction(1, 2, 0, 0.5 * (tbr - 1) / (tbr + 1), 0), 30),
                new Case("clutch pack", s -> s.biasFriction(1, 2, 4, 0.1, 0), 25))) {
            Vehicle v = vehicle(List.of(part("axle", TestParts.gears("a", "split " + c.name(),
                    List.of(rotor("input", 0.05, true), rotor("left", 0.5, false), rotor("right", 0.5, false)), List.of(), List.of(), 0, 0, s -> {
                        s.rotorTorque(0, -1000 * (s.rotorSpeed(0) - 20), -1000, false);
                        s.differential(0, 1, 2, 1, 0.5, 1);
                        s.hold(1, 10);
                        s.hold(2, 1000);
                        c.bias().accept(s);
                    }, ROWS)), anvil()), List.of());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            VehicleDynamics.Result r = run(v, st, Map.of(), 1000, true);
            assertEquals(0, speed(st, "axle", 2), 1e-9, c.name() + ": the gripping wheel stays");
            assertTrue(speed(st, "axle", 1) > 39, c.name() + ": the other spins: " + speed(st, "axle", 1));
            assertEquals(10, Math.abs(row(r, "axle", 1)), 1e-6, c.name() + ": the spinning side takes its friction");
            assertEquals(c.right(), Math.abs(row(r, "axle", 2)), 1e-6 * c.right(), c.name() + ": the gripping side's torque");
        }
    }

    // ----- clutches and brakes -----

    @Test void aClutchSlipsAtItsCapacityAndLocksWhereMomentumSays() {
        Vehicle v = vehicle(List.of(part("pair", TestParts.gears("p", "clutch 20", List.of(rotor("a", 0.5, false), rotor("b", 1.5, false)),
                List.of(new Part.Link(0, 1)), List.of(), 0, 0, s -> s.clutch(0, 1, 1, 20), ROWS)),
                part("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(100)))), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTrainSpeed(train(v, "pair", 0), 100);
        VehicleDynamics.Result r = run(v, st, Map.of(), 200, false);
        // slipping: 20 N m slows a at 40 rad/s^2 and speeds b at 13.3
        assertEquals(60, speed(st, "pair", 0), 1e-9);
        assertEquals(20 / 1.5, speed(st, "pair", 1), 1e-9);
        assertEquals(-20, row(r, "pair", 0), 1e-9, "slipping at its capacity");
        r = run(v, st, Map.of(), 200, false);
        // locked after 1.875 s at the common speed 50 / 2
        assertEquals(25, speed(st, "pair", 0), 1e-9);
        assertEquals(25, speed(st, "pair", 1), 1e-9);
        assertEquals(0, row(r, "pair", 0), 1e-9, "no torque once locked");
        double energy = 0.5 * 0.5 * Math.pow(speed(st, "pair", 0), 2) + 0.5 * 1.5 * Math.pow(speed(st, "pair", 1), 2);
        assertEquals(2500 - 1875, energy, 1e-6, "the clutch took the analytic 1875 J");
        assertEquals(0, st.body().angularMomentum().length(), 1e-9, "a clutch between rotors on one axis does not turn the body");
    }

    @Test void aBrakeHoldsBelowItsCapacityAndSlidesAbove() {
        Vehicle v = vehicle(List.of(new VehicleBlueprint.Entry("wheel", TestParts.gears("w", "brake 100", List.of(rotor("", 2, true)), List.of(), List.of("torque"), 0, 0,
                s -> { s.rotorTorque(0, s.input("torque", 0), 0, false); s.hold(0, 100); }, ROWS), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))), anvil()),
                List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, Map.of("torque", 80.0), 100, true);
        assertEquals(0, speed(st, "wheel", 0), 1e-12, "held");
        assertEquals(-80, row(r, "wheel", 0), 1e-9);
        assertEquals(0, r.load(AppliedForce.Kind.THRUST).torque().x(), 1e-9, "motor and brake react on the body alike");
        for (double torque : new double[]{150, -150}) {
            st.setTrainSpeed(0, 0);
            r = run(v, st, Map.of("torque", torque), 200, true);
            assertEquals(Math.signum(torque) * 25, speed(st, "wheel", 0), 1e-9, "sliding with the torque over capacity");
            assertEquals(-Math.signum(torque) * 100, row(r, "wheel", 0), 1e-9);
        }
    }

    // ----- smooth couplings -----

    @Test void aViscousCouplingDecaysTheSlipAsBackwardEuler() {
        double c = 2;
        Vehicle v = vehicle(List.of(part("pair", TestParts.gears("p", "viscous 2", List.of(rotor("a", 0.5, false), rotor("b", 1.5, false)),
                List.of(new Part.Link(0, 1)), List.of(), 0, 0, s -> s.viscous(0, 1, 1, c), a -> {})), anvil()), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        st.setTrainSpeed(train(v, "pair", 0), 100);
        run(v, st, Map.of(), 100, true);
        double slip = 100 / Math.pow(1 + DT * c * (1 / 0.5 + 1 / 1.5), 100);
        assertEquals(slip, speed(st, "pair", 0) - speed(st, "pair", 1), 1e-9 * slip);
        assertEquals(50, 0.5 * speed(st, "pair", 0) + 1.5 * speed(st, "pair", 1), 1e-9, "momentum between them");
    }

    @Test void aTorqueConverterStallsWhereItsCapacityMeetsTheEngine() {
        double stall = 150, engine = 200, cp = engine / (stall * stall), ratio = 2.2;
        Vehicle v = vehicle(List.of(part("converter", TestParts.gears("c", "converter", List.of(rotor("pump", 0.2, true), rotor("turbine", 0.1, false)),
                List.of(new Part.Link(0, 1)), List.of(), 0, 0, s -> {
                    double w = s.rotorSpeed(0);
                    s.rotorTorque(0, engine, 0, false);
                    // at stall the pump absorbs cp w^2 and the turbine gets ratio times it
                    s.torquePair(0, 1, -cp * w * Math.abs(w), ratio * cp * w * Math.abs(w), -2 * cp * Math.abs(w), 0, 2 * ratio * cp * Math.abs(w), 0);
                    s.hold(1, Double.POSITIVE_INFINITY);
                }, ROWS)), anvil()), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, Map.of(), 2000, true);
        assertEquals(stall, speed(st, "converter", 0), 1e-6 * stall, "stall speed");
        assertEquals(0, speed(st, "converter", 1), 1e-12, "turbine held");
        assertEquals(-ratio * engine, row(r, "converter", 0), 1e-6 * engine, "the brake holds the multiplied torque");
        assertEquals(0, r.load(AppliedForce.Kind.THRUST).torque().x(), 1e-6 * engine, "engine, stator and brake balance on the body");
    }

    // ----- conservation and robustness -----

    /** A part with four rotors and random rows between them (no driving torques unless torques). */
    private static PartDesign randomGears(long seed, boolean torques) {
        Random random = new Random(seed);
        List<Part.Rotor> rotors = new ArrayList<>();
        for (int k = 0; k < 4; k++) { rotors.add(rotor("r" + k, 0.1 + 2 * random.nextDouble(), false)); }
        long rowsSeed = random.nextLong();
        return TestParts.gears("x", "random " + seed + " " + torques, rotors, List.of(), List.of(), 0, 0, s -> {
            Random r = new Random(rowsSeed);
            int rows = 2 + r.nextInt(5), diff = -1;
            for (int i = 0; i < rows; i++) {
                int a = r.nextInt(4), b = (a + 1 + r.nextInt(3)) % 4, c = (b + 1) % 4 == a ? (b + 2) % 4 : (b + 1) % 4;
                double ratio = (r.nextBoolean() ? 1 : -1) * (0.3 + 3 * r.nextDouble()), efficiency = 0.7 + 0.3 * r.nextDouble();
                switch (r.nextInt(6)) {
                    case 0 -> s.gear(a, b, ratio, efficiency);
                    case 1 -> s.clutch(a, b, ratio, r.nextBoolean() ? Double.POSITIVE_INFINITY : 200 * r.nextDouble());
                    case 2 -> s.hold(a, r.nextBoolean() ? 0 : 100 * r.nextDouble());
                    case 3 -> diff = s.differential(a, b, c, ratio, 0.1 + 0.8 * r.nextDouble(), efficiency);
                    case 4 -> { if (diff >= 0) { s.biasFriction(b, c, 10 * r.nextDouble(), r.nextDouble(), diff); } else { s.freewheel(a, b, ratio, efficiency, r.nextBoolean() ? 1 : -1); } }
                    default -> s.viscous(a, b, ratio, 5 * r.nextDouble());
                }
            }
            if (torques) {
                for (int k = 0; k < 4; k++) { s.rotorTorque(k, 1000 * (r.nextDouble() - 0.5), 50 * (r.nextDouble() - 0.6), r.nextBoolean()); }
                s.torquePair(0, 3, 100 * r.nextDouble(), -100 * r.nextDouble(), -r.nextDouble(), r.nextDouble(), r.nextDouble(), -r.nextDouble());
            }
        }, a -> {});
    }

    @Test void withoutDrivingTorquesTheRowsOnlyTakeEnergy() {
        for (long seed = 0; seed < 40; seed++) {
            Vehicle v = vehicle(List.of(part("x", randomGears(seed, false)), anvil()), List.of());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            Random random = new Random(seed + 1000);
            for (int t = 0; t < v.trains(); t++) { st.setTrainSpeed(t, 200 * (random.nextDouble() - 0.5)); }
            double energy = energy(v, st);
            for (int i = 0; i < 200; i++) {
                run(v, st, Map.of(), 1, true);
                double next = energy(v, st);
                assertTrue(next <= energy * (1 + 1e-9) + 1e-9, "seed " + seed + " step " + i + ": " + energy + " -> " + next);
                energy = next;
            }
        }
    }

    private static double energy(Vehicle v, VehicleState st) {
        double e = 0;
        for (int k = 0; k < v.rotors().size(); k++) { e += 0.5 * v.rotors().get(k).inertiaKgM2() * st.rotorSpeed(k) * st.rotorSpeed(k); }
        return e;
    }

    @Test void shiftingKeepsTheVehiclesAngularMomentum() {
        for (double ratio : new double[]{2, -1}) {
            Vehicle v = vehicle(List.of(part("box", TestParts.gears("g", "shift " + ratio, List.of(rotor("input", 0.2, false), rotor("output", 0.6, false)),
                    List.of(new Part.Link(0, 1)), List.of(), 0, 0, s -> s.clutch(0, 1, ratio, 50), a -> {})),
                    part("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(50)))), List.of());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setTrainSpeed(train(v, "box", 0), 300);
            Vec3d before = total(v, st);
            run(v, st, Map.of(), 400, false);
            assertEquals(speed(st, "box", 0), ratio * speed(st, "box", 1), 1e-9 * 300, "engaged at ratio " + ratio);
            assertTrue(st.body().angularMomentum().length() > 1, "the case's reaction turned the body");
            assertEquals(0, total(v, st).subtract(before).length(), 1e-9 * before.length(), "body and rotors together");
        }
    }

    private static Vec3d total(Vehicle v, VehicleState st) {
        Vec3d spin = Vec3d.ZERO;
        for (int k = 0; k < v.rotors().size(); k++) { spin = spin.add(v.rotors().get(k).axis().scale(v.rotors().get(k).momentumKgM2() * st.rotorSpeed(k))); }
        return st.body().angularMomentum().add(st.body().orientation().rotate(spin));
    }

    @Test void redundantAndContradictoryLocksHoldWithoutFailing() {
        Vehicle v = vehicle(List.of(part("box", TestParts.gears("g", "redundant", List.of(rotor("a", 0.5, true), rotor("b", 0.5, false)), List.of(new Part.Link(0, 1)),
                List.of(), 0, 0, s -> {
                    s.rotorTorque(0, 100, 0, false);
                    s.gear(0, 1, 2, 1);
                    s.gear(0, 1, 2, 1);
                    s.hold(0, Double.POSITIVE_INFINITY);
                    s.hold(1, Double.POSITIVE_INFINITY);
                }, ROWS)), anvil()), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleDynamics.Result r = run(v, st, Map.of(), 50, true);
        assertEquals(0, speed(st, "box", 0), 1e-9);
        assertEquals(0, speed(st, "box", 1), 1e-9);
        assertEquals(0, 100 + row(r, "box", 0) + row(r, "box", 1) + row(r, "box", 2), 1e-4, "the rows hold a's 100 N m between them");
        assertEquals(0, -2 * row(r, "box", 0) - 2 * row(r, "box", 1) + row(r, "box", 3), 1e-4, "and b's share");
        Vehicle bound = vehicle(List.of(part("box", TestParts.gears("g", "contradictory", List.of(rotor("a", 0.5, true), rotor("b", 0.5, false)),
                List.of(new Part.Link(0, 1)), List.of(), 0, 0, s -> { s.rotorTorque(0, 100, 0, false); s.gear(0, 1, 2, 1); s.gear(0, 1, 3, 1); }, a -> {})),
                anvil()), List.of());
        VehicleState jammed = bound.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(bound, jammed, Map.of(), 50, true);
        assertEquals(0, speed(jammed, "box", 0), 1e-9, "gears of two ratios at once jam");
    }

    @Test void randomDrivetrainsNeverThrowOrGoNonFinite() {
        int steps = 0, fallbacks = 0;
        for (long seed = 0; seed < 60; seed++) {
            Vehicle v = vehicle(List.of(part("x", randomGears(seed, true)), anvil()), List.of());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            for (int i = 0; i < 100; i++) {
                st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                VehicleDynamics.Prepared prepared = VehicleDynamics.prepare(v, st, SPACE, Map.of(), Vec3d.ZERO, List.of(), i * DT, DT);
                VehicleDynamics.finish(prepared, prepared.damped(), dev.mcmod.aerocore.api.Wrench.ZERO, List.of(), 0, null);
                steps++;
                if (prepared.drive().fallback()) { fallbacks++; }
                for (int k = 0; k < v.rotors().size(); k++) { assertTrue(Double.isFinite(st.rotorSpeed(k)), "seed " + seed + " step " + i); }
            }
        }
        // the pivoting finishes nearly always; Gauss-Seidel is the exception (contradictory random rows)
        assertTrue(fallbacks < 0.02 * steps, fallbacks + " fallbacks in " + steps + " steps");
    }

    // ----- rolling forces and line dampers -----

    @Test void aRollingForceSpinsTheWheelUpAsTheBodySlows() {
        double k = 500, radius = 0.3, inertia = 1, v0 = 10;
        for (boolean solver : new boolean[]{false, true}) {
            // a linear "tyre" on a wheel spinning about z at the COM: the contact point (0, -R, 0) slips at u = v + w R, the force
            // -k u along x acts there and R times it on the wheel
            Vec3d contact = new Vec3d(0, -radius, 0);
            Vehicle v = vehicle(List.of(part("wheel", TestParts.gears("w", "rolling " + solver, List.of(new Part.Rotor("", Vec3d.Z, inertia, false)), List.of(),
                    List.of(), 0, 0, s -> {
                        double vx = s.velocity(contact).x(), slip = vx + s.rotorSpeed(0) * radius;
                        s.rollingForce(0, s.offset(contact), Vec3d.X, -k * slip, -k * radius, radius);
                        s.damper(s.offset(contact), Vec3d.X, k, vx);
                        if (solver) { s.hold(0, 0); }
                    }, a -> {})),
                    part("body", TestParts.box("b", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(100)))), List.of());
            VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
            st.setBody(new RigidBodyState(st.body().position(), new Vec3d(v0, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
            run(v, st, Map.of(), 400, false);
            double m = v.properties().massKg(), rolling = m * v0 / (m + inertia / (radius * radius));
            Vec3d at = st.body().velocity().add(st.body().angularVelocity(v.properties()).cross(st.body().worldOffset(v.properties(), contact)));
            assertEquals(0, at.x() + speed(st, "wheel", 0) * radius, 1e-6, "rolling at the end");
            assertEquals(rolling, st.body().velocity().x(), 0.005 * rolling, "the wheel's spin-up took the body's momentum (solver " + solver + ")");
        }
    }

    @Test void aLineDamperActsAlongItsLineOnly() {
        RigidBodyProperties p = new RigidBodyProperties(10, Vec3d.ZERO, InertiaTensor.box(10, new Vec3d(1, 1, 1)));
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, new Vec3d(2, 3, 0), Orientation.IDENTITY, Vec3d.ZERO);
        double c = 20, dt = 0.01;
        RigidBodyState next = RigidBodyDynamics.implicitVelocity(s, p, dev.mcmod.aerocore.api.Wrench.ZERO, null, List.of(), List.of(),
                List.of(new RigidBodyDynamics.LineDamper(Vec3d.ZERO, Vec3d.X, c, 0.5)), dt);
        assertEquals((10 * 2 + c * dt * 0.5) / (10 + c * dt), next.velocity().x(), 1e-12, "implicit along x toward 0.5 m/s");
        assertEquals(3, next.velocity().y(), 1e-12, "nothing across it");
        // off the COM it also turns the body, and the point's speed along the line comes closer to the reference
        RigidBodyState turned = RigidBodyDynamics.implicitVelocity(s, p, dev.mcmod.aerocore.api.Wrench.ZERO, null, List.of(), List.of(),
                List.of(new RigidBodyDynamics.LineDamper(new Vec3d(0, 0, 1), Vec3d.X, c, 0.5)), dt);
        Vec3d point = turned.velocity().add(turned.angularVelocity(p).cross(new Vec3d(0, 0, 1)));
        assertTrue(Math.abs(turned.angularVelocity(p).y()) > 0, "yaw from a damper off the COM");
        assertTrue(point.x() < 2 && point.x() > 0.5, "the point slows toward 0.5 m/s: " + point.x());
        dev.mcmod.aerocore.api.Wrench load = new dev.mcmod.aerocore.api.Wrench(new Vec3d(5, -3, 1), new Vec3d(0.2, 1, -2));
        List<RigidBodyDynamics.PointDrag> drags = List.of(new RigidBodyDynamics.PointDrag(new Vec3d(1, 0, 0.5), Vec3d.ZERO, 3, 2));
        assertEquals(RigidBodyDynamics.implicitVelocity(s, p, load, null, drags, List.of(), dt),
                RigidBodyDynamics.implicitVelocity(s, p, load, null, drags, List.of(), List.of(), dt), "without dampers the old update exactly");
    }

    // ----- ports, links, after -----

    /** A gearbox with named input and output joined by its own clutch, and an engine (flywheel driving "torque"). */
    private static List<VehicleBlueprint.Entry> gearbox() {
        return List.of(new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 0.2, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                part("box", TestParts.gears("g", "ported box", List.of(rotor("input", 0.05, false), rotor("output", 0.1, false)), List.of(new Part.Link(0, 1)),
                        List.of(), 0, 0, s -> s.clutch(0, 1, 3, 500), a -> {})),
                part("shaft", TestParts.flywheel("s", 0.3, false)), anvil());
    }

    @Test void shaftsJoinTheRotorsTheirPortsName() {
        Vehicle v = vehicle(gearbox(), List.of(Connection.Shaft.ports("engine", null, "box", "input", 1, 1), Connection.Shaft.ports("box", "output", "shaft", null, 1, 1)));
        assertEquals(2, v.trains());
        assertEquals(train(v, "engine", 0), train(v, "box", 0), "the engine turns the input");
        assertEquals(train(v, "shaft", 0), train(v, "box", 1), "the output turns the shaft");
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, Map.of("torque", 30.0), 200, true);
        assertEquals(speed(st, "box", 0), 3 * speed(st, "shaft", 0), 1e-9 * speed(st, "box", 0), "through the gearbox's clutch at 3:1");

        DesignException unknown = assertThrows(DesignException.class, () -> vehicle(gearbox(), List.of(Connection.Shaft.ports("engine", null, "box", "drive", 1, 1))));
        assertTrue(unknown.getMessage().contains("no rotor drive"), unknown.getMessage());
        DesignException looped = assertThrows(DesignException.class, () -> vehicle(gearbox(),
                List.of(Connection.Shaft.ports("engine", null, "box", "input", 1, 1), Connection.Shaft.ports("engine", null, "box", "output", 1, 1))));
        assertTrue(looped.getMessage().contains("which the part couples itself"), looped.getMessage());
    }

    @Test void portsGoToJsonOnlyWhenNamed() {
        Connection plain = new Connection.Shaft("a", "b", 2, 0.97), named = Connection.Shaft.ports("a", "output", "b", null, 2, 0.97);
        assertFalse(plain.toJson().containsKey("fromPort"));
        assertFalse(named.toJson().containsKey("toPort"));
        assertEquals("output", named.toJson().get("fromPort"));
        assertEquals(plain, Connection.fromJson(Fields.of(Json.parse(Json.pretty(plain.toJson())), "c")));
        assertEquals(named, Connection.fromJson(Fields.of(Json.parse(Json.pretty(named.toJson())), "c")));
    }

    @Test void afterSeesTheRowsAndKeepsStatesAndShownValues() {
        Vehicle v = vehicle(List.of(part("brake", TestParts.gears("b", "after", List.of(rotor("", 1, true)), List.of(), List.of(), 1, 2, s -> {
            s.rotorTorque(0, 3, 0, false);
            s.hold(0, 5);
            s.show(0, 1.5);
            s.carry("step " + s.time());
        }, a -> {
            assertEquals("step " + a.time(), a.carried());
            assertEquals(1, a.rows());
            a.setInternal(0, a.internal(0) + a.rowTorque(0));
            a.show(1, a.rotorSpeed(0) + 7);
        })), anvil()), List.of());
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        run(v, st, Map.of(), 10, true);
        assertEquals(-30, st.internal("brake", 0), 1e-9, "ten steps of the brake's -3 N m");
        assertEquals(1.5, st.shown("brake", 0), 0);
        assertEquals(7, st.shown("brake", 1), 1e-12);
        assertEquals(2, st.shownValues().length);
    }
}

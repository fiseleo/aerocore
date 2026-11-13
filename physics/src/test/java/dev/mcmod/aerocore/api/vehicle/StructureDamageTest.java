package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.PropellerModels;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Rudder;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.aero.AirPropeller;
import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.HelicopterRotor;
import dev.mcmod.aerocore.api.aero.SectionPolar;
import dev.mcmod.aerocore.api.aero.Wing;
import dev.mcmod.aerocore.api.body.BodyModule;
import dev.mcmod.aerocore.api.body.BodyShape;
import dev.mcmod.aerocore.api.body.SectionShape;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.vehicle.part.AirPropellerPart;
import dev.mcmod.aerocore.api.vehicle.part.BodyPart;
import dev.mcmod.aerocore.api.vehicle.part.ControlSurfacePart;
import dev.mcmod.aerocore.api.vehicle.part.FramePart;
import dev.mcmod.aerocore.api.vehicle.part.PropellerPart;
import dev.mcmod.aerocore.api.vehicle.part.RotorPart;
import dev.mcmod.aerocore.api.vehicle.part.WingPart;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 0.42: parts the game damages (wings, control surfaces, propellers and rotors, bodies, frames; their "structure" or
 * "blades"): undamaged they step as ones that have no condition, to the last bit; damaged they lose what their rule
 * says; nothing breaks with everything used up. And a body's cargo holds.
 */
class StructureDamageTest {
    private static final double DT = 0.005, G = TestTrack.G;
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-G);
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final VehicleEnvironment SKY = VehicleEnvironment.calm(-1000, WATER, AIR), SEA = VehicleEnvironment.calm(0, WATER, AIR);

    /**
     * Steps a plain vehicle and one with conditions alike from like states, the second made new again after each step,
     * and checks that their saved states stay the same to the last bit.
     */
    private static void alike(Vehicle plain, Vehicle wearing, VehicleState a, VehicleState b, VehicleEnvironment env, Vec3d gravity, int steps,
                              IntFunction<Map<String, Double>> controls, java.util.function.Consumer<VehicleState> before) {
        assertEquals(0, plain.conditions());
        assertTrue(wearing.conditions() > 0);
        for (int i = 0; i < steps; i++) {
            Map<String, Double> c = controls.apply(i);
            before.accept(a);
            before.accept(b);
            VehicleDynamics.step(plain, a, env, c, gravity, List.of(), null, i * DT, DT);
            VehicleDynamics.step(wearing, b, env, c, gravity, List.of(), null, i * DT, DT);
            b.setUsedValues(new double[wearing.conditions()]);
            if (i % 20 == 19 || i == steps - 1) { assertEquals(Json.write(a.toJson()), Json.write(b.toJson()), "step " + i); }
        }
    }

    private static void finite(VehicleState st) {
        RigidBodyState b = st.body();
        for (Vec3d x : new Vec3d[]{b.position(), b.velocity(), b.angularMomentum()}) {
            assertTrue(Double.isFinite(x.x()) && Double.isFinite(x.y()) && Double.isFinite(x.z()), "finite: " + x);
        }
        for (double x : st.internals) { assertTrue(Double.isFinite(x)); }
        for (int k = 0; k < st.vehicle().rotors().size(); k++) { assertTrue(Double.isFinite(st.rotorSpeed(k))); }
    }

    private static void moving(VehicleState st, Vec3d velocity) { st.setBody(new RigidBodyState(st.body().position(), velocity, st.body().orientation(), Vec3d.ZERO)); }

    private static void spin(VehicleState st, String id, double radPerS) {
        Vehicle.Rotor r = st.vehicle().rotors().get(st.vehicle().installed(id).firstRotor());
        st.setTrainSpeed(r.train(), radPerS / r.ratio());
    }

    // ----- wing -----

    /** TestParts' wing (10 m x 1.2 m, NACA 2412, ailerons), damageable or not. */
    private static PartDesign wing(String name, WingPart.Wearing wearing) {
        Airfoil a = Airfoil.naca4(0.02, 0.4, 0.12);
        MassProperties m = new MassProperties(100, Vec3d.ZERO, 100 * 100 / 12.0, 100 * 100 / 12.0, 100 * 1.2 * 1.2 / 12, 0, 0, 0);
        return new TestParts.Fixture(name, "damage wing " + wearing, d -> {
            WingPart w = new WingPart(d, new Wing.Planform(10, 1.2, 1.2, 0, 0, 0, 0, true), a, a,
                    List.of(new Wing.ControlSurface("aileron", SectionPolar.FlapType.PLAIN, 0.25, 0.6, 0.95, 20, 20, true)), 8, m, List.of());
            return wearing == null ? w : w.wearing(wearing);
        });
    }

    private static Vehicle glider(String name, WingPart.Wearing wearing) {
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("wing", wing("w", wearing), new Mount(Vec3d.ZERO, 0, 4, 0), List.of(Binding.of("aileron", "roll"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 1, 1), 0, false, 1, MassSpec.given(300)), Mount.ORIGIN)), List.of()).compile();
    }

    @Test void aDamagedWingLiftsLessAndDragsMore() {
        Vehicle plain = glider("plain glider", null), wearing = glider("glider", WingPart.Wearing.typical());
        assertEquals(List.of(WingPart.STRUCTURE), wearing.installed("wing").part().conditions());
        VehicleState a = plain.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT), b = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        moving(a, new Vec3d(40, 0, 0));
        moving(b, new Vec3d(40, 0, 0));
        alike(plain, wearing, a, b, SKY, GRAVITY, 300, i -> Map.of("roll", 0.6 * Math.sin(0.02 * i)), s -> {});
        assertTrue(!a.diagnostics().get("wing").containsKey("structureUsed") && b.diagnostics().get("wing").containsKey("structureUsed"));
        // one step at 40 m/s with some aileron: lift x (1 - 0.7 u), drag x (1 + 2 u), the rolling moment with the lift
        double[] lift = new double[3], drag = new double[3], roll = new double[3], liftN = new double[3], dragN = new double[3];
        double[] used = {0, 0.5, 1};
        for (int k = 0; k < 3; k++) {
            VehicleState st = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
            moving(st, new Vec3d(40, 0, 0));
            st.setUsed("wing", 0, used[k]);
            VehicleDynamics.Result r = VehicleDynamics.step(wearing, st, SKY, Map.of("roll", 0.5), GRAVITY, List.of(), null, 0, DT);
            Wrench w = r.load(AppliedForce.Kind.LIFT);
            lift[k] = w.force().y(); drag[k] = -w.force().x(); roll[k] = w.torque().x();
            liftN[k] = r.diagnostics().get("wing").get("liftN"); dragN[k] = r.diagnostics().get("wing").get("dragN");
            assertEquals(used[k], r.diagnostics().get("wing").get("structureUsed"), 0);
            finite(st);
        }
        assertTrue(lift[0] > 5000 && drag[0] > 100 && roll[0] > 100, "a wing at 4 degrees with aileron: lift " + lift[0] + " drag " + drag[0] + " roll " + roll[0]);
        assertEquals(0.65, lift[1] / lift[0], 1e-9, "half damaged: lift");
        assertEquals(2.0, drag[1] / drag[0], 1e-9, "half damaged: drag");
        assertEquals(0.65, roll[1] / roll[0], 1e-9, "half damaged: the ailerons' roll");
        assertEquals(0.30, lift[2] / lift[0], 1e-9, "wrecked: lift");
        assertEquals(3.0, drag[2] / drag[0], 1e-9, "wrecked: drag");
        assertEquals(0.30, liftN[2] / liftN[0], 1e-9);
        assertEquals(3.0, dragN[2] / dragN[0], 1e-9);
        assertThrows(IllegalArgumentException.class, () -> new WingPart.Wearing(1.5, 2));
        assertThrows(IllegalArgumentException.class, () -> new WingPart.Wearing(0.5, -1));
    }

    // ----- control surface -----

    private static PartDesign rudder(String name, ControlSurfacePart.Wearing wearing) {
        MassProperties m = new MassProperties(12, Vec3d.ZERO, 0.1, 0.05, 0.12, 0, 0, 0);
        return new TestParts.Fixture(name, "damage rudder " + wearing, d -> {
            ControlSurfacePart s = new ControlSurfacePart(d, 0.22, 0.32, 35, 0.15, Rudder.lowAspectPolar(0.22, 0.32), m, List.of());
            return wearing == null ? s : s.wearing(wearing);
        });
    }

    private static Vehicle ruddered(String name, ControlSurfacePart.Wearing wearing) {
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("rudder", rudder("r", wearing), Mount.at(new Vec3d(-2, -0.3, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 0.5, 1), 0, false, 1, MassSpec.given(1000)), Mount.ORIGIN)), List.of()).compile();
    }

    /** The rudder's load in one step at 3 m/s under water with an input and so much of it used. */
    private static VehicleDynamics.Result ruddering(Vehicle v, double input, double used) {
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        moving(st, new Vec3d(3, 0, 0.2));
        if (used > 0) { st.setUsed("rudder", 0, used); }
        return VehicleDynamics.step(v, st, VehicleEnvironment.calm(1, WATER, AIR), Map.of("rudder", input), Vec3d.ZERO, List.of(), null, 0, DT);
    }

    @Test void aDamagedControlSurfaceAnswersLessAndJamsAtNeutral() {
        Vehicle plain = ruddered("plain rudder", null), wearing = ruddered("rudder", ControlSurfacePart.Wearing.typical());
        assertEquals(List.of(ControlSurfacePart.STRUCTURE), wearing.installed("rudder").part().conditions());
        VehicleState a = plain.newState(Vec3d.ZERO, Orientation.IDENTITY, DT), b = wearing.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        moving(a, new Vec3d(3, 0, 0));
        moving(b, new Vec3d(3, 0, 0));
        alike(plain, wearing, a, b, VehicleEnvironment.calm(1, WATER, AIR), Vec3d.ZERO, 300, i -> Map.of("rudder", Math.sin(0.03 * i)), s -> {});
        VehicleDynamics.Result full = ruddering(wearing, 1, 0), half = ruddering(wearing, 1, 0.5), jammed = ruddering(wearing, 1, 1);
        assertEquals(35, full.diagnostics().get("rudder").get("deflectionDeg"), 1e-9);
        assertEquals(17.5, half.diagnostics().get("rudder").get("deflectionDeg"), 1e-9, "half its authority");
        assertEquals(0, jammed.diagnostics().get("rudder").get("deflectionDeg"), 0, "jammed at neutral");
        // half damaged at full input is the undamaged surface at half input; jammed, the surface with no input at all
        Vec3d halfForce = half.load(AppliedForce.Kind.LIFT).force(), sameAngle = ruddering(wearing, 0.5, 0).load(AppliedForce.Kind.LIFT).force();
        assertEquals(0, halfForce.subtract(sameAngle).length(), 1e-9 * sameAngle.length());
        Vec3d neutral = ruddering(wearing, 0, 0).load(AppliedForce.Kind.LIFT).force();
        assertEquals(0, jammed.load(AppliedForce.Kind.LIFT).force().subtract(neutral).length(), 1e-12 * Math.max(neutral.length(), 1));
        assertTrue(Math.abs(full.load(AppliedForce.Kind.LIFT).force().z()) > 3 * Math.abs(neutral.z()) && neutral.length() > 0,
                "it still carries its fin's load: " + neutral + " against " + full.load(AppliedForce.Kind.LIFT).force());
        assertThrows(IllegalArgumentException.class, () -> new ControlSurfacePart.Wearing(1.2));
    }

    // ----- air propeller -----

    /** NACA Report 640's three-blade propeller roughly (RotorAndPropellerTest), fixed pitch. */
    private static PartDesign airscrew(String name, AirPropellerPart.Wearing wearing) {
        double pitch = Math.PI * 0.75 * Math.tan(Math.toRadians(25));
        AirPropeller.Geometry g = new AirPropeller.Geometry(3, 3.05, 0.2, 0.07, 0.04, pitch, 0, Airfoil.naca4(0.035, 0.3, 0.20), Airfoil.naca4(0.035, 0.3, 0.08));
        MassProperties m = new MassProperties(60, Vec3d.ZERO, 12, 6, 6, 0, 0, 0);
        return new TestParts.Fixture(name, "damage airscrew " + wearing, d -> {
            AirPropellerPart p = new AirPropellerPart(d, g, 0, 0, m, List.of());
            return wearing == null ? p : p.wearing(wearing);
        });
    }

    private static Vehicle tractor(String name, AirPropellerPart.Wearing wearing) {
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 1.0, true), Mount.ORIGIN, List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("prop", airscrew("p", wearing), Mount.at(new Vec3d(2, 0, 0))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 1, 1), 0, false, 1, MassSpec.given(1500)), Mount.ORIGIN)),
                List.of(new Connection.Shaft("engine", "prop", 1, 1))).compile();
    }

    @Test void damagedPropellerBladesPushLessForTheSameTorque() {
        Vehicle plain = tractor("plain tractor", null), wearing = tractor("tractor", AirPropellerPart.Wearing.typical());
        assertEquals(List.of(AirPropellerPart.BLADES), wearing.installed("prop").part().conditions());
        VehicleState a = plain.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT), b = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        for (VehicleState s : new VehicleState[]{a, b}) { moving(s, new Vec3d(30, 0, 0)); spin(s, "prop", 120); }
        alike(plain, wearing, a, b, SKY, GRAVITY, 200, i -> Map.of("torque", 1500.0), s -> {});
        double[] thrust = new double[3], thrustN = new double[3], torque = new double[3], used = {0, 0.5, 1};
        for (int k = 0; k < 3; k++) {
            VehicleState st = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
            moving(st, new Vec3d(30, 0, 0));
            spin(st, "prop", 150);
            st.setUsed("prop", 0, used[k]);
            VehicleDynamics.Result r = VehicleDynamics.step(wearing, st, SKY, Map.of("torque", 0.0), Vec3d.ZERO, List.of(), null, 0, DT);
            thrust[k] = r.load(AppliedForce.Kind.THRUST).force().x();
            thrustN[k] = r.diagnostics().get("prop").get("thrustN"); torque[k] = r.diagnostics().get("prop").get("torqueNm");
            assertEquals(used[k], r.diagnostics().get("prop").get("bladesUsed"), 0);
        }
        assertTrue(thrust[0] > 2000 && torque[0] > 500, "pulling: " + thrust[0] + " N for " + torque[0] + " N m");
        assertEquals(0.7, thrust[1] / thrust[0], 1e-9, "half damaged");
        assertEquals(0.1, thrust[2] / thrust[0], 1e-9, "stubs");
        assertEquals(0.7, thrustN[1] / thrustN[0], 1e-9);
        assertEquals(torque[0], torque[1], 0, "the shaft's load does not fall");
        assertEquals(torque[0], torque[2], 0);
        AirPropellerPart.Wearing w = AirPropellerPart.Wearing.typical();
        assertEquals(1, w.thrustShare(0), 0);
        assertEquals(0.4, w.thrustShare(0.999999999), 1e-6, "falling to 0.4 as they come to their end");
        assertEquals(0.1, w.thrustShare(1), 0);
        assertThrows(IllegalArgumentException.class, () -> new AirPropellerPart.Wearing(0.6, 1.5));
    }

    // ----- rotor -----

    private static PartDesign rotor(String name, RotorPart.Wearing wearing) {
        Airfoil sc = Airfoil.naca4(0.009, 0.3, 0.095);
        HelicopterRotor.Geometry g = new HelicopterRotor.Geometry(4, 8.18, 0.2, 0.527, 0.527, -16, sc, sc, 0.0465, 116, 0, false);
        return new TestParts.Fixture(name, "damage rotor " + wearing, d -> {
            RotorPart r = new RotorPart(d, g, 0, 16, 8, 50, MassProperties.point(200, Vec3d.ZERO), List.of());
            return wearing == null ? r : r.wearing(wearing);
        });
    }

    private static Vehicle helicopter(String name, RotorPart.Wearing wearing) {
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("rotor", rotor("r", wearing), Mount.ORIGIN, List.of(Binding.of("collective", "collective"), Binding.of("cyclicPitch", "pitch"))),
                new VehicleBlueprint.Entry("mass", TestParts.box("m", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(5000)), Mount.at(new Vec3d(0, -2, 0)))),
                List.of()).compile();
    }

    @Test void damagedRotorBladesCarryLessForTheSameTorque() {
        Vehicle plain = helicopter("plain helicopter", null), wearing = helicopter("helicopter", RotorPart.Wearing.typical());
        assertEquals(List.of(RotorPart.BLADES), wearing.installed("rotor").part().conditions());
        assertEquals(plain.installed("rotor").part().shownValues(), wearing.installed("rotor").part().shownValues());
        VehicleState a = plain.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT), b = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        alike(plain, wearing, a, b, SKY, GRAVITY, 300, i -> Map.of("collective", 0.6, "pitch", 0.3 * Math.sin(0.02 * i)), s -> spin(s, "rotor", 27));
        // held at its place and speed for a second: the inflow settles the same whatever the blades give
        double[] lift = new double[3], torque = new double[3], used = {0, 0.5, 1};
        Vec3d[] moment = new Vec3d[3];
        for (int k = 0; k < 3; k++) {
            VehicleState st = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
            st.setUsed("rotor", 0, used[k]);
            VehicleDynamics.Result r = null;
            for (int i = 0; i < 200; i++) {
                st.setBody(new RigidBodyState(new Vec3d(0, 100, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                spin(st, "rotor", 27);
                r = VehicleDynamics.step(wearing, st, SKY, Map.of("collective", 0.6, "pitch", 0.4), Vec3d.ZERO, List.of(), null, i * DT, DT);
            }
            finite(st);
            lift[k] = r.diagnostics().get("rotor").get("thrustN"); torque[k] = r.diagnostics().get("rotor").get("torqueNm");
            // the hub's own force and moment: the THRUST loads less the shaft's reaction (a couple about the shaft)
            Wrench w = r.load(AppliedForce.Kind.THRUST);
            moment[k] = new Vec3d(w.torque().x(), 0, w.torque().z());
            assertEquals(lift[k], w.force().y(), 0.05 * lift[0] + 1, "the thrust is the hub's lift, nearly");
        }
        assertTrue(lift[0] > 30000 && torque[0] > 5000, "a Black Hawk's rotor at 9.6 degrees: " + lift[0] + " N for " + torque[0] + " N m");
        assertEquals(0.7, lift[1] / lift[0], 1e-9, "half damaged");
        assertEquals(0.1, lift[2] / lift[0], 1e-9, "stubs");
        assertEquals(torque[0], torque[1], 0, "the shaft's load does not fall");
        assertEquals(torque[0], torque[2], 0);
        assertTrue(moment[0].length() > 100, "the cyclic tilts the disk: " + moment[0]);
        assertEquals(0.7, moment[1].length() / moment[0].length(), 1e-6, "and the hub's moment goes with the blades");
    }

    // ----- marine propeller -----

    private static PartDesign screw(String name, PropellerPart.Wearing wearing) {
        double chordRatio = PropellerModels.chordRatio(4, 0.70, 0.18);
        MassProperties m = new MassProperties(8, Vec3d.ZERO, 0.02, 0.012, 0.012, 0, 0, 0);
        return new TestParts.Fixture(name, "damage screw " + wearing, d -> {
            PropellerPart p = new PropellerPart(d, 0.35, 4, 1.4, 0.18, chordRatio, () -> PropellerModels.fourQuadrant(4, 0.70, 1.4, 0.18), m, List.of());
            return wearing == null ? p : p.wearing(wearing);
        });
    }

    /** VehicleTest's bollard pull with a rudder in the screw's race: a motor, a B4-70 screw, a body. */
    private static Vehicle boat(String name, PropellerPart.Wearing blades, ControlSurfacePart.Wearing surface, double bodyKg) {
        return new VehicleBlueprint(name, List.of(
                new VehicleBlueprint.Entry("motor", TestParts.motor("m", 200, 20000, 3000, 0.02, 0.9, 30), Mount.ORIGIN, List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("screw", screw("s", blades), Mount.at(new Vec3d(-1, -1, 0))),
                new VehicleBlueprint.Entry("rudder", rudder("r", surface), Mount.at(new Vec3d(-1.5, -1, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("body", TestParts.box("b", new Vec3d(4, 1, 1.5), 1.0, true, 4, MassSpec.given(bodyKg)), Mount.ORIGIN)),
                List.of(new Connection.Shaft("motor", "screw", 2, 0.95), new Connection.Slipstream("screw", "rudder"))).compile();
    }

    @Test void aDamagedScrewPushesLessForTheSameTorque() {
        Vehicle plain = boat("plain boat", null, null, 2000), wearing = boat("boat", PropellerPart.Wearing.typical(), null, 2000);
        assertEquals(List.of(PropellerPart.BLADES), wearing.installed("screw").part().conditions());
        VehicleState a = plain.newState(new Vec3d(0, 0.3, 0), Orientation.IDENTITY, DT), b = wearing.newState(new Vec3d(0, 0.3, 0), Orientation.IDENTITY, DT);
        alike(plain, wearing, a, b, SEA, GRAVITY, 300, i -> Map.of("throttle", i < 200 ? 0.8 : -0.5, "rudder", Math.sin(0.03 * i)), s -> {});
        assertTrue(a.body().position().x() > 0.05, "it got under way: " + a.body().position());
        // held still (a bollard pull): the same torque turns the screw as fast whatever its blades give
        double[] thrust = new double[3], torque = new double[3], rudder = new double[3], used = {0, 0.5, 1};
        for (int k = 0; k < 3; k++) {
            VehicleState st = wearing.newState(new Vec3d(0, 0.3, 0), Orientation.IDENTITY, DT);
            st.setUsed("screw", 0, used[k]);
            VehicleDynamics.Result r = null;
            for (int i = 0; i < 1500; i++) {
                st.setBody(new RigidBodyState(new Vec3d(0, 0.3, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO));
                r = VehicleDynamics.step(wearing, st, SEA, Map.of("throttle", 0.6, "rudder", 0.5), GRAVITY, List.of(), null, i * DT, DT);
            }
            finite(st);
            thrust[k] = r.diagnostics().get("screw").get("thrustN"); torque[k] = r.diagnostics().get("screw").get("torqueNm");
            rudder[k] = r.diagnostics().get("rudder").get("forceN");
        }
        assertTrue(thrust[0] > 500, "a bollard pull of " + thrust[0] + " N");
        assertEquals(0.7, thrust[1] / thrust[0], 1e-9, "half damaged");
        assertEquals(0.1, thrust[2] / thrust[0], 1e-9, "stubs");
        assertEquals(torque[0], torque[1], 1e-12 * torque[0], "the shaft's load does not fall");
        assertEquals(torque[0], torque[2], 1e-12 * torque[0]);
        assertTrue(rudder[0] > rudder[1] && rudder[1] > rudder[2] && rudder[2] >= 0, "a weaker race over the rudder: " + rudder[0] + ", " + rudder[1] + ", " + rudder[2]);
    }

    // ----- body: dents and cargo holds -----

    private static final SectionShape CELL = new SectionShape.Cabin(1.8, 1.3, 0.15, 0.6, 0.72, 0.15);
    private static final BodyShape SHAPE = new BodyShape(CELL.scaled(0.6), List.of(BodyModule.straight("cell", 4.5, CELL)));

    /** RoadAeroBodyTest's body (1200 kg), plain (the constructor before 0.42) or with holds and a condition. */
    private static PartDesign body(String name, BodyPart.Wearing wearing, List<Part.Hold> holds) {
        MassProperties m = MassShapes.box(new Vec3d(4.5, 1.3, 1.8), 1).scaled(1200 / MassShapes.box(new Vec3d(4.5, 1.3, 1.8), 1).massKg()).at(new Vec3d(2.25, 0.7, 0));
        return new TestParts.Fixture(name, "damage body " + wearing + " " + holds, d -> {
            if (wearing == null && holds == null) { return new BodyPart(d, SHAPE, 0, BodyPart.Hydro.AUTO, 8, m, Map.of(), List.of(), List.of(), List.of(), null, null); }
            BodyPart b = new BodyPart(d, SHAPE, 0, BodyPart.Hydro.AUTO, 8, m, Map.of(), List.of(), List.of(), List.of(), null, null, holds == null ? List.of() : holds);
            return wearing == null ? b : b.wearing(wearing);
        });
    }

    private static Vehicle shell(String name, BodyPart.Wearing wearing, List<Part.Hold> holds) {
        return new VehicleBlueprint(name, List.of(new VehicleBlueprint.Entry("body", body("b", wearing, holds), Mount.ORIGIN)), List.of()).compile();
    }

    @Test void aDentedBodyDragsMoreAndItsHoldsCarryCargo() {
        List<Part.Hold> holds = List.of(new Part.Hold("boot", 0.4, new Vec3d(0.6, 0.6, 0)), new Part.Hold("bed", 2.0, new Vec3d(2.25, 0.7, 0)));
        Vehicle plain = shell("plain shell", null, null), wearing = shell("shell", BodyPart.Wearing.typical(), holds);
        assertEquals(List.of(BodyPart.STRUCTURE), wearing.installed("body").part().conditions());
        assertEquals(List.of(), plain.holds());
        assertEquals(List.of("body.boot", "body.bed"), wearing.holds().stream().map(Vehicle.Hold::id).toList());
        // through the air and into the water, yawed and turning: empty holds and an undamaged body change nothing
        VehicleState a = plain.newState(new Vec3d(0, 1.2, 0), Orientation.rotationVector(Vec3d.Y.scale(0.1)), DT);
        VehicleState b = wearing.newState(new Vec3d(0, 1.2, 0), Orientation.rotationVector(Vec3d.Y.scale(0.1)), DT);
        for (VehicleState s : new VehicleState[]{a, b}) { moving(s, new Vec3d(10, 0, 0)); }
        alike(plain, wearing, a, b, SEA, GRAVITY, 300, i -> Map.of(), s -> {});
        assertTrue(a.body().position().y() < 0.8, "it came down onto the water: " + a.body().position());
        // one step at 30 m/s in the air, nose on: drag x (1 + 0.3 u), the extra against the air through the centroid
        double[] drag = new double[3], forceX = new double[3], used = {0, 0.5, 1};
        Vec3d[] torque = new Vec3d[3];
        for (int k = 0; k < 3; k++) {
            VehicleState st = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
            moving(st, new Vec3d(30, 0, 0));
            st.setUsed("body", 0, used[k]);
            VehicleDynamics.Result r = VehicleDynamics.step(wearing, st, SKY, Map.of(), Vec3d.ZERO, List.of(), null, 0, DT);
            drag[k] = r.diagnostics().get("body").get("airDragN");
            forceX[k] = r.load(AppliedForce.Kind.DRAG).force().x();
            torque[k] = r.load(AppliedForce.Kind.DRAG).torque();
            assertEquals(used[k], r.diagnostics().get("body").get("structureUsed"), 0);
        }
        assertTrue(drag[0] > 100, "a van-like cell at 30 m/s: " + drag[0] + " N");
        assertEquals(1.15, drag[1] / drag[0], 1e-12, "half damaged");
        assertEquals(1.30, drag[2] / drag[0], 1e-12, "a wreck");
        assertEquals(-0.3 * drag[0], forceX[2] - forceX[0], 1e-9 * drag[0], "the extra drag is on the body");
        // the body's centre of mass is at its centroid's height here (0.7 m against the cell's own), so the extra drag turns it little
        assertTrue(torque[2].subtract(torque[0]).length() < 0.2 * 0.3 * drag[0], "through the centroid: " + torque[2].subtract(torque[0]));
        // cargo rides as a point mass with its weight (ConditionTest has the rest)
        VehicleState st = wearing.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, DT);
        st.setHold(1, 300);
        VehicleDynamics.Result r = VehicleDynamics.step(wearing, st, SKY, Map.of(), GRAVITY, List.of(), null, 0, DT);
        assertEquals(-(1200 + 300) * G, r.load(AppliedForce.Kind.WEIGHT).force().y(), 1e-9 * 1500 * G);
        assertEquals(-G * DT, st.body().velocity().y(), 0.01 * G * DT, "and falls with it (less the air's buoyancy on the shell, half a percent)");
        assertThrows(IllegalArgumentException.class, () -> new BodyPart.Wearing(-0.1));
    }

    // ----- frame -----

    private static final WheelCorner.Wearing AXLES = WheelCorner.Wearing.typical(3e-3, 2e-4, 8000);

    /** FrameFlexTest's car: a 1200 kg frame twisting between two axles; frame and axles with conditions or not. */
    private static Vehicle frameCar(String name, FrameTwist frame, FramePart.Wearing wearing, WheelCorner.Wearing axles) {
        double b = FrameFlexTest.BODY;
        MassProperties m = new MassProperties(b, Vec3d.ZERO, b * (1 + 1.7 * 1.7) / 12, b * (4.2 * 4.2 + 1.7 * 1.7) / 12, b * (4.2 * 4.2 + 1) / 12, 0, 0, 0);
        String key = (frame == null ? "rigid" : java.util.Arrays.toString(frame.stationX()) + java.util.Arrays.deepToString(frame.twistNmPerRad())) + " " + wearing;
        PartDesign body = new TestParts.Fixture("body", "damage frame " + key, d -> {
            FramePart f = new FramePart(d, m, Map.of(), List.of(), frame, List.of());
            return wearing == null ? f : f.wearing(wearing);
        });
        List<VehicleBlueprint.Entry> parts = new ArrayList<>(List.of(new VehicleBlueprint.Entry("body", body, Mount.at(new Vec3d(0, 0.6, 0)))));
        for (boolean front : new boolean[]{true, false}) {
            double load = FrameFlexTest.corner(front), c = 2 * 0.3 * Math.sqrt(FrameFlexTest.RATE * load / G);
            parts.add(new VehicleBlueprint.Entry(front ? "front" : "rear", TestParts.axle(front ? "f" : "r", FrameFlexTest.TRACK, FrameFlexTest.RATE, load, c, c,
                    Suspension.Type.TRAILING_ARM, true, null, null, 0, 0, null, axles),
                    Mount.at(new Vec3d(front ? FrameFlexTest.FRONT : FrameFlexTest.REAR, FrameFlexTest.TYRE.radiusM() - FrameFlexTest.TYRE.deflectionM(load), 0)),
                    List.of(Binding.of("brake", "brake"))));
        }
        return new VehicleBlueprint(name, parts, List.of()).compile();
    }

    /** The corner loads {front left, front right, rear left, rear right} (N) and the frame's twist (deg) at rest on a ground, with the frame so damaged. */
    private static double[] standing(Vehicle v, Ground ground, double used) {
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        if (used > 0) { st.setUsed("body", 0, used); }
        TestTrack.drive(v, st, ground, AIR, (t, s) -> Map.of("brake", 1.0), 3, s -> false, Integer.MAX_VALUE);
        st.setBody(new RigidBodyState(st.body().position(), Vec3d.ZERO, st.body().orientation(), Vec3d.ZERO));
        List<TestTrack.Sample> s = TestTrack.drive(v, st, ground, AIR, (t, x) -> Map.of("brake", 1.0), 0.5, x -> false, Integer.MAX_VALUE);
        Map<String, Map<String, Double>> d = s.get(s.size() - 1).diagnostics();
        finite(st);
        return new double[]{d.get("front").get("leftLoadN"), d.get("front").get("rightLoadN"), d.get("rear").get("leftLoadN"), d.get("rear").get("rightLoadN"),
                d.get("body").get("twistDeg")};
    }

    @Test void aDamagedFrameTwistsMoreEasily() {
        FrameTwist frame = FrameTwist.between(FrameFlexTest.REAR, FrameFlexTest.FRONT, 3e5);
        Vehicle plain = frameCar("plain frame car", frame, null, null), wearing = frameCar("frame car", frame, FramePart.Wearing.typical(), AXLES);
        assertEquals(List.of(FramePart.STRUCTURE), wearing.installed("body").part().conditions());
        assertEquals(7, wearing.conditions());
        // onto a block under one wheel, rolling and then braking: frame and axles with conditions, all new
        Ground ground = FrameFlexTest.block(0.05);
        VehicleState a = plain.newState(Vec3d.ZERO, Orientation.IDENTITY, DT), b = wearing.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        for (VehicleState s : new VehicleState[]{a, b}) { moving(s, new Vec3d(3, 0, 0)); }
        double[] twist = {0};
        int at = plain.installed("body").firstInternal();
        alike(plain, wearing, a, b, VehicleEnvironment.land(ground, AIR), GRAVITY, 600, i -> Map.of("brake", i > 300 ? 0.5 : 0.0),
                s -> twist[0] = Math.max(twist[0], Math.abs(s.internals[at + 1] - s.internals[at])));
        assertTrue(twist[0] > 1e-4, "it twisted on the way: " + twist[0] + " rad");
        // on a block under the front left wheel: a frame used up (0.4 of its stiffness left) stands as one built with 0.4 of it
        Ground block = FrameFlexTest.block(0.02);
        double[] sound = standing(wearing, block, 0), half = standing(wearing, block, 0.5), wrecked = standing(wearing, block, 1);
        double[] soft = standing(frameCar("soft frame car", FrameTwist.between(FrameFlexTest.REAR, FrameFlexTest.FRONT, 0.4 * 3e5), null, null), block, 0);
        double[] softer = standing(frameCar("softer frame car", FrameTwist.between(FrameFlexTest.REAR, FrameFlexTest.FRONT, 0.7 * 3e5), null, null), block, 0);
        for (int i = 0; i < 5; i++) {
            assertEquals(soft[i], wrecked[i], 1e-6 * Math.abs(soft[i]), "used up, value " + i);
            assertEquals(softer[i], half[i], 1e-6 * Math.abs(softer[i]), "half damaged, value " + i);
        }
        assertTrue(wrecked[4] > 1.5 * sound[4] && half[4] > 1.15 * sound[4], "it twists more: " + sound[4] + ", " + half[4] + ", " + wrecked[4] + " degrees");
        double warp = sound[0] - sound[1] + sound[3] - sound[2], warpWrecked = wrecked[0] - wrecked[1] + wrecked[3] - wrecked[2];
        // (the springs are much softer than even the damaged frame, so they still take most of the warp)
        assertTrue(warpWrecked < 0.98 * warp && warpWrecked > 0.5 * warp, "and loads its wheels less unevenly on the block: " + warpWrecked + " against " + warp + " N");
        // the solve itself: a share scales the whole matrix (still symmetric, rows summing to zero); 1 is the solve as it was
        double[] x = {-1.6, 0, 1.1};
        FrameTwist chain = FrameTwist.chain(x, new double[]{2e5, 5e5});
        Vehicle.Flex flex = new Vehicle.Flex(0, x, chain.twistNmPerRad(), new double[]{0.3, 0.4, 0.3});
        double[][] scaled = new double[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { scaled[i][j] = 0.4 * chain.twistNmPerRad()[i][j]; }
        new FrameTwist(x, scaled);   // a valid stiffness: symmetric, zero row sums
        List<FrameFlex.Corner> corners = List.of(new FrameFlex.Corner(1.1, -0.8, 5000, 40000), new FrameFlex.Corner(1.1, 0.8, 3000, 40000),
                new FrameFlex.Corner(-1.6, -0.8, 3500, 35000), new FrameFlex.Corner(-1.6, 0.8, 3600, 35000));
        double[] start = {0.001, 0, -0.002};
        assertArrayEquals(FrameFlex.solve(flex, start, corners), FrameFlex.solve(flex, start, corners, 1), 0);
        assertArrayEquals(FrameFlex.solve(new Vehicle.Flex(0, x, scaled, flex.weights()), start, corners), FrameFlex.solve(flex, start, corners, 0.4), 1e-15);
        // a rigid frame only carries the condition
        Vehicle rigid = frameCar("rigid frame car", null, FramePart.Wearing.typical(), null);
        assertEquals(null, rigid.flex());
        assertEquals(List.of("structure"), rigid.installed("body").part().conditions());
        assertThrows(IllegalArgumentException.class, () -> new FramePart.Wearing(1.1));
    }

    // ----- everything used up -----

    @Test void nothingBreaksWithEverythingUsedUp() {
        // an aeroplane: wing, tailplane, propeller and body all wrecked
        Vehicle plane = new VehicleBlueprint("wreck", List.of(
                new VehicleBlueprint.Entry("body", body("b", BodyPart.Wearing.typical(), List.of(new Part.Hold("hold", 1, new Vec3d(2, 0.5, 0)))), Mount.at(new Vec3d(-2.25, -0.7, 0))),
                new VehicleBlueprint.Entry("wing", wing("w", WingPart.Wearing.typical()), new Mount(new Vec3d(0.3, 0.3, 0), 0, 3, 0), List.of(Binding.of("aileron", "roll"))),
                new VehicleBlueprint.Entry("tail", rudder("t", ControlSurfacePart.Wearing.typical()), new Mount(new Vec3d(-2.2, 0.2, 0), 0, 0, 90), List.of(Binding.of("deflection", "pitch"))),
                new VehicleBlueprint.Entry("engine", TestParts.flywheel("e", 1.0, true), Mount.at(new Vec3d(1.5, 0, 0)), List.of(Binding.of("torque", "torque"))),
                new VehicleBlueprint.Entry("prop", airscrew("p", AirPropellerPart.Wearing.typical()), Mount.at(new Vec3d(2.4, 0, 0)))),
                List.of(new Connection.Shaft("engine", "prop", 1, 1))).compile();
        for (double used : new double[]{0, 1}) {
            VehicleState st = plane.newState(new Vec3d(0, 300, 0), Orientation.IDENTITY, DT);
            double[] all = new double[plane.conditions()];
            java.util.Arrays.fill(all, used);
            st.setUsedValues(all);
            st.setHold(0, 150);
            moving(st, new Vec3d(45, 0, 0));
            spin(st, "prop", 150);
            double height = st.body().position().y();
            for (int i = 0; i < 800; i++) {
                VehicleDynamics.step(plane, st, SKY, Map.of("torque", 1200.0, "roll", 0.5 * Math.sin(0.01 * i), "pitch", -0.3), GRAVITY, List.of(), null, i * DT, DT);
                finite(st);
            }
            assertEquals(used, st.worstUsed(), 0, "damage does not come or go by itself");
            if (used == 1) { assertTrue(st.body().position().y() < height - 20, "a wreck comes down: " + st.body().position()); }
        }
        // a helicopter with its blades gone
        Vehicle heli = helicopter("wrecked helicopter", RotorPart.Wearing.typical());
        VehicleState hs = heli.newState(new Vec3d(0, 300, 0), Orientation.IDENTITY, DT);
        hs.setUsedValues(new double[]{1});
        for (int i = 0; i < 600; i++) {
            spin(hs, "rotor", 27);
            VehicleDynamics.step(heli, hs, SKY, Map.of("collective", 0.9, "pitch", Math.sin(0.02 * i)), GRAVITY, List.of(), null, i * DT, DT);
            finite(hs);
        }
        assertTrue(hs.body().velocity().y() < -10, "falling: " + hs.body().velocity());
        // a boat with screw and rudder gone
        Vehicle boat = boat("wrecked boat", PropellerPart.Wearing.typical(), ControlSurfacePart.Wearing.typical(), 2000);
        VehicleState bs = boat.newState(new Vec3d(0, 0.3, 0), Orientation.IDENTITY, DT);
        bs.setUsedValues(new double[]{1, 1});
        for (int i = 0; i < 800; i++) {
            VehicleDynamics.step(boat, bs, SEA, Map.of("throttle", i < 500 ? 1.0 : -1.0, "rudder", Math.sin(0.02 * i)), GRAVITY, List.of(), null, i * DT, DT);
            finite(bs);
        }
        // a car on a frame bent to scrap, its axles worn out, over a block
        Vehicle car = frameCar("wrecked frame car", FrameTwist.between(FrameFlexTest.REAR, FrameFlexTest.FRONT, 3e5), new FramePart.Wearing(1), AXLES);
        VehicleState cs = car.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        double[] ones = new double[car.conditions()];
        java.util.Arrays.fill(ones, 1);
        cs.setUsedValues(ones);
        moving(cs, new Vec3d(4, 0, 0));
        VehicleEnvironment land = VehicleEnvironment.land(FrameFlexTest.block(0.06), AIR);
        for (int i = 0; i < 800; i++) {
            VehicleDynamics.step(car, cs, land, Map.of("brake", i > 500 ? 1.0 : 0.0), GRAVITY, List.of(), null, i * DT, DT);
            finite(cs);
        }
        assertTrue(cs.body().position().y() > 0.2 && cs.body().position().y() < 1.5, "on its wheels: " + cs.body().position());
    }
}

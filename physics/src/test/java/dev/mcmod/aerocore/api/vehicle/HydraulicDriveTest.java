package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.ContactMaterial;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.hydraulic.Cylinder;
import dev.mcmod.aerocore.api.hydraulic.Motor;
import dev.mcmod.aerocore.api.hydraulic.Pump;
import dev.mcmod.aerocore.api.vehicle.part.HydraulicMotorPart;
import dev.mcmod.aerocore.api.vehicle.part.HydraulicPumpPart;
import dev.mcmod.aerocore.api.vehicle.part.LinkPart;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** 0.43: hydraulic pumps, rams, slews and motors on a vehicle's circuits (HydraulicPumpPart, LinkPart, HydraulicMotorPart). */
class HydraulicDriveTest {
    private static final double DT = 0.005, G = Atmosphere.GRAVITY_MPS2;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final VehicleEnvironment ENV = VehicleEnvironment.calm(-1e4, WATER, AIR);
    private static final double BAR = 1e5;
    private static final List<CollisionSphere> FEET = List.of(new CollisionSphere(new Vec3d(-2, -0.5, -1.2), 0.3), new CollisionSphere(new Vec3d(-2, -0.5, 1.2), 0.3),
            new CollisionSphere(new Vec3d(2, -0.5, -1.2), 0.3), new CollisionSphere(new Vec3d(2, -0.5, 1.2), 0.3));

    private static FloatingBodyDynamics.Collision floor() {
        WorldBox ground = new WorldBox(new Vec3d(-50, -10, -50), new Vec3d(50, -0.8, 50), ContactMaterial.STEEL_ON_ROCK);
        return new FloatingBodyDynamics.Collision(FEET, List.of(ground), ContactMaterial.STEEL_ON_ROCK);
    }

    private static PartDesign fixture(String key, java.util.function.Function<PartDesign, Part> build) { return new TestParts.Fixture(key, key, build); }

    private static MassProperties box(Vec3d size, double kg, Vec3d centre) {
        MassProperties m = MassShapes.box(size, 1e9);
        return m.scaled(kg / m.massKg()).transformed(centre, Orientation.IDENTITY);
    }

    /** A 100 cc/rev axial piston pump at 300 bar, driven 1 : 1 by a 200 kW electric motor running at up to 2000 rpm. */
    private static final Pump PUMP = Pump.axialPiston(100e-6, Pump.Control.PRESSURE_COMPENSATED, 300 * BAR, 0);
    /** A 100 / 70 mm ram, 1 m stroke. */
    private static final Cylinder RAM = Cylinder.of(0.1, 0.07, 1.0);

    /**
     * A heavy base with a pump, and one or two 3 m arms of armKg hinged on its top, each lifted by a ram from 1 m below
     * its pin to 1 m out along it (lever about 0.71 m/rad at q = 0), with a valve of valveLps L/s.
     */
    private static Vehicle machine(int arms, double armKg, double valveLps) {
        List<VehicleBlueprint.Entry> parts = new ArrayList<>();
        Map<String, Mount> pins = new java.util.HashMap<>();
        for (int k = 0; k < arms; k++) { pins.put("pin" + k, Mount.at(new Vec3d(1.5, 0.5, -0.8 + 1.6 * k))); }
        parts.add(new VehicleBlueprint.Entry("base", TestParts.solid("base", new Vec3d(5, 1, 3), 20000, Vec3d.ZERO, pins, null, 0, FEET), Mount.at(Vec3d.ZERO)));
        parts.add(new VehicleBlueprint.Entry("engine", TestParts.motor("engine", 1500, 200e3, 2000, 0.8, 0.95, 300), Mount.at(new Vec3d(-1.5, 0.8, 0)),
                List.of(Binding.of("throttle", "throttle"))));
        parts.add(new VehicleBlueprint.Entry("pump", fixture("pump", d -> new HydraulicPumpPart(d, PUMP, "main", Vec3d.X, 0.02, MassProperties.point(50, Vec3d.ZERO), List.of())),
                Mount.at(new Vec3d(-1, 0.8, 0))));
        for (int k = 0; k < arms; k++) {
            LinkPart.Ram ram = new LinkPart.Ram(RAM, new Vec3d(0, -1, 0), new Vec3d(1, 0, 0), "main", 300 * BAR, 350 * BAR, valveLps * 1e-3);
            Part.Joint hinge = Part.Joint.revolute(Vec3d.ZERO, Vec3d.Z, -0.6, 1.0, 0);
            MassProperties m = box(new Vec3d(3, 0.3, 0.3), armKg, new Vec3d(1.5, 0, 0));
            parts.add(new VehicleBlueprint.Entry("arm" + k, fixture("arm" + armKg + valveLps,
                    d -> new LinkPart(d, m, List.of(), Map.of(), hinge, ram, List.of(), List.of(), List.of(), List.of())),
                    Mount.on("base", "pin" + k), List.of(Binding.of("command", "arm" + k))));
        }
        return new VehicleBlueprint("machine", parts, List.of(Connection.Shaft.ports("engine", "", "pump", "input", 1, 0.98))).compile();
    }

    private static VehicleState run(Vehicle v, Map<String, Double> controls, double seconds, VehicleState st) {
        if (st == null) { st = v.newState(new Vec3d(0, 0.02, 0), Orientation.IDENTITY, DT); }
        int steps = (int) Math.round(seconds / DT);
        for (int i = 0; i < steps; i++) { VehicleDynamics.step(v, st, ENV, controls, Vec3d.Y.scale(-G), List.of(), floor(), i * DT, DT); }
        return st;
    }

    @Test
    void aRamLiftsAtItsValvesFlowOverItsArea() {
        Vehicle v = machine(1, 300, 2.0);
        VehicleState st = run(v, Map.of("throttle", 1.0), 2, null);   // the engine and pump up to speed, the arm held
        assertEquals(0, st.jointValue(0), 1e-3, "centred, the ram holds the arm");
        st = run(v, Map.of("throttle", 1.0, "arm0", 1.0), 0.5, st);
        double rate = st.jointRate(0);
        LinkPart arm = (LinkPart) v.installed("arm0").part();
        double lever = arm.lever((LinkPart.Ram) arm.actuator(), st.jointValue(0));
        double expected = RAM.extendingVelocityMps(2e-3) / lever;
        assertEquals(expected, rate, 0.03 * expected, "rate = valve flow / head area / lever");
        double pressure = st.hydraulicPressure("main");
        double needed = 300 * G * 1.5 * Math.cos(st.jointValue(0)) / lever / RAM.headAreaM2();
        assertEquals(needed, pressure, 0.15 * needed, "the circuit runs at the pressure the arm's weight takes");
    }

    @Test
    void aRamStallsAgainstAWeightBeyondItsRelief() {
        // 300 bar on a 100 mm bore pushes 224 kN; at a 0.71 m lever that holds 160 kN m, a 12 t arm at 1.5 m weighs 177 kN m
        Vehicle v = machine(1, 12000, 2.0);
        VehicleState st = run(v, Map.of("throttle", 1.0), 1, null);
        double before = st.jointValue(0);
        st = run(v, Map.of("throttle", 1.0, "arm0", 1.0), 1, st);
        assertTrue(st.jointValue(0) < before - 0.05, "it gives way under the load: " + st.jointValue(0));
        // the circuit runs at what the sinking load takes, near the relief (the lever grows as the arm sinks)
        double p = st.hydraulicPressure("main");
        assertTrue(p > 0.85 * 300 * BAR && p <= 300 * BAR, "near the relief: " + p / BAR + " bar");
    }

    @Test
    void twoRamsShareAPumpThatCannotFeedThemBoth() {
        // the pump gives about 3.3 L/s at 2000 rpm; two valves ask 3 L/s each
        Vehicle v = machine(2, 300, 3.0);
        VehicleState st = run(v, Map.of("throttle", 1.0), 2, null);
        st = run(v, Map.of("throttle", 1.0, "arm0", 1.0, "arm1", 1.0), 0.3, st);
        double share = st.hydraulicShare("main");
        assertTrue(share < 0.65 && share > 0.45, "about 3.3 of 6 L/s: " + share);
        assertEquals(st.jointRate(0), st.jointRate(1), 0.02 * st.jointRate(0), "both slow down alike");
        double[] flows = st.hydraulicFlows("main");
        assertEquals(6e-3, flows[1], 1e-9, "they asked for their valves' flows");
    }

    @Test
    void thePumpLoadsTheEngineWithTheTorqueThePressureTakes() {
        Vehicle v = machine(1, 12000, 2.0);
        VehicleState st = run(v, Map.of("throttle", 1.0, "arm0", 1.0), 3, null);
        Map<String, Double> pump = st.diagnostics().get("pump");
        double p = pump.get("pressurePa"), torque = Math.abs(pump.get("torqueNm"));
        // against the relief a compensated pump strokes back to its leakage; what remains is its drag
        assertTrue(torque < PUMP.displacementM3PerRev() * p / (2 * Math.PI), "a compensated pump at its set pressure takes little: " + torque);
        VehicleState free = run(machine(1, 300, 2.0), Map.of("throttle", 1.0, "arm0", 1.0), 1, null);
        Map<String, Double> lifting = free.diagnostics().get("pump");
        double flowing = Math.abs(lifting.get("torqueNm")), expected = PUMP.torqueNm(Math.abs(free.rotorSpeed(v.installed("pump").firstRotor())), lifting.get("pressurePa"));
        assertEquals(expected, flowing, 0.05 * expected + 1, "lifting, the shaft carries D p / (2 pi eta) and the drag");
    }

    @Test
    void aWinchPaysOutAndHaulsInItsRope() {
        Motor motor = Motor.axialPiston(50e-6);
        HydraulicMotorPart.Drum drum = new HydraulicMotorPart.Drum(0.3, 2, 10, 2, 40);
        List<VehicleBlueprint.Entry> parts = new ArrayList<>();
        parts.add(new VehicleBlueprint.Entry("base", TestParts.solid("base", new Vec3d(5, 1, 3), 20000, Vec3d.ZERO, Map.of(), null, 0, FEET), Mount.at(Vec3d.ZERO)));
        parts.add(new VehicleBlueprint.Entry("engine", TestParts.motor("engine", 1500, 200e3, 2000, 0.8, 0.95, 300), Mount.at(new Vec3d(-1.5, 0.8, 0)),
                List.of(Binding.of("throttle", "throttle"))));
        parts.add(new VehicleBlueprint.Entry("pump", fixture("pump", d -> new HydraulicPumpPart(d, PUMP, "main", Vec3d.X, 0.02, MassProperties.point(50, Vec3d.ZERO), List.of())),
                Mount.at(new Vec3d(-1, 0.8, 0))));
        parts.add(new VehicleBlueprint.Entry("winch", fixture("winch", d -> new HydraulicMotorPart(d, motor, "main", 300 * BAR, 20, 2000, 0.5, Vec3d.Z, drum,
                MassProperties.point(200, Vec3d.ZERO), List.of())), Mount.at(new Vec3d(0, 0.8, 0)), List.of(Binding.of("command", "winch"))));
        Vehicle v = new VehicleBlueprint("winch", parts, List.of(Connection.Shaft.ports("engine", "", "pump", "input", 1, 0.98))).compile();
        VehicleState st = run(v, Map.of("throttle", 1.0), 1, null);
        assertEquals(10, st.internal("winch", 1), 1e-9, "centred, the drum holds");
        st = run(v, Map.of("throttle", 1.0, "winch", 1.0), 2, st);
        double paid = st.internal("winch", 1) - 10;
        // 20 rad/s x 0.3 m / 2 falls = 3 m/s of hook, a little less while it spins up
        assertTrue(paid > 5 && paid < 6.01, "it pays out about 3 m/s: " + paid);
        st = run(v, Map.of("throttle", 1.0, "winch", -1.0), 10, st);
        assertEquals(2, st.internal("winch", 1), 1e-9, "hauling in stops at the rope's end");
    }
}

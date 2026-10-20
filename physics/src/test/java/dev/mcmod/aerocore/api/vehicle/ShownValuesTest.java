package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.Airfoil;
import dev.mcmod.aerocore.api.aero.HelicopterRotor;
import dev.mcmod.aerocore.api.ground.Ground;
import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.vehicle.part.LegPart;
import dev.mcmod.aerocore.api.vehicle.part.RotorPart;
import dev.mcmod.aerocore.api.wheel.Suspension;
import dev.mcmod.aerocore.api.wheel.Tyre;
import dev.mcmod.aerocore.api.wheel.WheelCorner;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/** 0.30: wheels' slip and load, a retractable leg, and a rotor's flapping and pitch as shown values. */
class ShownValuesTest {
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final double G = TestTrack.G;
    private static final Ground FLAT = Ground.flat(0, GroundContact.Surface.TARMAC);

    /** A main leg under 5 kN (WheelTest's oleo leg), retractable or not, and a mass on it. */
    private static Vehicle leg(LegPart.Retraction retraction) { return leg(retraction, null); }
    /** 0.36: or telescoping. */
    private static Vehicle leg(LegPart.Retraction retraction, LegPart.Telescope telescope) {
        double load = 5000, stroke = 0.3, area = 0.0042, pStatic = load / area + 101325, staticStroke = 0.84 * stroke;
        double extended = pStatic / 4, volume = area * staticStroke / (1 - Math.pow(0.25, 1 / Suspension.POLYTROPIC));
        Suspension strut = new Suspension(new Suspension.Geometry(0, 0, false), Suspension.Spring.oleo(area, volume, extended),
                Suspension.Damper.orifice(area, 1.45e-4, 850, 3, 1800), stroke, 0, staticStroke);
        Tyre tyre = new Tyre(new Tyre.Spec(Tyre.Construction.AIRCRAFT, 0.16, 0.91, 0.1524, 290e3, 290e3, Tyre.Tread.AIRCRAFT_RIB, 1));
        WheelCorner wheel = new WheelCorner(new WheelCorner.Spec(new Vec3d(0, 0, 0), 0, 0, tyre, 0.05, strut, null, 0));
        PartDesign leg = new TestParts.Fixture("leg", telescope == null ? "leg " + retraction : "leg " + telescope, d -> new LegPart(d, wheel, LegPart.Steer.FIXED, 0, 0.1, null,
                MassProperties.point(20, Vec3d.ZERO), List.of(), retraction, telescope));
        return new VehicleBlueprint("gear", List.of(
                new VehicleBlueprint.Entry("leg", leg, Mount.ORIGIN, retraction == null && telescope == null ? List.of() : List.of(Binding.of("retract", "landing_gear"))),
                new VehicleBlueprint.Entry("mass", TestParts.box("m", new Vec3d(0.5, 0.5, 0.5), 0, false, 1, MassSpec.given(load / G - 20)), Mount.at(new Vec3d(0, 0.5, 0)))),
                List.of()).compile();
    }

    private static final LegPart.Retraction FOLDS = new LegPart.Retraction(2.0, Vec3d.X, Math.toRadians(90), 0.25, 0.8);

    @Test void aFixedLegKeepsItsInputsAndStates() {
        Vehicle fixed = leg(null);
        LegPart part = (LegPart) fixed.installed("leg").part();
        assertEquals(List.of("brake"), part.inputs());
        assertEquals(WheelCorner.STATES + 1, part.internalStates(), "as in 0.29");
        Vehicle folding = leg(FOLDS);
        assertEquals(List.of("brake", "retract"), folding.installed("leg").part().inputs());
        assertEquals(WheelCorner.STATES + 2, folding.installed("leg").part().internalStates());
    }

    @Test void parkedTheWheelShowsNoSlipAndItsLoadStaysPut() {
        Vehicle v = leg(null);
        VehicleState st = TestTrack.settle(v, FLAT, AIR, Vec3d.ZERO, 0, Map.of(), 2);
        VehicleEnvironment env = VehicleEnvironment.land(FLAT, AIR);
        // the oleo strut creeps for a while yet
        for (int i = 0; i < 1200; i++) { VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G), List.of(), null, i * TestTrack.DT, TestTrack.DT); }
        double slip = st.shown("leg", WheelCorner.SHOWN), load = st.shown("leg", WheelCorner.SHOWN + 1);
        assertEquals(0, slip, 1e-12);
        assertEquals(5.0, load, 0.3, "about the 5 kN it carries");
        double[] all = new double[v.installed("leg").part().shownValues()];
        for (int k = 0; k < all.length; k++) { all[k] = st.shown("leg", k); }
        for (int i = 0; i < 200; i++) {
            VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G), List.of(), null, i * TestTrack.DT, TestTrack.DT);
            assertEquals(slip, st.shown("leg", WheelCorner.SHOWN), "slip unchanged at rest");
            assertEquals(load, st.shown("leg", WheelCorner.SHOWN + 1), "load unchanged at rest (no updates to send)");
            // every shown value: the spin's angle, the steering and the compression too (the strut still creeps by microns)
            for (int k = 0; k < all.length; k++) { assertEquals(all[k], st.shown("leg", k), "shown value " + k + " unchanged at rest"); }
        }
        // pushed along with the wheel standing still, it slides
        RigidBodyState b = st.body();
        st.setBody(new RigidBodyState(b.position(), new Vec3d(10, 0, 0), b.orientation(), Vec3d.ZERO));
        VehicleDynamics.step(v, st, env, Map.of(), Vec3d.Y.scale(-G), List.of(), null, 0, TestTrack.DT);
        assertTrue(st.shown("leg", WheelCorner.SHOWN) > 5, "sliding: " + st.shown("leg", WheelCorner.SHOWN));
    }

    @Test void aRetractableLegStaysDownWithWeightOnItAndFoldsInTheAir() {
        Vehicle v = leg(FOLDS);
        int fraction = WheelCorner.SHOWN + WheelCorner.EXTRA;
        VehicleState st = TestTrack.settle(v, FLAT, AIR, Vec3d.ZERO, 0, Map.of(), 2);
        VehicleEnvironment env = VehicleEnvironment.land(FLAT, AIR);
        for (int i = 0; i < 400; i++) {
            VehicleDynamics.step(v, st, env, Map.of("landing_gear", 1.0), Vec3d.Y.scale(-G), List.of(), null, i * TestTrack.DT, TestTrack.DT);
        }
        assertEquals(0, st.shown("leg", fraction), "on the ground the gear stays down");
        assertTrue(st.shown("leg", WheelCorner.SHOWN + 1) > 4, "and carries the weight");
        // in the air at 60 m/s, held up (a wing's lift): it folds in its 2 s, and its drag goes with it
        VehicleState air = v.newState(new Vec3d(0, 40, 0), Orientation.IDENTITY, TestTrack.DT);
        air.internals[v.installed("leg").firstInternal()] = 0;   // the strut hangs extended (no weight on the wheel)
        air.setBody(new RigidBodyState(air.body().position(), new Vec3d(60, 0, 0), Orientation.IDENTITY, Vec3d.ZERO));
        double massKg = v.properties().massKg();
        var lift = List.of(new dev.mcmod.aerocore.api.AppliedForce(dev.mcmod.aerocore.api.AppliedForce.Kind.LIFT, Vec3d.ZERO, Vec3d.Y.scale(massKg * G)));
        double firstDrag = Double.NaN, lastDrag = Double.NaN;
        int steps = (int) Math.round(2.1 / TestTrack.DT);
        for (int i = 0; i < steps; i++) {
            VehicleDynamics.Result r = VehicleDynamics.step(v, air, env, Map.of("landing_gear", 1.0), Vec3d.Y.scale(-G), lift, null, i * TestTrack.DT, TestTrack.DT);
            double drag = air.diagnostics().getOrDefault("leg", Map.of()).getOrDefault("gearDragN", 0.0);
            if (i == 0) { firstDrag = drag; }
            lastDrag = drag;
        }
        double expected = 0.5 * AIR.densityKgPerM3() * 60 * 60 * 0.8 * 0.25;
        assertEquals(expected, firstDrag, 0.05 * expected, "down, it drags as a bluff body");
        assertEquals(1.0, air.shown("leg", fraction), 1e-9, "folded in its time");
        assertEquals(0, lastDrag, 1e-9, "folded, no drag");
        assertEquals(0, air.shown("leg", WheelCorner.SHOWN + 1), "and no load");
    }

    @Test void aTelescopingLegDrawsItsLoadDownAndLetsItUpAgain() {
        // 0.36: the leg's strut draws up 0.3 m in 2 s with the weight on it (a trailer's landing leg), and back
        Vehicle v = leg(null, new LegPart.Telescope(0.3, 2));
        LegPart part = (LegPart) v.installed("leg").part();
        assertEquals(List.of("brake", "retract"), part.inputs());
        assertEquals(WheelCorner.STATES + 2, part.internalStates());
        int drawn = WheelCorner.SHOWN + WheelCorner.EXTRA;
        VehicleState st = TestTrack.settle(v, FLAT, AIR, Vec3d.ZERO, 0, Map.of(), 2);
        VehicleEnvironment env = VehicleEnvironment.land(FLAT, AIR);
        double down = st.body().position().y(), up = Double.NaN, again = Double.NaN;
        for (int i = 0; i < 1800; i++) {
            double retract = i < 900 ? 1 : 0;
            VehicleDynamics.step(v, st, env, Map.of("landing_gear", retract), Vec3d.Y.scale(-G), List.of(), null, i * TestTrack.DT, TestTrack.DT);
            if (i == 899) { up = st.body().position().y(); assertEquals(1.0, st.shown("leg", drawn), 1e-9, "drawn up"); }
        }
        again = st.body().position().y();
        assertEquals(down - 0.3, up, 0.01, "the load comes down by the stroke");
        assertEquals(down, again, 0.01, "and goes back up");
        assertEquals(0, st.shown("leg", drawn), 1e-9, "let down");
        assertTrue(st.shown("leg", WheelCorner.SHOWN + 1) > 4, "carrying the weight all along");
    }

    @Test void aRotorShowsItsFlappingAndPitch() {
        Airfoil sc = Airfoil.naca4(0.009, 0.3, 0.095);
        HelicopterRotor.Geometry g = new HelicopterRotor.Geometry(4, 8.18, 0.2, 0.527, 0.527, -16, sc, sc, 0.0465, 116, 0, false);
        PartDesign rotor = new TestParts.Fixture("rotor", "black hawk", d -> new RotorPart(d, g, 0, 16, 8, 50, MassProperties.point(200, Vec3d.ZERO), List.of()));
        Vehicle v = new VehicleBlueprint("rotor", List.of(
                new VehicleBlueprint.Entry("rotor", rotor, Mount.ORIGIN, List.of(Binding.of("collective", "collective"), Binding.of("cyclicPitch", "pitch"))),
                new VehicleBlueprint.Entry("mass", TestParts.box("m", new Vec3d(1, 1, 1), 0, false, 1, MassSpec.given(5000)), Mount.at(new Vec3d(0, -2, 0)))),
                List.of()).compile();
        VehicleState st = v.newState(new Vec3d(0, 100, 0), Orientation.IDENTITY, TestTrack.DT);
        st.setTrainSpeed(v.rotors().get(v.installed("rotor").firstRotor()).train(), 27);
        VehicleEnvironment env = VehicleEnvironment.land(Ground.flat(-1000, GroundContact.Surface.TARMAC), AIR);
        Map<String, Double> controls = Map.of("collective", 0.6, "pitch", 0.4);
        for (int i = 0; i < 300; i++) {
            st.setTrainSpeed(v.rotors().get(v.installed("rotor").firstRotor()).train(), 27);
            VehicleDynamics.step(v, st, env, controls, Vec3d.ZERO, List.of(), null, i * TestTrack.DT, TestTrack.DT);
        }
        int at = v.installed("rotor").firstInternal();
        assertEquals(RotorPart.SHOWN, v.installed("rotor").part().shownValues());
        for (int i = 0; i < 3; i++) { assertEquals(st.internals[at + 1 + i], st.shown("rotor", i), 1e-3, "flapping " + i); }
        assertEquals(Math.toRadians(0.6 * 16), st.shown("rotor", 3), 1e-3, "collective");
        assertTrue(Math.abs(st.shown("rotor", 4)) + Math.abs(st.shown("rotor", 5)) > 1e-3, "cyclic shown");
        assertEquals(st.internals[at], st.shown("rotor", 6), 0.1, "induced velocity");
        assertTrue(st.shown("rotor", 0) > 0, "coning up under thrust");
    }

    @Test void steppedValuesMoveOnlyPastThreeQuartersOfAStep() {
        assertEquals(1.0, PartStep.stepped(1.0, 1.07, 0.1));
        assertEquals(1.1, PartStep.stepped(1.0, 1.08, 0.1), 1e-12);
        assertEquals(1.0, PartStep.stepped(1.0, 0.93, 0.1));
    }
}

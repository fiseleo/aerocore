package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.RigidBodyState;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class JetEngineTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;
    private static final RigidBodyState HELD = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);

    private static boolean invalid(JetEngineDesign e, String parameter) {
        return e.check().stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID && i.parameter().startsWith(parameter));
    }

    /** The design with one component replaced. */
    private static JetEngineDesign with(JetEngineDesign e, String key, EngineComponent c) {
        Map<String, EngineComponent> m = new java.util.LinkedHashMap<>(e.components());
        m.put(key, c);
        return new JetEngineDesign(e.name(), (JetInletDesign) m.get("inlet"), (CompressorDesign) m.get("compressor"), (CombustorDesign) m.get("combustor"),
                (TurbineDesign) m.get("turbine"), (JetNozzleDesign) m.get("nozzle"), (JetControlDesign) m.get("control"));
    }

    private static Vehicle bench(JetEngineDesign e) {
        Vehicle v = new VehicleBlueprint("bench", List.of(new VehicleBlueprint.Entry("jet", e, Mount.at(Vec3d.ZERO),
                List.of(Binding.of("throttle", "throttle"), Binding.of("ignition", "ignition")))), List.of()).compile();
        v.warmUp();
        return v;
    }

    /** A test stand: the engine held in place, run for some seconds; returns the last diagnostics. */
    private static final class Stand {
        final Vehicle vehicle;
        final VehicleState state;
        VehicleEnvironment environment = VehicleEnvironment.calm(-100, WATER, AIR);
        final Map<String, Double> controls = new HashMap<>(Map.of("throttle", 0.0, "ignition", 1.0));
        double t, maxSurging;
        VehicleDynamics.Result last;

        Stand(JetEngineDesign e) { vehicle = bench(e); state = vehicle.newState(Vec3d.ZERO, Orientation.IDENTITY, DT); }

        Map<String, Double> run(double seconds) {
            for (int i = 0; i < seconds / DT; i++, t += DT) {
                last = VehicleDynamics.step(vehicle, state, environment, controls, Vec3d.ZERO, List.of(), null, t, DT);
                state.setBody(HELD);
                maxSurging = Math.max(maxSurging, d().get("surging"));
            }
            return d();
        }
        Map<String, Double> d() { return last.diagnostics().get("jet"); }
    }

    @Test void theSampleTurbojetsAreValidRoundTripAndWeighLikeTheirKind() {
        for (JetEngineDesign e : List.of(SampleEngines.turbojet(), SampleEngines.modelTurbojet())) {
            assertTrue(e.check().stream().noneMatch(i -> i.kind() == DesignIssue.Kind.INVALID), e.check().toString());
            String json = Json.write(e.toJson());
            PartDesign back = PartTypes.read(Json.parse(json), "jet");
            assertEquals(e, back);
            assertEquals(json, Json.write(back.toJson()));
        }
        // J85-GE-17A: 181 kg, 1.03 m (Smithsonian); JetCat P80: 1.32 kg with its starter
        assertEquals(181, SampleEngines.turbojet().totalMassKg(), 0.25 * 181);
        assertEquals(1.32, SampleEngines.modelTurbojet().totalMassKg(), 0.25 * 1.32);
        assertEquals(1.03, SampleEngines.turbojet().length(), 0.15);
    }

    @Test void enginesThatCannotWorkAreRejected() {
        JetEngineDesign e = SampleEngines.turbojet();
        CompressorDesign c = e.compressor();
        TurbineDesign t = e.turbine();
        JetControlDesign k = e.control();
        // two axial stages cannot make a pressure ratio of 8
        assertTrue(invalid(with(e, "compressor", new CompressorDesign("c", c.kind(), 2, c.pressureRatio(), c.airflowKgPerS(), c.polytropicEfficiency(),
                c.surgeMargin(), c.tipDiameterM(), c.hubTipRatio(), c.designRpm(), c.blades(), c.material(), Double.NaN)), "compressor.stages"));
        // a titanium rotor at twice the speed yields
        assertTrue(invalid(with(e, "compressor", new CompressorDesign("c", c.kind(), c.stages(), c.pressureRatio(), c.airflowKgPerS(), c.polytropicEfficiency(),
                c.surgeMargin(), c.tipDiameterM(), c.hubTipRatio(), 2.4 * c.designRpm(), c.blades(), c.material(), Double.NaN)), "compressor.designRpm"));
        // gas at 1500 K melts lightly cooled Inconel 713C blades (service 1253 K)
        assertTrue(invalid(with(e, "control", new JetControlDesign("k", k.fuel(), 1500, k.idleFraction(), k.maxSpeedFraction(), true, k.surgeReserve(),
                k.starterTorqueNm(), k.lightOffFraction(), Double.NaN)), "control.turbineInletK"));
        // too cold a turbine cannot drive the compressor
        assertTrue(invalid(with(e, "control", new JetControlDesign("k", k.fuel(), 700, k.idleFraction(), k.maxSpeedFraction(), true, k.surgeReserve(),
                k.starterTorqueNm(), k.lightOffFraction(), Double.NaN)), "cycle"));
        // an aluminium tailpipe in a 1000 K exhaust
        assertTrue(invalid(with(e, "nozzle", new JetNozzleDesign("n", false, 1, 0.98, 0.25, EngineMaterial.ALUMINIUM_2618, Double.NaN)), "nozzle.material"));
        // an inlet too small for 20 kg/s
        assertTrue(invalid(with(e, "inlet", new JetInletDesign("i", e.inlet().kind(), 0.05, 0.98, 0.1, EngineMaterial.ALUMINIUM_7075, Double.NaN)),
                "inlet.captureAreaM2"));
        // one small turbine stage cannot take the compressor's work
        assertTrue(invalid(with(e, "turbine", new TurbineDesign("t", 1, t.polytropicEfficiency(), 0.30, t.coolingEffectiveness(), t.coolingBleed(),
                t.mechanicalEfficiency(), t.material(), Double.NaN)), "turbine.stages"));
    }

    @Test void bothSamplesStartIdleRunUpToDesignThrustAndSpoolDown() {
        for (JetEngineDesign e : List.of(SampleEngines.turbojet(), SampleEngines.modelTurbojet())) {
            String name = e.name();
            Stand s = new Stand(e);
            double designThrust = e.sizing().point().thrustN(), idle = e.control().idleFraction();
            Map<String, Double> d = s.run(8);
            assertEquals(2, s.state.internal("jet", JetEngineDesign.STATE_LIT), 0, name + ": lit and running on its own");
            assertEquals(0, d.get("starter"), 0, name + ": the starter has cut out");
            assertEquals(idle, d.get("speedFraction"), 0.03, name + ": idles");
            s.controls.put("throttle", 1.0);
            d = s.run(8);
            assertEquals(1, d.get("speedFraction"), 0.01, name + ": full speed");
            assertEquals(designThrust, d.get("thrustN"), 0.01 * designThrust, name + ": design thrust");
            assertTrue(d.get("turbineInletK") <= e.control().turbineInletK() + 1, name + ": held to its turbine temperature");
            assertEquals(0, s.maxSurging, 0, name + ": the acceleration schedule keeps it out of surge");
            // the thrust acts along the part's axis
            AppliedForce thrust = s.last.forces().stream().filter(f -> f.kind() == AppliedForce.Kind.THRUST).findFirst().orElseThrow();
            assertEquals(d.get("thrustN"), thrust.force().x(), 1e-9 * designThrust);
            s.controls.put("throttle", 0.0);
            d = s.run(6);
            assertEquals(idle, d.get("speedFraction"), 0.03, name + ": back to idle");
            // its internals survive saving
            VehicleState back = VehicleState.fromJson(s.vehicle, Json.parse(Json.write(s.state.toJson())), DT);
            for (int i = 0; i < 3; i++) { assertEquals(s.state.internal("jet", i), back.internal("jet", i), 1e-12); }
            s.controls.put("ignition", 0.0);
            d = s.run(3);
            assertEquals(0, d.get("lit"), 0, name + ": out");
            assertTrue(d.get("speedFraction") < 0.8 * idle, name + ": spools down " + d.get("speedFraction"));
        }
    }

    @Test void withoutItsAccelerationScheduleASlammedThrottleSurges() {
        JetEngineDesign e = SampleEngines.turbojet();
        JetControlDesign k = e.control();
        Stand s = new Stand(with(e, "control", new JetControlDesign("k", k.fuel(), k.turbineInletK(), k.idleFraction(), k.maxSpeedFraction(), false,
                k.surgeReserve(), k.starterTorqueNm(), k.lightOffFraction(), Double.NaN)));
        // (its start, fuelled to the turbine temperature limit alone, may stall too)
        assertEquals(0, s.run(8).get("surging"), 0, "settled at idle");
        s.maxSurging = 0;
        s.controls.put("throttle", 1.0);
        s.run(1);
        assertEquals(1, s.maxSurging, 0, "over-fuelled, the compressor stalls");
        Map<String, Double> d = s.run(8);
        assertEquals(1, d.get("speedFraction"), 0.01, "and recovers at full speed");
        assertEquals(0, d.get("surging"), 0);
    }

    @Test void aDrownedInletPutsTheFlameOut() {
        Stand s = new Stand(SampleEngines.turbojet());
        s.run(8);
        assertEquals(1, s.d().get("lit"), 0);
        s.environment = VehicleEnvironment.calm(5, WATER, AIR);
        Map<String, Double> d = s.run(1);
        assertEquals(0, d.get("lit"), 0);
        assertEquals(0, d.get("thrustN"), 0);
    }
}

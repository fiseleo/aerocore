package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.engine.EngineOil;
import dev.mcmod.aerocore.api.engine.Friction;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class EngineDesignTest {
    private static final double DT = 0.005;
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid(), WATER = FluidState.FRESH_WATER_20C;

    private static boolean invalid(PistonEngineDesign e, String parameter) {
        return e.check().stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID && i.parameter().startsWith(parameter));
    }

    @Test void theSampleEngineIsValidAndItsBlueprintRoundTrips() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        assertTrue(e.check().stream().noneMatch(i -> i.kind() == DesignIssue.Kind.INVALID), e.check().toString());
        String json = Json.write(e.toJson());
        PartDesign back = PartTypes.read(Json.parse(json), "engine");
        assertEquals(e, back);
        assertEquals(json, Json.write(back.toJson()));
        assertEquals(1.998e-3, e.displacement(), 1e-6);
    }

    @Test void compressionRatioComesFromChamberGasketDeckAndCrown() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        assertEquals(11, e.compressionRatio(), 1e-9);
        double vd = e.cylinderDisplacement(), area = e.pistonArea();
        // a 5 cm^3 dome takes 5 cm^3 from the clearance volume
        PistonDesign domed = new PistonDesign("p", e.piston().diameterM(), e.piston().compressionHeightM(), 5e-6, e.piston().crownThicknessM(),
                e.piston().skirtLengthM(), e.piston().pinDiameterM(), 3, 0.8, EngineMaterial.ALUMINIUM_4032, Double.NaN);
        PistonEngineDesign d = with(e, "piston", domed);
        double vc = vd / 10 - 5e-6;
        assertEquals((vd + vc) / vc, d.compressionRatio(), 1e-9);
        // decking the block 0.5 mm: the crown comes 0.5 mm closer to the head
        EngineBlockDesign b = e.block();
        EngineBlockDesign decked = new EngineBlockDesign("b", b.layout(), b.cylinders(), b.bankAngleDeg(), b.boreM(), b.boreSpacingM(),
                b.deckHeightM() - 0.0005, b.mainBearings(), b.material(), b.wallThicknessM(), b.linerRoughness(), Double.NaN);
        PistonEngineDesign shorter = with(e, "block", decked);
        double vc2 = vd / 10 - 0.0005 * area;
        assertEquals((vd + vc2) / vc2, shorter.compressionRatio(), 1e-9);
        // decked past the gasket the piston hits the head
        EngineBlockDesign tooShort = new EngineBlockDesign("b", b.layout(), b.cylinders(), b.bankAngleDeg(), b.boreM(), b.boreSpacingM(),
                b.deckHeightM() - 0.002, b.mainBearings(), b.material(), b.wallThicknessM(), b.linerRoughness(), Double.NaN);
        assertTrue(invalid(with(e, "block", tooShort), "block.deckHeightM"));
    }

    @Test void partsThatDoNotFitEachOtherAreRejected() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        PistonDesign p = e.piston();
        PistonDesign big = new PistonDesign("p", e.bore() + 0.0001, p.compressionHeightM(), 0, p.crownThicknessM(), p.skirtLengthM(), p.pinDiameterM(),
                3, 0.8, p.material(), Double.NaN);
        assertTrue(invalid(with(e, "piston", big), "piston.diameterM"));
        ConnectingRodDesign r = e.rod();
        ConnectingRodDesign wrongPin = new ConnectingRodDesign("r", r.lengthM(), 0.045, r.bigEndWidthM(), r.smallEndBoreM(), r.beamAreaM2(), r.material(), Double.NaN);
        assertTrue(invalid(with(e, "rod", wrongPin), "rod.bigEndBoreM"));
        FuelSystemDesign f = e.fuelSystem();
        FuelSystemDesign direct = new FuelSystemDesign("f", "gasoline", 95, 85, FuelSystemDesign.Delivery.DIRECT_INJECTION, 1, 15e6, 0.88, 1, Double.NaN);
        assertTrue(invalid(with(e, "fuelSystem", direct), "injector.kind"));
        assertFalse(invalid(e, "injector"));
        assertNotNull(f);
    }

    @Test void valvesMustFitTheBoreAndInclinationMakesRoom() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        assertTrue(e.valvesFit());
        ValveDesign huge = new ValveDesign("v", 0.040, 0.0055, 0.034, 0.0055, 0.105, EngineMaterial.VALVE_STEEL, EngineMaterial.VALVE_STEEL, Double.NaN);
        assertTrue(invalid(with(e, "valves", huge), "valves"));
        // 35.5/30.5 mm valves fit an 86 mm bore with the head's 40 degree included angle, not as parallel valves: by hand,
        // rows of circles 3 mm apart with 1.5 mm to the wall reach x = 13.9 and 20.2 mm, 34.2 mm between centres where 36.0
        // are needed; tilted 20 degrees each, the ellipses reach about 14.5 and 20.9 mm and clear each other by about 4.5 mm.
        ValveDesign larger = new ValveDesign("v", 0.0355, 0.0055, 0.0305, 0.0055, 0.105, EngineMaterial.VALVE_STEEL, EngineMaterial.VALVE_STEEL, Double.NaN);
        PistonEngineDesign inclined = with(e, "valves", larger);
        assertTrue(inclined.valvesFit());
        CylinderHeadDesign h = e.head();
        CylinderHeadDesign parallel = new CylinderHeadDesign("h", h.layout(), 2, 2, h.chamber(), h.chamberVolumeM3(), 0, h.intakeThroatRatio(),
                h.exhaustThroatRatio(), h.portFlowCoefficient(), 1, 0, h.gasketThicknessM(), h.material(), Double.NaN);
        assertFalse(with(inclined, "head", parallel).valvesFit());
    }

    @Test void theSampleEngineHasAProductionLikeFullLoadCurve() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        PistonEngineDesign.EngineMap map = e.buildMap();
        Friction.Engine f = e.frictionModel();
        double scale = e.displacement() / (4 * Math.PI), peakTorque = 0, peakPower = 0, bestBsfc = Double.MAX_VALUE;
        int top = map.pressures().length - 1;
        for (int i = 0; i < map.rpms().length; i++) {
            PistonEngineDesign.MapPoint p = map.at(i, top);
            if (p.rpm() > e.ignition().revLimitRpm()) { continue; }
            double fmep = Friction.fmep(f, p.rpm(), 101325, 101325, EngineOil.W5_20.frictionScaling(90)).totalPa();
            double torque = (p.netImepPa() - fmep) * scale, power = torque * p.rpm() * Math.PI / 30;
            peakTorque = Math.max(peakTorque, torque); peakPower = Math.max(peakPower, power);
            if (p.rpm() > 1500) { bestBsfc = Math.min(bestBsfc, 4 * p.fuelKg() * p.rpm() / 120 / power * 3.6e9); }
        }
        double bmep = peakTorque * 4 * Math.PI / e.displacement();
        // Naturally aspirated four-valve petrol engines with tuned intakes and cam phasing: 10-13 bar BMEP, 45-80 kW per litre,
        // full-load BSFC (rich) 250-320 g/kWh (Heywood ch. 15; production 2.0 l engines).
        assertTrue(bmep > 10e5 && bmep < 13.5e5, "BMEP " + bmep);
        assertTrue(peakPower / 1000 / 2.0 > 45 && peakPower / 1000 / 2.0 < 80, "kW/l " + peakPower / 2000);
        assertTrue(bestBsfc > 240 && bestBsfc < 320, "BSFC " + bestBsfc);
    }

    private static Vehicle bench(PistonEngineDesign e) {
        VehicleBlueprint b = new VehicleBlueprint("bench", List.of(new VehicleBlueprint.Entry("engine", e, Mount.at(Vec3d.ZERO),
                List.of(Binding.of("throttle", "throttle"), Binding.of("ignition", "ignition")))), List.of());
        return b.compile();
    }

    @Test void itStartsIdlesRevsToItsLimiterSlowsAndStops() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        Vehicle v = bench(e);
        v.warmUp();
        VehicleState st = v.newState(Vec3d.ZERO, Orientation.IDENTITY, DT);
        VehicleEnvironment env = VehicleEnvironment.calm(-100, WATER, AIR);
        Map<String, Double> controls = new HashMap<>(Map.of("throttle", 0.0, "ignition", 1.0));
        double t = 0;
        double[] rpm = new double[1];
        java.util.function.DoubleConsumer run = seconds -> {};
        for (int i = 0; i < 4 / DT; i++, t += DT) {
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, controls, Vec3d.ZERO, List.of(), null, t, DT);
            rpm[0] = r.diagnostics().get("engine").get("rpm");
        }
        assertEquals(1, st.internal("engine", PistonEngineDesign.STATE_RUNNING), 0, "the starter got it running");
        assertEquals(e.ignition().idleRpm(), rpm[0], 300, "it idles");
        double startTemperature = st.internal("engine", PistonEngineDesign.STATE_TEMPERATURE);
        controls.put("throttle", 1.0);
        double peak = 0;
        for (int i = 0; i < 4 / DT; i++, t += DT) {
            VehicleDynamics.Result r = VehicleDynamics.step(v, st, env, controls, Vec3d.ZERO, List.of(), null, t, DT);
            peak = Math.max(peak, r.diagnostics().get("engine").get("rpm"));
        }
        assertTrue(peak > 0.9 * e.ignition().revLimitRpm() && peak < 1.05 * e.ignition().revLimitRpm(), "revs to the limiter: " + peak);
        assertTrue(st.internal("engine", PistonEngineDesign.STATE_TEMPERATURE) > startTemperature, "it warms up");
        controls.put("throttle", 0.0);
        for (int i = 0; i < 4 / DT; i++, t += DT) {
            rpm[0] = VehicleDynamics.step(v, st, env, controls, Vec3d.ZERO, List.of(), null, t, DT).diagnostics().get("engine").get("rpm");
        }
        assertTrue(rpm[0] < 2000, "back towards idle: " + rpm[0]);
        controls.put("ignition", 0.0);
        for (int i = 0; i < 4 / DT; i++, t += DT) {
            rpm[0] = VehicleDynamics.step(v, st, env, controls, Vec3d.ZERO, List.of(), null, t, DT).diagnostics().get("engine").get("rpm");
        }
        assertTrue(rpm[0] < 50, "stopped: " + rpm[0]);
        assertEquals(0, st.internal("engine", PistonEngineDesign.STATE_RUNNING), 0);
        // its internals survive saving
        VehicleState back = VehicleState.fromJson(v, Json.parse(Json.write(st.toJson())), DT);
        for (int i = 0; i < 3; i++) { assertEquals(st.internal("engine", i), back.internal("engine", i), 1e-12); }
        assertNotNull(run);
    }

    @Test void aColdEngineFightsMoreFriction() {
        PistonEngineDesign e = SampleEngines.inlineFour();
        Friction.Engine f = e.frictionModel();
        EngineOil oil = e.lubrication().oil();
        double cold = Friction.fmep(f, 2000, 101325, 101325, oil.frictionScaling(15)).total();
        double warm = Friction.fmep(f, 2000, 101325, 101325, oil.frictionScaling(90)).total();
        assertTrue(cold > 1.5 * warm, "cold " + cold + " warm " + warm);
    }

    /** The design with one component replaced. */
    static PistonEngineDesign with(PistonEngineDesign e, String key, EngineComponent c) {
        Map<String, EngineComponent> m = new java.util.LinkedHashMap<>(e.components());
        m.put(key, c);
        return new PistonEngineDesign(e.name(), (EngineBlockDesign) m.get("block"), (CylinderHeadDesign) m.get("head"), (PistonDesign) m.get("piston"),
                (ConnectingRodDesign) m.get("rod"), (CrankshaftDesign) m.get("crank"), (ValveDesign) m.get("valves"), (CamshaftDesign) m.get("camshaft"),
                (SparkPlugDesign) m.get("sparkPlug"), (FuelInjectorDesign) m.get("injector"), (ValvetrainDesign) m.get("valvetrain"),
                (FuelSystemDesign) m.get("fuelSystem"), (IgnitionDesign) m.get("ignition"), (LubricationDesign) m.get("lubrication"),
                (CoolingDesign) m.get("cooling"), (IntakeDesign) m.get("intake"), (ExhaustDesign) m.get("exhaust"));
    }
}

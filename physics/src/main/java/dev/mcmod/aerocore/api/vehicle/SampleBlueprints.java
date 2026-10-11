package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.TestHull;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;
import java.util.Map;

/**
 * Example blueprints built from AeroCore's own parts, on the scale of its test bodies: starting points to copy and the
 * vehicles commands and tests spawn. Channels: "throttle" (-1..1) and "rudder" (-1..1, positive turns to starboard).
 * They are examples of the format and the physics, not designs a vehicle mod should ship.
 */
public final class SampleBlueprints {
    private SampleBlueprints() {}

    /** Name to blueprint, for /aerocore vehicle samples. */
    public static Map<String, VehicleBlueprint> all() {
        return Map.of("platform", platform(), "launch", launch(), "cruiser", cruiser());
    }

    /**
     * The test platform's layout as parts: four 0.45 m steel floats at the corners of a 1.7 m square, an open 300 kg
     * frame, 200 kg of ballast, a 3 kW motor turning a 0.3 m B3-65 screw at the stern and a rudder in its race.
     */
    public static VehicleBlueprint platform() {
        PartDesign float1 = new FloatDesign("steel float", 0.45, MassSpec.shell(Material.STEEL, 0.004));
        return new VehicleBlueprint("platform", List.of(
                new VehicleBlueprint.Entry("float_fl", float1, Mount.at(new Vec3d(0.85, 0, -0.85))),
                new VehicleBlueprint.Entry("float_fr", float1, Mount.at(new Vec3d(0.85, 0, 0.85))),
                new VehicleBlueprint.Entry("float_al", float1, Mount.at(new Vec3d(-0.85, 0, -0.85))),
                new VehicleBlueprint.Entry("float_ar", float1, Mount.at(new Vec3d(-0.85, 0, 0.85))),
                new VehicleBlueprint.Entry("frame", new BoxDesign("frame", new Vec3d(2.2, 0.1, 2.2), 1.17, false, 2, MassSpec.given(300)), Mount.at(new Vec3d(0, 0.5, 0))),
                new VehicleBlueprint.Entry("ballast", new BallastDesign("ballast", 200), Mount.at(new Vec3d(0, -0.35, 0))),
                new VehicleBlueprint.Entry("motor", new MotorDesign("3 kW motor", 40, 3000, 3000, 0.004, 0.9, 15), Mount.at(new Vec3d(-1.1, 0.3, 0)),
                        List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("screw", new PropellerDesign("B3-65", 0.3, 3, 0.65, 1.0, 0.18, 0.1, MassSpec.solid(Material.BRONZE)),
                        Mount.at(new Vec3d(-1.4, -0.4, 0))),
                new VehicleBlueprint.Entry("rudder", new ControlSurfaceDesign("rudder", 0.25, 0.35, 35, true, 0.15, MassSpec.solid(Material.STEEL)),
                        Mount.at(new Vec3d(-1.65, -0.4, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("seat", new SeatDesign("seat", 5), Mount.at(new Vec3d(0, 0.55, 0)))),
                List.of(new Connection.Shaft("motor", "screw", 1, 0.97), new Connection.Slipstream("screw", "rudder")));
    }

    /**
     * A 6 m planing launch on Fridsma's hull (B 1.2 m, 20 deg deadrise), about the test planing boat's 1048 kg: a 60 kW
     * motor through a 1.5 reduction to a 0.35 m B4-70 P/D 1.4 screw under the transom, a spade rudder in its race.
     */
    public static VehicleBlueprint launch() {
        return new VehicleBlueprint("launch", List.of(
                new VehicleBlueprint.Entry("hull", new PlaningHullDesign("fridsma hull", 6, 1.2, 20, 0.75, 48, 6, MassSpec.given(880)), Mount.ORIGIN),
                new VehicleBlueprint.Entry("motor", new MotorDesign("60 kW motor", 250, 60000, 4000, 0.05, 0.94, 60), Mount.at(new Vec3d(1.2, 0.25, 0)),
                        List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("screw", new PropellerDesign("B4-70 P/D 1.4", 0.35, 4, 0.70, 1.4, 0.18, 0.1, MassSpec.solid(Material.BRONZE)),
                        Mount.at(new Vec3d(-0.25, -0.2, 0))),
                new VehicleBlueprint.Entry("rudder", new ControlSurfaceDesign("rudder", 0.22, 0.32, 35, true, 0.15, MassSpec.solid(Material.STEEL)),
                        Mount.at(new Vec3d(-0.5, -0.22, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("ballast", new BallastDesign("fittings", 80), Mount.at(new Vec3d(2.2, 0.3, 0))),
                new VehicleBlueprint.Entry("seat", new SeatDesign("helm", 10), Mount.at(new Vec3d(2.0, 0.35, 0)))),
                List.of(new Connection.Shaft("motor", "screw", 1.5, 0.96), new Connection.Slipstream("screw", "rudder")));
    }

    /**
     * The 6 m test hull as a displacement launch of 1200 kg: a 10 kW motor through a 2:1 reduction to a 0.4 m B3-65
     * screw, a rudder behind it.
     */
    public static VehicleBlueprint cruiser() {
        HullDesign hull = new HullDesign("test hull", HullModel.Type.SURFACE, HullDesign.stationsOf(TestHull.HULL), TestHull.MASS, 0, 6,
                new MassSpec.Given(1050, TestHull.CENTER_OF_MASS));
        return new VehicleBlueprint("cruiser", List.of(
                new VehicleBlueprint.Entry("hull", hull, Mount.ORIGIN),
                new VehicleBlueprint.Entry("motor", new MotorDesign("10 kW motor", 60, 10000, 3000, 0.01, 0.92, 30), Mount.at(new Vec3d(-2.2, -0.1, 0)),
                        List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("screw", new PropellerDesign("B3-65", 0.4, 3, 0.65, 1.0, 0.18, 0.1, MassSpec.solid(Material.BRONZE)),
                        Mount.at(new Vec3d(-3.15, -0.38, 0))),
                new VehicleBlueprint.Entry("rudder", new ControlSurfaceDesign("rudder", 0.30, 0.50, 35, true, 0.15, MassSpec.solid(Material.STEEL)),
                        Mount.at(new Vec3d(-3.45, -0.42, 0)), List.of(Binding.of("deflection", "rudder"))),
                new VehicleBlueprint.Entry("ballast", new BallastDesign("keel ballast", 100), Mount.at(new Vec3d(0, -0.3, 0))),
                new VehicleBlueprint.Entry("seat", new SeatDesign("helm", 10), Mount.at(new Vec3d(-1.5, 0.45, 0)))),
                List.of(new Connection.Shaft("motor", "screw", 2, 0.97), new Connection.Slipstream("screw", "rudder")));
    }
}

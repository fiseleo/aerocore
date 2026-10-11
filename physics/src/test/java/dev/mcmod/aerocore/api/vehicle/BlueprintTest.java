package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class BlueprintTest {
    @Test void jsonRoundTripsValuesExactly() {
        Object value = Json.parse("{\"a\": [1, 2.5, -3e-7, 0.1], \"b\": {\"s\": \"x\\\"y\\n\", \"t\": true, \"n\": null}}");
        String text = Json.write(value);
        assertEquals(value, Json.parse(text));
        assertEquals(value, Json.parse(Json.pretty(value)));
        // Shortest round-trip doubles, integers without a fraction.
        assertEquals("[1,0.1,1.0E-300]", Json.write(List.of(1.0, 0.1, 1e-300)));
        assertEquals(0.30000000000000004, ((Number) ((List<?>) Json.parse(Json.write(List.of(0.1 + 0.2)))).getFirst()).doubleValue());
    }

    @Test void jsonRejectsMalformedInput() {
        assertThrows(IllegalArgumentException.class, () -> Json.parse("{\"a\": 1, \"a\": 2}"), "duplicate key");
        assertThrows(IllegalArgumentException.class, () -> Json.parse("[1, 2"));
        assertThrows(IllegalArgumentException.class, () -> Json.parse("{\"a\": 1} x"));
        assertThrows(IllegalArgumentException.class, () -> Json.parse("[".repeat(100) + "]".repeat(100)), "nesting limit");
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> Json.parse("{\"a\": tru}"));
        assertTrue(e.getMessage().contains("character"), e.getMessage());
    }

    static VehicleBlueprint everyPart() {
        PartDesign hull = HullDesign.tapered("hull", 6, 0.7, -0.45, 0.45, 0.7, 2, 13, 1200, MassSpec.shell(Material.GRP, 0.01));
        PartDesign planing = new PlaningHullDesign("planing", 6, 1.2, 20, 0.75, 24, 6, MassSpec.given(1048));
        PartDesign prop = new PropellerDesign("screw", 0.35, 4, 0.70, 1.4, 0.18, 0.1, MassSpec.solid(Material.BRONZE));
        PartDesign rudder = new ControlSurfaceDesign("rudder", 0.22, 0.32, 35, true, 0.15, MassSpec.solid(Material.STEEL));
        PartDesign motor = new MotorDesign("motor", 120, 30000, 3000, 0.05, 0.92, 40);
        PartDesign box = new BoxDesign("cabin", new Vec3d(2, 1, 1.4), 1.05, true, 2, MassSpec.shell(new Material("pine", 500), 0.02));
        PartDesign float1 = new FloatDesign("float", 0.45, MassSpec.shell(Material.STEEL, 0.004));
        return new VehicleBlueprint("everything", List.of(
                new VehicleBlueprint.Entry("hull", hull, Mount.ORIGIN),
                new VehicleBlueprint.Entry("planing", planing, new Mount(new Vec3d(0, -1, 0), 180, 0, 0)),
                new VehicleBlueprint.Entry("prop", prop, Mount.at(new Vec3d(-3.2, -0.3, 0))),
                new VehicleBlueprint.Entry("rudder", rudder, Mount.at(new Vec3d(-3.5, -0.3, 0)), List.of(new Binding("deflection", "rudder", -1, 0))),
                new VehicleBlueprint.Entry("motor", motor, Mount.at(new Vec3d(-2, 0, 0)), List.of(Binding.of("throttle", "throttle"))),
                new VehicleBlueprint.Entry("cabin", box, Mount.at(new Vec3d(0, 1, 0))),
                new VehicleBlueprint.Entry("float", float1, new Mount(new Vec3d(0, 0, 2), 10, 20, 30)),
                new VehicleBlueprint.Entry("ballast", new BallastDesign("lead", 200), Mount.at(new Vec3d(0.5, -0.3, 0))),
                new VehicleBlueprint.Entry("seat", new SeatDesign("seat", 15), Mount.at(new Vec3d(-1, 0.45, 0)))),
                List.of(new Connection.Shaft("motor", "prop", 2, 0.97), new Connection.Slipstream("prop", "rudder")));
    }

    @Test void blueprintsRoundTripThroughJson() {
        VehicleBlueprint b = everyPart();
        String text = b.toJsonText();
        VehicleBlueprint read = VehicleBlueprint.fromJsonText(text);
        assertEquals(b, read, "every design, mount, binding and connection survives");
        assertEquals(text, read.toJsonText());
        assertTrue(read.check().stream().noneMatch(i -> i.kind() == DesignIssue.Kind.INVALID), read.check().toString());
    }

    @Test void unphysicalValuesAreNamedAndStopCompiling() {
        VehicleBlueprint bad = new VehicleBlueprint("bad", List.of(
                new VehicleBlueprint.Entry("f", new FloatDesign("f", -1, MassSpec.shell(Material.STEEL, 0.01)), Mount.ORIGIN),
                new VehicleBlueprint.Entry("p", new PropellerDesign("p", 0.3, 1, 0.5, 1, 0.18, 0.1, MassSpec.solid(Material.BRONZE)), Mount.ORIGIN),
                new VehicleBlueprint.Entry("p", new BallastDesign("b", 1), Mount.ORIGIN)),
                List.of(new Connection.Shaft("f", "nowhere", 0, 2)));
        List<DesignIssue> issues = bad.check();
        List<String> paths = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).map(DesignIssue::parameter).toList();
        assertTrue(paths.contains("parts.f.radiusM"), paths.toString());
        assertTrue(paths.contains("parts.p.blades"), paths.toString());
        assertTrue(paths.contains("parts.p"), "duplicate id: " + paths);
        assertTrue(paths.contains("connections[0].to") && paths.contains("connections[0].ratio") && paths.contains("connections[0].efficiency"), paths.toString());
        DesignException e = assertThrows(DesignException.class, bad::compile);
        assertFalse(e.issues().isEmpty());
    }

    @Test void valuesOutsideValidatedRangesCompileWithWarnings() {
        // A five-blade, 1.8 pitch screw: outside the B-series, so blade-element theory and an extrapolation note.
        PropellerDesign wide = new PropellerDesign("wide", 0.4, 5, 0.8, 1.8, 0.18, 0.1, MassSpec.solid(Material.BRONZE));
        assertFalse(wide.inBSeriesRange());
        Part part = wide.compile();
        assertTrue(part.issues().stream().anyMatch(i -> i.parameter().equals("pitchRatio") && i.kind() == DesignIssue.Kind.EXTRAPOLATION));
        // A barge (L/B 3): Holtrop's form factor is extrapolated.
        HullDesign barge = HullDesign.tapered("barge", 6, 1, -0.5, 0.5, 0.2, 8, 13, 4000, MassSpec.shell(Material.STEEL, 0.006));
        List<DesignIssue> hull = barge.compile().issues();
        assertTrue(hull.stream().anyMatch(i -> i.parameter().equals("lengthBeamRatio")), hull.toString());
        // An over-powered motor.
        assertTrue(new MotorDesign("m", 100, 100000, 3000, 0.01, 0.9, 5).check().stream().anyMatch(i -> i.parameter().equals("specificPowerWPerKg")));
    }

    @Test void bindingsMustNameAnInputOfThePart() {
        VehicleBlueprint b = new VehicleBlueprint("b", List.of(
                new VehicleBlueprint.Entry("f", new FloatDesign("f", 0.5, MassSpec.shell(Material.STEEL, 0.01)), Mount.ORIGIN, List.of(Binding.of("throttle", "t")))),
                List.of());
        DesignException e = assertThrows(DesignException.class, b::compile);
        assertTrue(e.getMessage().contains("throttle"), e.getMessage());
    }

    @Test void customPartTypesNeedANamespacedIdAndReadBack() {
        assertThrows(IllegalArgumentException.class, () -> PartTypes.register("NoNamespace", f -> null));
        assertThrows(IllegalArgumentException.class, () -> PartTypes.register(FloatDesign.TYPE, f -> null), "already registered");
        assertTrue(PartTypes.types().containsAll(List.of(FloatDesign.TYPE, HullDesign.TYPE, PropellerDesign.TYPE, MotorDesign.TYPE)));
        Map<String, Object> json = new FloatDesign("f", 0.5, MassSpec.given(10)).toJson();
        json.put("format", PartDesign.FORMAT + 1);
        assertThrows(IllegalArgumentException.class, () -> PartTypes.read(json, "x"), "a newer format is refused");
    }

    @Test void hullStationsKeepTheirShapes() {
        HullDesign h = HullDesign.tapered("h", 6, 0.7, -0.45, 0.45, 0.7, 2, 13, 1200, MassSpec.shell(Material.GRP, 0.01));
        assertEquals(HullModel.Type.SURFACE, h.hullType());
        HullDesign back = (HullDesign) PartTypes.read(Json.parse(Json.write(h.toJson())), "h");
        assertEquals(h.stations(), back.stations());
    }
}

package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class BlueprintTest {
    /** A part type of the tests: a sphere of mass, with a driving rotor (and a throttle) or a driven one, registered as "test:part". */
    record TestPart(String name, double radiusM, double massKg, String rotor) implements PartDesign {
        static final String TYPE = "test:part";
        static {
            if (!PartTypes.types().contains(TYPE)) {
                PartTypes.register(TYPE, f -> new TestPart(f.string("name", "part"), f.number("radiusM"), f.number("massKg"), f.string("rotor", "none")));
            }
        }
        static void register() {}

        @Override public String type() { return TYPE; }
        @Override public List<DesignIssue> check() {
            List<DesignIssue> issues = new ArrayList<>();
            if (!(radiusM > 0)) { issues.add(DesignIssue.invalid("radiusM", "Radius must be positive")); }
            DesignIssue.range(issues, "massKg", massKg, 0, 1000, "test range");
            return issues;
        }
        @Override public Part compile() {
            List<DesignIssue> issues = check();
            DesignException.throwIfInvalid(issues);
            MassProperties mass = MassProperties.point(massKg, Vec3d.ZERO);
            PartDesign self = this;
            return new Part() {
                @Override public PartDesign design() { return self; }
                @Override public MassProperties mass() { return mass; }
                @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, radiusM)); }
                @Override public List<DesignIssue> issues() { return issues; }
                @Override public List<Rotor> rotors() { return rotor.equals("none") ? List.of() : List.of(new Rotor(Vec3d.X, 0.1, rotor.equals("driver"))); }
                @Override public List<String> inputs() { return rotor.equals("driver") ? List.of("throttle") : List.of(); }
                @Override public void loads(PartStep step) {}
            };
        }
        @Override public Map<String, Object> parameters() { return Fields.object("radiusM", radiusM, "massKg", massKg, "rotor", rotor); }
    }

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

    @Test void blueprintsRoundTripThroughJson() {
        TestPart.register();
        VehicleBlueprint b = new VehicleBlueprint("parts", List.of(
                new VehicleBlueprint.Entry("motor", new TestPart("motor", 0.2, 40, "driver"), new Mount(new Vec3d(-2, 0, 0), 180, 0, 0),
                        List.of(new Binding("throttle", "throttle", -1, 0.5))),
                new VehicleBlueprint.Entry("wheel", new TestPart("wheel", 0.3, 10, "driven"), new Mount(new Vec3d(0, 0, 2), 10, 20, 30)),
                new VehicleBlueprint.Entry("body", new TestPart("body", 1, 500, "none"), Mount.ORIGIN)),
                List.of(new Connection.Shaft("motor", "wheel", 2, 0.97), new Connection.Slipstream("motor", "body")));
        String text = b.toJsonText();
        VehicleBlueprint read = VehicleBlueprint.fromJsonText(text);
        assertEquals(b, read, "every design, mount, binding and connection survives");
        assertEquals(text, read.toJsonText());
        assertTrue(read.check().stream().noneMatch(i -> i.kind() == DesignIssue.Kind.INVALID), read.check().toString());
        assertEquals(550, read.compile().properties().massKg(), 1e-12);
    }

    @Test void unphysicalValuesAreNamedAndStopCompiling() {
        VehicleBlueprint bad = new VehicleBlueprint("bad", List.of(
                new VehicleBlueprint.Entry("f", new TestPart("f", -1, 10, "none"), Mount.ORIGIN),
                new VehicleBlueprint.Entry("p", new TestPart("p", 1, 10, "none"), Mount.ORIGIN),
                new VehicleBlueprint.Entry("p", new TestPart("q", 1, 10, "none"), Mount.ORIGIN)),
                List.of(new Connection.Shaft("f", "nowhere", 0, 2)));
        List<DesignIssue> issues = bad.check();
        List<String> paths = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).map(DesignIssue::parameter).toList();
        assertTrue(paths.contains("parts.f.radiusM"), paths.toString());
        assertTrue(paths.contains("parts.p"), "duplicate id: " + paths);
        assertTrue(paths.contains("connections[0].to") && paths.contains("connections[0].ratio") && paths.contains("connections[0].efficiency"), paths.toString());
        DesignException e = assertThrows(DesignException.class, bad::compile);
        assertFalse(e.issues().isEmpty());
    }

    @Test void valuesOutsideValidatedRangesCompileWithWarnings() {
        Part part = new TestPart("heavy", 1, 5000, "none").compile();
        assertTrue(part.issues().stream().anyMatch(i -> i.parameter().equals("massKg") && i.kind() == DesignIssue.Kind.EXTRAPOLATION));
        DesignIssue under = DesignIssue.extrapolation("massKg", 5000, 0, 1000, "test range").under("parts.heavy");
        assertEquals("parts.heavy.massKg", under.parameter());
    }

    @Test void bindingsMustNameAnInputOfThePart() {
        VehicleBlueprint b = new VehicleBlueprint("b", List.of(
                new VehicleBlueprint.Entry("f", new TestPart("f", 0.5, 10, "none"), Mount.ORIGIN, List.of(Binding.of("throttle", "t")))),
                List.of());
        DesignException e = assertThrows(DesignException.class, b::compile);
        assertTrue(e.getMessage().contains("throttle"), e.getMessage());
    }

    @Test void customPartTypesNeedANamespacedIdAndReadBack() {
        TestPart.register();
        assertThrows(IllegalArgumentException.class, () -> PartTypes.register("NoNamespace", f -> null));
        assertThrows(IllegalArgumentException.class, () -> PartTypes.register(TestPart.TYPE, f -> null), "already registered");
        assertTrue(PartTypes.types().contains(TestPart.TYPE));
        TestPart part = new TestPart("t", 0.5, 10, "none");
        assertEquals(part, PartTypes.read(Json.parse(Json.write(part.toJson())), "t"));
        Map<String, Object> json = part.toJson();
        json.put("format", PartDesign.FORMAT + 1);
        assertThrows(IllegalArgumentException.class, () -> PartTypes.read(json, "x"), "a newer format is refused");
        assertThrows(IllegalArgumentException.class, () -> PartTypes.read(Json.parse("{\"type\": \"test:unknown\"}"), "x"), "an unknown type");
    }
}

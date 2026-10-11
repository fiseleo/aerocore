package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.Turbojet;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Inlet (進氣道) of a jet engine: a pitot intake (a normal shock above Mach 1) or an external-compression ramp
 * (MIL-E-5008B recovery), capture area, subsonic total-pressure recovery, duct length, material. The capture area
 * must pass the design airflow below Mach 1 (241 kg/(s m^2) of corrected flow chokes it). Mass: the duct as a 2 mm
 * shell 0.4 % of its diameter thick, at least 0.3 mm (estimate).
 */
public record JetInletDesign(String name, Turbojet.Inlet kind, double captureAreaM2, double recovery, double lengthM, EngineMaterial material,
                             double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:jet_inlet";

    public JetInletDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(kind, "kind"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "captureAreaM2", captureAreaM2);
        if (!(recovery > 0.5 && recovery <= 1)) { issues.add(DesignIssue.invalid("recovery", "Recovery must be in (0.5, 1]")); }
        else { DesignIssue.range(issues, "recovery", recovery, 0.93, 0.995, "subsonic inlets"); }
        EngineComponent.Rules.nonnegative(issues, "lengthM", lengthM);
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    @Override public double massKg() {
        double d = Math.sqrt(4 * captureAreaM2 / Math.PI);
        return Double.isNaN(massGivenKg) ? Math.PI * d * lengthM * EngineComponent.Rules.shell(d) * material.densityKgPerM3() : massGivenKg;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("kind", kind.name().toLowerCase(), "captureAreaM2", captureAreaM2, "recovery", recovery, "lengthM", lengthM,
                "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static JetInletDesign read(String name, Fields f) {
        return new JetInletDesign(name, EngineComponent.Rules.choice(f, "kind", Turbojet.Inlet.class, Turbojet.Inlet.PITOT), f.number("captureAreaM2"),
                f.number("recovery", 0.98), f.number("lengthM", 0.3), EngineComponent.Rules.material(f, "material", EngineMaterial.ALUMINIUM_7075),
                EngineComponent.Rules.massOr(f));
    }
}

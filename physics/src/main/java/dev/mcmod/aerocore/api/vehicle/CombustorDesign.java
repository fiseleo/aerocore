package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Combustion chamber (燃燒室) of a jet engine: total-pressure loss (a fraction of the burner inlet pressure; liners
 * lose about 3-8 % to mix and stabilise the flame), combustion efficiency, length and casing diameter, material.
 * Flame stability and liner temperature are not modelled. Mass: casing and liner as two shells 0.4 % of the casing
 * diameter thick, at least 0.3 mm (estimate).
 */
public record CombustorDesign(String name, double pressureLoss, double efficiency, double lengthM, double casingDiameterM, EngineMaterial material,
                              double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:combustor";

    public CombustorDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(pressureLoss >= 0 && pressureLoss < 0.3)) { issues.add(DesignIssue.invalid("pressureLoss", "Pressure loss must be in [0, 0.3)")); }
        else { DesignIssue.range(issues, "pressureLoss", pressureLoss, 0.03, 0.08, "combustor liners"); }
        if (!(efficiency > 0.5 && efficiency <= 1)) { issues.add(DesignIssue.invalid("efficiency", "Combustion efficiency must be in (0.5, 1]")); }
        else { DesignIssue.range(issues, "efficiency", efficiency, 0.95, 0.999, "gas turbine combustors at power"); }
        EngineComponent.Rules.positive(issues, "lengthM", lengthM);
        EngineComponent.Rules.positive(issues, "casingDiameterM", casingDiameterM);
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    @Override public double massKg() {
        return Double.isNaN(massGivenKg) ? 2 * Math.PI * casingDiameterM * lengthM * EngineComponent.Rules.shell(casingDiameterM) * material.densityKgPerM3() : massGivenKg;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("pressureLoss", pressureLoss, "efficiency", efficiency, "lengthM", lengthM, "casingDiameterM", casingDiameterM,
                "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CombustorDesign read(String name, Fields f) {
        return new CombustorDesign(name, f.number("pressureLoss", 0.05), f.number("efficiency", 0.98), f.number("lengthM"), f.number("casingDiameterM"),
                EngineComponent.Rules.material(f, "material", EngineMaterial.INCONEL_718), EngineComponent.Rules.massOr(f));
    }
}

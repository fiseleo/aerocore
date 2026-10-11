package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Valves (氣門): intake and exhaust head and stem diameters, overall length, materials. Estimated mass of one valve: a
 * head disc 10 % of its diameter thick plus a solid stem over the length (a sodium-filled or hollow stem can be given
 * as a lower density material).
 */
public record ValveDesign(String name, double intakeHeadM, double intakeStemM, double exhaustHeadM, double exhaustStemM, double lengthM,
                          EngineMaterial intakeMaterial, EngineMaterial exhaustMaterial, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:valves";

    public ValveDesign {
        Objects.requireNonNull(name, "name"); Objects.requireNonNull(intakeMaterial, "intakeMaterial"); Objects.requireNonNull(exhaustMaterial, "exhaustMaterial");
    }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "intakeHeadM", intakeHeadM);
        EngineComponent.Rules.positive(issues, "intakeStemM", intakeStemM);
        EngineComponent.Rules.positive(issues, "exhaustHeadM", exhaustHeadM);
        EngineComponent.Rules.positive(issues, "exhaustStemM", exhaustStemM);
        EngineComponent.Rules.positive(issues, "lengthM", lengthM);
        if (intakeStemM >= 0.5 * intakeHeadM) { issues.add(DesignIssue.invalid("intakeStemM", "The stem must be well under the head diameter")); }
        if (exhaustStemM >= 0.5 * exhaustHeadM) { issues.add(DesignIssue.invalid("exhaustStemM", "The stem must be well under the head diameter")); }
        intakeMaterial.check(issues, "intakeMaterial");
        exhaustMaterial.check(issues, "exhaustMaterial");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    static double valveMass(double head, double stem, double length, EngineMaterial m) {
        return m.densityKgPerM3() * (Math.PI / 4 * head * head * 0.1 * head + Math.PI / 4 * stem * stem * length);
    }
    public double intakeValveMassKg() { return Double.isNaN(massGivenKg) ? valveMass(intakeHeadM, intakeStemM, lengthM, intakeMaterial) : massGivenKg / 2; }
    public double exhaustValveMassKg() { return Double.isNaN(massGivenKg) ? valveMass(exhaustHeadM, exhaustStemM, lengthM, exhaustMaterial) : massGivenKg / 2; }
    /** One intake and one exhaust valve, kg (a given mass is for that pair). */
    @Override public double massKg() { return intakeValveMassKg() + exhaustValveMassKg(); }

    @Override public Map<String, Object> parameters() {
        return Fields.object("intakeHeadM", intakeHeadM, "intakeStemM", intakeStemM, "exhaustHeadM", exhaustHeadM, "exhaustStemM", exhaustStemM,
                "lengthM", lengthM, "intakeMaterial", intakeMaterial.toJson(), "exhaustMaterial", exhaustMaterial.toJson(),
                "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static ValveDesign read(String name, Fields f) {
        return new ValveDesign(name, f.number("intakeHeadM"), f.number("intakeStemM", 0.0055), f.number("exhaustHeadM"), f.number("exhaustStemM", 0.0055),
                f.number("lengthM", 0.1), EngineComponent.Rules.material(f, "intakeMaterial", EngineMaterial.VALVE_STEEL),
                EngineComponent.Rules.material(f, "exhaustMaterial", EngineMaterial.VALVE_STEEL), EngineComponent.Rules.massOr(f));
    }
}

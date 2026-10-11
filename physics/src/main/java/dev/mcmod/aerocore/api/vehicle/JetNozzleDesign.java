package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Exhaust nozzle and tailpipe (噴嘴／排氣尾管): convergent, or convergent-divergent with a fixed exit-to-throat area
 * ratio (supersonic exhaust; it separates at less than 0.4 of ambient pressure), total-pressure retention of the
 * tailpipe, tailpipe length, material (its service temperature must take the turbine exit gas). The throat is sized by
 * the engine's design point. Mass: a shell of 1.2 times the throat's diameter (the exit's, if
 * divergent) over the tailpipe, 0.4 % of that diameter thick, at least 0.3 mm (estimate).
 */
public record JetNozzleDesign(String name, boolean convergentDivergent, double exitAreaRatio, double pressureRetention, double tailpipeLengthM,
                              EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:jet_nozzle";

    public JetNozzleDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (convergentDivergent) {
            if (!(exitAreaRatio >= 1)) { issues.add(DesignIssue.invalid("exitAreaRatio", "A convergent-divergent nozzle widens after its throat")); }
            else { DesignIssue.range(issues, "exitAreaRatio", exitAreaRatio, 1, 3, "fixed convergent-divergent nozzles"); }
        }
        if (!(pressureRetention > 0.5 && pressureRetention <= 1)) { issues.add(DesignIssue.invalid("pressureRetention", "Pressure retention must be in (0.5, 1]")); }
        else { DesignIssue.range(issues, "pressureRetention", pressureRetention, 0.95, 1, "tailpipes and nozzles"); }
        EngineComponent.Rules.nonnegative(issues, "tailpipeLengthM", tailpipeLengthM);
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Mass for a throat area, kg. */
    public double massFor(double throatAreaM2) {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double d = Math.sqrt(4 * throatAreaM2 / Math.PI) * (convergentDivergent ? Math.sqrt(exitAreaRatio) : 1.2);
        return Math.PI * d * Math.max(tailpipeLengthM, 0.1) * EngineComponent.Rules.shell(d) * material.densityKgPerM3();
    }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("convergentDivergent", convergentDivergent, "exitAreaRatio", exitAreaRatio, "pressureRetention", pressureRetention,
                "tailpipeLengthM", tailpipeLengthM, "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static JetNozzleDesign read(String name, Fields f) {
        return new JetNozzleDesign(name, f.bool("convergentDivergent", false), f.number("exitAreaRatio", 1), f.number("pressureRetention", 0.98),
                f.number("tailpipeLengthM", 0.4), EngineComponent.Rules.material(f, "material", EngineMaterial.STAINLESS_321), EngineComponent.Rules.massOr(f));
    }
}

package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Spark plug (火星塞): thread diameter (it needs room in the head between the valves when central) and mass. How many
 * plugs a cylinder has and where they sit is the head's design. Gap, heat range and electrode material are not
 * modelled (no source-backed relation to misfire or pre-ignition in this model yet).
 */
public record SparkPlugDesign(String name, double threadDiameterM, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:spark_plug";
    /** A 14 mm plug weighs about 45 g; others scale with the thread's area (an estimate). */
    private static final double REFERENCE_MASS = 0.045, REFERENCE_THREAD = 0.014;

    public SparkPlugDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "threadDiameterM", threadDiameterM);
        if (threadDiameterM > 0) { DesignIssue.range(issues, "threadDiameterM", threadDiameterM, 0.008, 0.018, "M8 to M18 plugs"); }
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    @Override public double massKg() {
        return Double.isNaN(massGivenKg) ? REFERENCE_MASS * Math.pow(threadDiameterM / REFERENCE_THREAD, 2) : massGivenKg;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("threadDiameterM", threadDiameterM, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static SparkPlugDesign read(String name, Fields f) {
        return new SparkPlugDesign(name, f.number("threadDiameterM", 0.012), EngineComponent.Rules.massOr(f));
    }
}

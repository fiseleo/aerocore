package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Ignition system (點火系統) and its control: knock control (a knock sensor that retards the spark at the knock limit;
 * without it the spark stays at best torque and the engine knocks where that is past the limit), the largest spark
 * advance the system gives, the rev limiter, the idle speed the idle control holds, and the starter's torque at the
 * crank (it turns the engine until it fires above 400 rpm).
 */
public record IgnitionDesign(String name, boolean knockControl, double maxAdvanceDeg, double revLimitRpm, double idleRpm,
                             double starterTorqueNm, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:ignition";

    public IgnitionDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(maxAdvanceDeg > 0 && maxAdvanceDeg <= 80)) { issues.add(DesignIssue.invalid("maxAdvanceDeg", "Maximum advance must be in (0, 80] degrees")); }
        EngineComponent.Rules.positive(issues, "revLimitRpm", revLimitRpm);
        EngineComponent.Rules.positive(issues, "idleRpm", idleRpm);
        EngineComponent.Rules.positive(issues, "starterTorqueNm", starterTorqueNm);
        if (idleRpm > 0 && revLimitRpm > 0 && !(revLimitRpm > idleRpm + 500)) { issues.add(DesignIssue.invalid("revLimitRpm", "The rev limit must be well above idle")); }
        if (idleRpm > 0) { DesignIssue.range(issues, "idleRpm", idleRpm, 500, 1500, "idle speeds"); }
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Coils, wiring and starter, kg: given, or 6 kg (an estimate). */
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 6 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("knockControl", knockControl, "maxAdvanceDeg", maxAdvanceDeg, "revLimitRpm", revLimitRpm, "idleRpm", idleRpm,
                "starterTorqueNm", starterTorqueNm, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static IgnitionDesign read(String name, Fields f) {
        return new IgnitionDesign(name, f.bool("knockControl", true), f.number("maxAdvanceDeg", 45), f.number("revLimitRpm"),
                f.number("idleRpm", 850), f.number("starterTorqueNm", 60), EngineComponent.Rules.massOr(f));
    }
}

package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Fuel injector (噴油嘴): static flow (cm^3 of fuel per minute, held open) at its rated pressure difference, and whether
 * it is a port or direct injector. Flow through the fixed orifice scales with the square root of the pressure
 * difference (Bernoulli); the injector can be open for at most 85 % of the cycle (a common duty-cycle limit, a rule
 * here). An engine that needs more fuel than its injectors deliver runs leaner than its target.
 */
public record FuelInjectorDesign(String name, double staticFlowCcPerMin, double ratedPressurePa, Kind kind, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:fuel_injector";
    public static final double MAX_DUTY = 0.85;

    public enum Kind { PORT, DIRECT }

    public FuelInjectorDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(kind, "kind"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "staticFlowCcPerMin", staticFlowCcPerMin);
        EngineComponent.Rules.positive(issues, "ratedPressurePa", ratedPressurePa);
        // A direct injector must inject against compression pressure.
        if (kind == Kind.DIRECT && ratedPressurePa < 3e6) { issues.add(DesignIssue.invalid("ratedPressurePa", "A direct injector needs at least 3 MPa to inject against compression")); }
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Volume flow held open at a pressure difference (Pa), m^3/s. */
    public double flowM3PerS(double pressurePa) { return staticFlowCcPerMin * 1e-6 / 60 * Math.sqrt(Math.max(0, pressurePa) / ratedPressurePa); }

    @Override public double massKg() { return Double.isNaN(massGivenKg) ? (kind == Kind.DIRECT ? 0.09 : 0.04) : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("staticFlowCcPerMin", staticFlowCcPerMin, "ratedPressurePa", ratedPressurePa, "kind", kind.name().toLowerCase(),
                "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static FuelInjectorDesign read(String name, Fields f) {
        Kind kind = EngineComponent.Rules.choice(f, "kind", Kind.class, Kind.PORT);
        return new FuelInjectorDesign(name, f.number("staticFlowCcPerMin"), f.number("ratedPressurePa", kind == Kind.DIRECT ? 15e6 : 3e5), kind,
                EngineComponent.Rules.massOr(f));
    }
}

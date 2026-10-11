package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Exhaust (排氣): the back pressure the system holds in the exhaust ports, as a factor on Patton, Nitschke and Heywood's
 * production exhaust, whose pressure drop gives 0.178 (p_i/p_a S_p)^2 kPa of pumping mean effective pressure
 * (SAE 890836, as given by Sandoval 2002, eq. 13): 1 = a 1980s production system, about 0.5 a free-flowing one, less
 * for open headers. Header wave tuning is not modelled. Mass: given, or 12 kg of pipes and silencers (an estimate).
 */
public record ExhaustDesign(String name, double backPressureFactor, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:exhaust";

    public ExhaustDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.nonnegative(issues, "backPressureFactor", backPressureFactor);
        DesignIssue.range(issues, "backPressureFactor", backPressureFactor, 0.1, 2, "open headers to restrictive systems");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Exhaust plenum pressure for an ambient pressure, manifold pressure and mean piston speed (m/s), Pa. */
    public double backPressure(double ambientPa, double manifoldPa, double meanPistonSpeed) {
        double x = manifoldPa / ambientPa * meanPistonSpeed;
        return ambientPa + backPressureFactor * 178 * x * x;
    }

    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 12 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("backPressureFactor", backPressureFactor, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static ExhaustDesign read(String name, Fields f) {
        return new ExhaustDesign(name, f.number("backPressureFactor", 1), EngineComponent.Rules.massOr(f));
    }
}

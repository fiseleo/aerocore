package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Intake (進氣): runner length and diameter per cylinder (EngineCycle's tuned runner: longer runners ram at lower
 * speeds), plenum volume (sets how fast the manifold pressure follows the throttle), throttle bore and how many throttle
 * bodies (1, or one per cylinder), and the air filter's loss coefficient (pressure drop = K 0.5 rho u^2 at the throttle
 * bore). Mass: given, or runners and plenum as 3 mm aluminium shells plus 1 kg per throttle body (estimates).
 */
public record IntakeDesign(String name, double runnerLengthM, double runnerDiameterM, double plenumVolumeM3, double throttleDiameterM,
                           int throttleBodies, double filterLoss, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:intake";

    public IntakeDesign { Objects.requireNonNull(name, "name"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.nonnegative(issues, "runnerLengthM", runnerLengthM);
        if (runnerLengthM > 0) { EngineComponent.Rules.positive(issues, "runnerDiameterM", runnerDiameterM); }
        EngineComponent.Rules.positive(issues, "plenumVolumeM3", plenumVolumeM3);
        EngineComponent.Rules.positive(issues, "throttleDiameterM", throttleDiameterM);
        EngineComponent.Rules.count(issues, "throttleBodies", throttleBodies, 1, 16);
        EngineComponent.Rules.nonnegative(issues, "filterLoss", filterLoss);
        DesignIssue.range(issues, "runnerLengthM", runnerLengthM, 0, 0.8, "intake runners (lumped model; wave tuning of very long runners not modelled)");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Total throttle area, m^2. */
    public double throttleArea() { return throttleBodies * Math.PI * throttleDiameterM * throttleDiameterM / 4; }

    /** For an engine of n cylinders, kg. */
    public double massFor(int cylinders) {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double runners = cylinders * Math.PI * runnerDiameterM * runnerLengthM * 0.003;
        double plenum = 6 * Math.cbrt(plenumVolumeM3) * Math.cbrt(plenumVolumeM3) * 0.003;
        return 2700 * (runners + plenum) + throttleBodies;
    }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("runnerLengthM", runnerLengthM, "runnerDiameterM", runnerDiameterM, "plenumVolumeM3", plenumVolumeM3,
                "throttleDiameterM", throttleDiameterM, "throttleBodies", throttleBodies, "filterLoss", filterLoss, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static IntakeDesign read(String name, Fields f) {
        return new IntakeDesign(name, f.number("runnerLengthM", 0.3), f.number("runnerDiameterM", 0.036), f.number("plenumVolumeM3", 0.003),
                f.number("throttleDiameterM", 0.06), f.integer("throttleBodies", 1), f.number("filterLoss", 1.0), EngineComponent.Rules.massOr(f));
    }
}

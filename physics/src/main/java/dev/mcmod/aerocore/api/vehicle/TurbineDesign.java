package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Turbine (渦輪) of a jet engine: stages, polytropic efficiency, tip diameter, blade cooling (effectiveness and the share
 * of compressor air it takes), mechanical efficiency of the spool's bearings, material.
 * <p>
 * Blade metal temperature with cooling effectiveness epsilon = (T_gas - T_metal) / (T_gas - T_coolant) (Saravanamuttoo
 * et al., Gas Turbine Theory, ch. 7; coolant is compressor delivery air): T_metal = T4 - epsilon (T4 - T3), which must
 * stay at the material's service temperature, so the hottest turbine inlet is (T_service - epsilon T3) / (1 - epsilon).
 * Film-cooled blades reach about 0.5-0.6 with 5-10 % of the air; the link between effectiveness and bleed is the
 * designer's (not modelled). Stage loading (work over tip speed squared) of turbines runs to about 2.5 (Smith's chart).
 * Disc stress as for compressors. Estimated mass 0.04 rho D^3 per stage (rotor, guide vanes and casing), the rotor a
 * quarter of it (rough estimates, see CompressorDesign).
 */
public record TurbineDesign(String name, int stages, double polytropicEfficiency, double tipDiameterM, double coolingEffectiveness,
                            double coolingBleed, double mechanicalEfficiency, EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:turbine";

    public TurbineDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.count(issues, "stages", stages, 1, 8);
        if (!(polytropicEfficiency > 0.5 && polytropicEfficiency < 1)) { issues.add(DesignIssue.invalid("polytropicEfficiency", "Efficiency must be between 0.5 and 1")); }
        else { DesignIssue.range(issues, "polytropicEfficiency", polytropicEfficiency, 0.8, 0.93, "turbines from model engines to modern cores"); }
        EngineComponent.Rules.positive(issues, "tipDiameterM", tipDiameterM);
        if (!(coolingEffectiveness >= 0 && coolingEffectiveness < 0.9)) { issues.add(DesignIssue.invalid("coolingEffectiveness", "Cooling effectiveness must be in [0, 0.9)")); }
        else { DesignIssue.range(issues, "coolingEffectiveness", coolingEffectiveness, 0, 0.65, "film-cooled blades"); }
        if (!(coolingBleed >= 0 && coolingBleed < 0.4)) { issues.add(DesignIssue.invalid("coolingBleed", "Cooling bleed must be in [0, 0.4)")); }
        else { DesignIssue.range(issues, "coolingBleed", coolingBleed, 0, 0.2, "turbine cooling air"); }
        if (coolingEffectiveness > 0 && coolingBleed == 0) { issues.add(DesignIssue.invalid("coolingBleed", "Cooled blades need cooling air")); }
        if (!(mechanicalEfficiency > 0.5 && mechanicalEfficiency <= 1)) { issues.add(DesignIssue.invalid("mechanicalEfficiency", "Mechanical efficiency must be in (0.5, 1]")); }
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Blade metal temperature for a turbine inlet and coolant temperature, K. */
    public double metalTemperature(double turbineInletK, double coolantK) { return turbineInletK - coolingEffectiveness * (turbineInletK - coolantK); }
    /** Hottest turbine inlet the blades take with coolant at T3, K. */
    public double maxInletTemperature(double coolantK) { return (material.serviceTemperatureK() - coolingEffectiveness * coolantK) / (1 - coolingEffectiveness); }
    public double discStress(double tipSpeed) { return (3 + 0.3) / 8 * material.densityKgPerM3() * tipSpeed * tipSpeed; }

    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0.04 * material.densityKgPerM3() * Math.pow(tipDiameterM, 3) * stages : massGivenKg; }
    public double rotorInertia() { return 0.25 * massKg() * 0.5 * Math.pow(tipDiameterM / 2, 2); }

    @Override public Map<String, Object> parameters() {
        return Fields.object("stages", stages, "polytropicEfficiency", polytropicEfficiency, "tipDiameterM", tipDiameterM,
                "coolingEffectiveness", coolingEffectiveness, "coolingBleed", coolingBleed, "mechanicalEfficiency", mechanicalEfficiency,
                "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static TurbineDesign read(String name, Fields f) {
        return new TurbineDesign(name, f.integer("stages", 1), f.number("polytropicEfficiency", 0.88), f.number("tipDiameterM"),
                f.number("coolingEffectiveness", 0), f.number("coolingBleed", 0), f.number("mechanicalEfficiency", 0.99),
                EngineComponent.Rules.material(f, "material", EngineMaterial.INCONEL_713C), EngineComponent.Rules.massOr(f));
    }
}

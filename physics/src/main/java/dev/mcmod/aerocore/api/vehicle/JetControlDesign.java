package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.Fuel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Fuel control (燃油控制) of a jet engine: fuel, the turbine inlet temperature of the design point (also the control's
 * limit), idle and maximum spool speed (fractions of design), whether it schedules acceleration fuel to stay clear of
 * surge and how much of the surge margin it keeps in reserve, the starter's torque and the speed at which the burner
 * lights.
 * <p>
 * Control rules (a schedule, not measured): throttle sets the target speed between idle and maximum; fuel follows the
 * running line's fuel at the target plus a proportional correction, limited by the turbine temperature, by
 * acceleration (the fuel at which the compressor would use more than the unreserved share of its surge margin) and
 * below by half the running line's fuel (lean blow-out); the fuel valve follows with a 0.1 s lag. With the ignition on,
 * the starter turns the spool to light-off, keeps helping and cuts out for good at 45 % speed (it comes back only after
 * the flame goes out). A spool below 10 % or an inlet under water puts the flame out.
 */
public record JetControlDesign(String name, String fuel, double turbineInletK, double idleFraction, double maxSpeedFraction,
                               boolean accelerationSchedule, double surgeReserve, double starterTorqueNm, double lightOffFraction,
                               double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:jet_control";

    public JetControlDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(fuel, "fuel"); }

    @Override public String type() { return TYPE; }

    public Fuel fuelModel() {
        return switch (fuel) {
            case "kerosene" -> Fuel.KEROSENE;
            case "gasoline" -> Fuel.gasoline(95, 85);
            case "methanol" -> Fuel.METHANOL;
            case "propane" -> Fuel.PROPANE;
            default -> throw new IllegalArgumentException("Unknown fuel " + fuel);
        };
    }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!List.of("kerosene", "gasoline", "methanol", "propane").contains(fuel)) { issues.add(DesignIssue.invalid("fuel", "Fuel must be kerosene, gasoline, methanol or propane")); }
        EngineComponent.Rules.positive(issues, "turbineInletK", turbineInletK);
        DesignIssue.range(issues, "turbineInletK", turbineInletK, 900, 1900, "turbojets from the 1950s to modern cooled turbines");
        if (!(idleFraction > 0.15 && idleFraction < 0.95)) { issues.add(DesignIssue.invalid("idleFraction", "Idle must be between 15 and 95 % speed")); }
        if (!(maxSpeedFraction > idleFraction && maxSpeedFraction <= 1.2)) { issues.add(DesignIssue.invalid("maxSpeedFraction", "Maximum speed must be above idle and at most 120 %")); }
        if (!(surgeReserve >= 0 && surgeReserve < 1)) { issues.add(DesignIssue.invalid("surgeReserve", "Surge reserve must be in [0, 1)")); }
        EngineComponent.Rules.positive(issues, "starterTorqueNm", starterTorqueNm);
        if (!(lightOffFraction > 0.03 && lightOffFraction < idleFraction)) { issues.add(DesignIssue.invalid("lightOffFraction", "Light-off must be below idle")); }
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /**
     * Accessories (fuel control and pump, starter, gearbox, oil system) for an engine's design airflow (kg/s), kg: given,
     * or 0.5 kg plus 1.2 kg per kg/s (estimate, see CompressorDesign).
     */
    public double massFor(double airflowKgPerS) { return Double.isNaN(massGivenKg) ? 0.5 + 1.2 * airflowKgPerS : massGivenKg; }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("fuel", fuel, "turbineInletK", turbineInletK, "idleFraction", idleFraction, "maxSpeedFraction", maxSpeedFraction,
                "accelerationSchedule", accelerationSchedule, "surgeReserve", surgeReserve, "starterTorqueNm", starterTorqueNm,
                "lightOffFraction", lightOffFraction, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static JetControlDesign read(String name, Fields f) {
        return new JetControlDesign(name, f.string("fuel", "kerosene"), f.number("turbineInletK"), f.number("idleFraction", 0.55),
                f.number("maxSpeedFraction", 1.02), f.bool("accelerationSchedule", true), f.number("surgeReserve", 0.3),
                f.number("starterTorqueNm"), f.number("lightOffFraction", 0.15), EngineComponent.Rules.massOr(f));
    }
}

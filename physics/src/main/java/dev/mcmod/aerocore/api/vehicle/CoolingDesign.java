package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Cooling (冷卻系統): liquid (radiator of frontal area and core depth, fan flow, coolant volume, thermostat) or air
 * (finned cylinders of a given fin area). The engine is one lumped temperature (metal, coolant, oil) heated by the
 * cycle's wall heat and all friction; a liquid system rejects heat once the thermostat opens (fully over 10 K, a wax
 * thermostat's usual band, a rule here).
 * <p>
 * Heat rejection: effectiveness-NTU with the coolant side much larger than the air side, epsilon = 1 - exp(-UA/C_air)
 * (Incropera, Fundamentals of Heat and Mass Transfer, ch. 11). Air surface = frontal area x depth x 1000 m^2/m^3 for a
 * radiator (compact automotive cores lie around 700-1500 m^2/m^3, Kays and London), or the fin area; the air-side
 * coefficient from laminar flow along the fins, Nu = 0.664 Re^0.5 Pr^(1/3) over the core depth (or a 50 mm fin), at the
 * larger of the fan's flow and 30 % of the vehicle's airspeed (the share of the free stream reaching a core behind a
 * grille, a rule).
 */
public record CoolingDesign(String name, Kind kind, double radiatorAreaM2, double radiatorDepthM, double finAreaM2, double fanFlowM3PerS,
                            double coolantVolumeM3, double thermostatC, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:cooling";
    public static final double SURFACE_DENSITY = 1000, RAM_FRACTION = 0.3, FIN_CHORD = 0.05, THERMOSTAT_BAND = 10;
    public static final double COOLANT_DENSITY = 1050, COOLANT_SPECIFIC_HEAT = 3500;   // 50 % glycol-water, typical values

    public enum Kind { LIQUID, AIR }

    public CoolingDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(kind, "kind"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (kind == Kind.LIQUID) {
            EngineComponent.Rules.positive(issues, "radiatorAreaM2", radiatorAreaM2);
            EngineComponent.Rules.positive(issues, "radiatorDepthM", radiatorDepthM);
            EngineComponent.Rules.positive(issues, "coolantVolumeM3", coolantVolumeM3);
            if (!(thermostatC > 40 && thermostatC < 120)) { issues.add(DesignIssue.invalid("thermostatC", "Thermostat must open between 40 and 120 C")); }
        } else { EngineComponent.Rules.positive(issues, "finAreaM2", finAreaM2); }
        EngineComponent.Rules.nonnegative(issues, "fanFlowM3PerS", fanFlowM3PerS);
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Wall temperature the cycle simulation uses, K: 60 K above the thermostat for liquid cooling, 470 K for air (estimates of Heywood's 400-500 K walls). */
    public double wallTemperatureK() { return kind == Kind.LIQUID ? thermostatC + 273.15 + 60 : 470; }

    /**
     * Heat rejected to air at engine temperature T (K), air temperature, density, viscosity and conductivity, and
     * vehicle airspeed (m/s), W. Below the thermostat a liquid system rejects nothing.
     */
    public double rejection(double engineK, double airK, double airDensity, double airViscosity, double airConductivity, double airspeed) {
        if (engineK <= airK) { return 0; }
        double open = kind == Kind.LIQUID ? Math.clamp((engineK - (thermostatC + 273.15)) / THERMOSTAT_BAND, 0, 1) : 1;
        if (open == 0) { return 0; }
        double frontal = kind == Kind.LIQUID ? radiatorAreaM2 : Math.max(finAreaM2 * 0.1, 1e-3);
        double length = kind == Kind.LIQUID ? radiatorDepthM : FIN_CHORD;
        double surface = kind == Kind.LIQUID ? radiatorAreaM2 * radiatorDepthM * SURFACE_DENSITY : finAreaM2;
        double speed = Math.max(fanFlowM3PerS / frontal, RAM_FRACTION * airspeed);
        if (speed <= 0) { speed = 0.3; }   // natural draught over a hot core, a floor
        double nu = airViscosity / airDensity, re = speed * length / nu, pr = 0.71;
        double h = 0.664 * airConductivity / length * Math.sqrt(re) * Math.cbrt(pr);
        double cAir = airDensity * speed * frontal * 1005;
        double effectiveness = 1 - Math.exp(-h * surface / cAir);
        return open * effectiveness * cAir * (engineK - airK);
    }

    /** Radiator, pump and coolant (or fins), kg: given, or core at 30 % aluminium plus coolant (estimates). */
    @Override public double massKg() {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        return kind == Kind.LIQUID ? radiatorAreaM2 * radiatorDepthM * 0.3 * 2700 + coolantVolumeM3 * COOLANT_DENSITY + 2 : 0;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("kind", kind.name().toLowerCase(), "radiatorAreaM2", radiatorAreaM2, "radiatorDepthM", radiatorDepthM, "finAreaM2", finAreaM2,
                "fanFlowM3PerS", fanFlowM3PerS, "coolantVolumeM3", coolantVolumeM3, "thermostatC", thermostatC, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CoolingDesign read(String name, Fields f) {
        return new CoolingDesign(name, EngineComponent.Rules.choice(f, "kind", Kind.class, Kind.LIQUID), f.number("radiatorAreaM2", 0.3),
                f.number("radiatorDepthM", 0.03), f.number("finAreaM2", 0), f.number("fanFlowM3PerS", 0.5), f.number("coolantVolumeM3", 0.006),
                f.number("thermostatC", 88), EngineComponent.Rules.massOr(f));
    }
}

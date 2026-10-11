package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.EngineOil;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Lubrication (潤滑系統): oil grade (its viscosity against temperature scales the friction model's hydrodynamic terms,
 * so a cold engine has about twice the friction of a warm one), oil volume (heat capacity, and mass), wet or dry sump.
 * The oil is taken at the engine's lumped temperature. Pump and sump type add mass only; oil starvation in hard
 * cornering (what a dry sump prevents) is not modelled.
 */
public record LubricationDesign(String name, String oilGrade, double oilVolumeM3, Sump sump, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:lubrication";
    /** Engine oil near operating temperature, kg/m^3 and J/(kg K) (typical values). */
    public static final double OIL_DENSITY = 870, OIL_SPECIFIC_HEAT = 1900;

    public enum Sump { WET, DRY }

    public LubricationDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(oilGrade, "oilGrade"); Objects.requireNonNull(sump, "sump"); }

    @Override public String type() { return TYPE; }

    public EngineOil oil() { return EngineOil.grade(oilGrade); }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        boolean known = false;
        for (EngineOil o : EngineOil.GRADES) { known |= o.grade().equalsIgnoreCase(oilGrade); }
        if (!known) { issues.add(DesignIssue.invalid("oilGrade", "Unknown oil grade " + oilGrade)); }
        EngineComponent.Rules.positive(issues, "oilVolumeM3", oilVolumeM3);
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Oil plus pump and pan (2 kg wet sump, 6 kg dry sump with its tank and scavenge pumps; estimates), kg. */
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? oilVolumeM3 * OIL_DENSITY + (sump == Sump.DRY ? 6 : 2) : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("oilGrade", oilGrade, "oilVolumeM3", oilVolumeM3, "sump", sump.name().toLowerCase(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static LubricationDesign read(String name, Fields f) {
        return new LubricationDesign(name, f.string("oilGrade", "10W-30"), f.number("oilVolumeM3", 0.004),
                EngineComponent.Rules.choice(f, "sump", Sump.class, Sump.WET), EngineComponent.Rules.massOr(f));
    }
}

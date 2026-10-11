package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;
import java.util.Map;

/**
 * How a part gets its mass. Derived: from its geometry and a material (shell parts use the wall thickness, solid
 * parts ignore it). Given: a mass the designer sets, for example to stand for fittings inside a hull; its distribution
 * is the derived one scaled to that mass, moved to centre when one is given (in the part's frame).
 */
public sealed interface MassSpec permits MassSpec.Derived, MassSpec.Given {
    record Derived(Material material, double thicknessM) implements MassSpec {}
    record Given(double massKg, Vec3d centre) implements MassSpec {}

    static MassSpec shell(Material material, double thicknessM) { return new Derived(material, thicknessM); }
    static MassSpec solid(Material material) { return new Derived(material, Double.NaN); }
    static MassSpec given(double massKg) { return new Given(massKg, null); }

    /**
     * The part's mass from its unit distribution: unitShell(t) is the mass properties at density 1 kg/m^3 for wall
     * thickness t (infinite: solid; solid-only parts ignore t). A given mass uses the thin-wall distribution (t = 1 mm).
     */
    default MassProperties resolve(java.util.function.DoubleFunction<MassProperties> unitShell) {
        if (this instanceof Derived d) {
            // No thickness: a solid part (the unit function then fills the whole shape).
            return unitShell.apply(Double.isFinite(d.thicknessM()) ? d.thicknessM() : Double.POSITIVE_INFINITY).scaled(d.material().densityKgPerM3());
        }
        Given g = (Given) this;
        MassProperties unit = unitShell.apply(1e-3);
        MassProperties scaled = unit.massKg() > 0 ? unit.scaled(g.massKg() / unit.massKg()) : MassProperties.point(g.massKg(), unit.centre());
        return g.centre() == null ? scaled : scaled.at(g.centre());
    }

    default void check(List<DesignIssue> issues, String parameter, boolean shell) {
        if (this instanceof Derived d) {
            d.material().check(issues, parameter + ".material");
            if (shell && !(d.thicknessM() > 0)) { issues.add(DesignIssue.invalid(parameter + ".thicknessM", "Wall thickness must be positive")); }
        } else {
            Given g = (Given) this;
            if (!(g.massKg() > 0) || !Double.isFinite(g.massKg())) { issues.add(DesignIssue.invalid(parameter + ".massKg", "Mass must be positive")); }
        }
    }

    default Map<String, Object> toJson() {
        if (this instanceof Derived d) {
            Map<String, Object> m = Fields.object("material", d.material().toJson());
            if (Double.isFinite(d.thicknessM())) { m.put("thicknessM", d.thicknessM()); }
            return m;
        }
        Given g = (Given) this;
        Map<String, Object> m = Fields.object("massKg", g.massKg());
        if (g.centre() != null) { m.put("centre", Fields.vector(g.centre())); }
        return m;
    }

    static MassSpec fromJson(Fields f) {
        if (f.has("massKg")) { return new Given(f.number("massKg"), f.has("centre") ? f.vec("centre") : null); }
        return new Derived(Material.fromJson(f.raw("material"), f.path() + ".material"), f.number("thicknessM", Double.NaN));
    }
}

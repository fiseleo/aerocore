package dev.mcmod.aerocore.api.vehicle;

import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A structural material for derived masses: a name and a density (kg/m^3). The constants are typical handbook values
 * (for example ASM and Engineering ToolBox tables), not one alloy or species; any density may be given instead.
 */
public record Material(String name, double densityKgPerM3) {
    public static final Material STEEL = new Material("steel", 7850);
    public static final Material STAINLESS_STEEL = new Material("stainless_steel", 8000);
    public static final Material ALUMINIUM = new Material("aluminium", 2700);
    public static final Material TITANIUM = new Material("titanium", 4430);
    /** Nickel-aluminium bronze, the usual ship-propeller alloy. */
    public static final Material BRONZE = new Material("bronze", 7600);
    /** Glass-fibre reinforced plastic laminate (1500 to 2000 by fibre content). */
    public static final Material GRP = new Material("grp", 1800);
    /** Carbon-fibre reinforced plastic laminate. */
    public static final Material CFRP = new Material("cfrp", 1550);
    /** Marine plywood, air dry. */
    public static final Material PLYWOOD = new Material("plywood", 600);
    public static final Material OAK = new Material("oak", 750);
    public static final List<Material> STANDARD = List.of(STEEL, STAINLESS_STEEL, ALUMINIUM, TITANIUM, BRONZE, GRP, CFRP, PLYWOOD, OAK);

    public Material {
        Objects.requireNonNull(name, "name");
        if (!Double.isFinite(densityKgPerM3)) { throw new IllegalArgumentException("Density must be finite"); }
    }

    void check(List<DesignIssue> issues, String parameter) {
        if (!(densityKgPerM3 > 0)) { issues.add(DesignIssue.invalid(parameter + ".density", "Material density must be positive")); }
    }

    Map<String, Object> toJson() {
        return Fields.object("name", name, "density", densityKgPerM3);
    }

    /** {"name", "density"}; a standard name alone gives its density. */
    static Material fromJson(Object value, String path) {
        if (value instanceof String s) { return standard(s, path); }
        Fields f = Fields.of(value, path);
        String name = f.string("name", "custom");
        if (!f.has("density")) { return standard(name, path); }
        return new Material(name, f.number("density"));
    }

    private static Material standard(String name, String path) {
        for (Material m : STANDARD) { if (m.name.equals(name)) { return m; } }
        throw new IllegalArgumentException(path + ": unknown material " + name + " (give a density)");
    }
}

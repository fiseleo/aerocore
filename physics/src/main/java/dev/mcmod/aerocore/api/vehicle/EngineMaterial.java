package dev.mcmod.aerocore.api.vehicle;

import java.util.List;
import java.util.Objects;

/**
 * A material for engine components: density (kg/m^3), yield strength and fatigue strength (Pa), Young's modulus (Pa),
 * thermal conductivity (W/(m K)), specific heat (J/(kg K)) and service temperature (K: about the highest metal
 * temperature for a long creep and oxidation life). The presets carry typical room-temperature values as listed for
 * these alloys in handbook tables (MatWeb, ASM; for the superalloys, producers' data sheets): fatigue strength is the
 * endurance limit for steels and titanium and the 5e8-cycle strength for aluminium; grey iron has no yield point, so
 * its tensile strength stands in. Service temperatures are rounded typical limits (aluminium alloys about 200 C,
 * Inconel 718 about 650 C, cast Inconel 713C about 980 C, single-crystal CMSX-4 about 1100 C). Strength at
 * operating temperature is lower and is not modelled. Any values may be given instead.
 */
public record EngineMaterial(String name, double densityKgPerM3, double yieldPa, double fatiguePa, double modulusPa,
                             double conductivityWPerMK, double specificHeatJPerKgK, double serviceTemperatureK) {
    public static final EngineMaterial GREY_IRON = new EngineMaterial("grey_iron", 7150, 214e6, 97e6, 110e9, 52, 460, 673);
    public static final EngineMaterial DUCTILE_IRON = new EngineMaterial("ductile_iron", 7100, 310e6, 195e6, 169e9, 36, 460, 673);
    public static final EngineMaterial ALUMINIUM_A356 = new EngineMaterial("aluminium_a356", 2680, 186e6, 90e6, 72e9, 151, 963, 473);
    public static final EngineMaterial ALUMINIUM_4032 = new EngineMaterial("aluminium_4032", 2680, 315e6, 110e6, 79e9, 154, 864, 523);
    public static final EngineMaterial ALUMINIUM_2618 = new EngineMaterial("aluminium_2618", 2760, 372e6, 124e6, 74e9, 146, 875, 523);
    public static final EngineMaterial ALUMINIUM_7075 = new EngineMaterial("aluminium_7075", 2810, 503e6, 159e6, 72e9, 130, 960, 423);
    public static final EngineMaterial STEEL_1045 = new EngineMaterial("steel_1045", 7850, 310e6, 280e6, 205e9, 50, 486, 673);
    public static final EngineMaterial STEEL_4340 = new EngineMaterial("steel_4340", 7850, 862e6, 480e6, 205e9, 44, 475, 723);
    public static final EngineMaterial TITANIUM_6AL4V = new EngineMaterial("titanium_6al4v", 4430, 880e6, 510e6, 114e9, 6.7, 526, 673);
    public static final EngineMaterial VALVE_STEEL = new EngineMaterial("valve_steel", 7750, 580e6, 350e6, 200e9, 15, 460, 1033);
    public static final EngineMaterial MAGNESIUM_AZ91 = new EngineMaterial("magnesium_az91", 1810, 150e6, 70e6, 45e9, 72, 1000, 423);
    /** Stainless steel 321, for tailpipes and casings. */
    public static final EngineMaterial STAINLESS_321 = new EngineMaterial("stainless_321", 8000, 205e6, 240e6, 193e9, 16, 500, 1088);
    /** Inconel 718, wrought nickel superalloy for discs and casings. */
    public static final EngineMaterial INCONEL_718 = new EngineMaterial("inconel_718", 8190, 1035e6, 500e6, 200e9, 11.4, 435, 923);
    /** Inconel 713C, cast nickel superalloy for turbine wheels and blades. */
    public static final EngineMaterial INCONEL_713C = new EngineMaterial("inconel_713c", 7910, 740e6, 250e6, 206e9, 10.9, 420, 1253);
    /** CMSX-4, single-crystal nickel superalloy for turbine blades. */
    public static final EngineMaterial CMSX_4 = new EngineMaterial("cmsx_4", 8700, 900e6, 400e6, 130e9, 10, 400, 1373);
    public static final List<EngineMaterial> STANDARD = List.of(GREY_IRON, DUCTILE_IRON, ALUMINIUM_A356, ALUMINIUM_4032, ALUMINIUM_2618,
            ALUMINIUM_7075, STEEL_1045, STEEL_4340, TITANIUM_6AL4V, VALVE_STEEL, MAGNESIUM_AZ91, STAINLESS_321, INCONEL_718, INCONEL_713C, CMSX_4);

    public EngineMaterial {
        Objects.requireNonNull(name, "name");
        for (double v : new double[]{densityKgPerM3, yieldPa, fatiguePa, modulusPa, conductivityWPerMK, specificHeatJPerKgK, serviceTemperatureK}) {
            if (!Double.isFinite(v)) { throw new IllegalArgumentException("Material properties must be finite"); }
        }
    }

    void check(List<DesignIssue> issues, String parameter) {
        if (!(densityKgPerM3 > 0)) { issues.add(DesignIssue.invalid(parameter + ".density", "Material density must be positive")); }
        if (!(yieldPa > 0) || !(fatiguePa > 0) || !(modulusPa > 0)) { issues.add(DesignIssue.invalid(parameter, "Material strengths and modulus must be positive")); }
        if (!(conductivityWPerMK > 0) || !(specificHeatJPerKgK > 0)) { issues.add(DesignIssue.invalid(parameter, "Material conductivity and specific heat must be positive")); }
        if (!(serviceTemperatureK > 0)) { issues.add(DesignIssue.invalid(parameter, "Material service temperature must be positive")); }
    }

    /** A standard name alone, or every property. */
    Object toJson() {
        for (EngineMaterial m : STANDARD) { if (m.equals(this)) { return name; } }
        return Fields.object("name", name, "density", densityKgPerM3, "yield", yieldPa, "fatigue", fatiguePa, "modulus", modulusPa,
                "conductivity", conductivityWPerMK, "specificHeat", specificHeatJPerKgK, "serviceTemperature", serviceTemperatureK);
    }

    static EngineMaterial fromJson(Object value, String path) {
        if (value instanceof String s) { return standard(s, path); }
        Fields f = Fields.of(value, path);
        String name = f.string("name", "custom");
        if (!f.has("density")) { return standard(name, path); }
        return new EngineMaterial(name, f.number("density"), f.number("yield"), f.number("fatigue"), f.number("modulus"),
                f.number("conductivity"), f.number("specificHeat"), f.number("serviceTemperature", 600));
    }

    public static EngineMaterial standard(String name, String path) {
        for (EngineMaterial m : STANDARD) { if (m.name.equals(name)) { return m; } }
        throw new IllegalArgumentException(path + ": unknown engine material " + name + " (give its properties)");
    }
}

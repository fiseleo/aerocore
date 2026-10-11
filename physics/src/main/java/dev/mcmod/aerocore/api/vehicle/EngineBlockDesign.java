package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Engine block and cylinders (引擎本體與汽缸): layout, cylinder count, bank angle, bore, bore spacing, deck height (crank
 * axis to deck), main bearing count, material, wall thickness and liner finish (the friction model's roughness
 * factor C_r, 1 = the 1980s production finish it was calibrated on; plateau honing is lower).
 * <p>
 * The estimated mass is a rough casting volume: cylinder barrels, water-jacket and end walls, deck plates, crankcase
 * skirt (half the deck height) and main bearing bulkheads, all at the wall thickness (bulkheads twice it), times the
 * density. It is an estimate of a plain casting, not of any production block.
 */
public record EngineBlockDesign(String name, Layout layout, int cylinders, double bankAngleDeg, double boreM, double boreSpacingM,
                                double deckHeightM, int mainBearings, EngineMaterial material, double wallThicknessM,
                                double linerRoughness, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:engine_block";

    public enum Layout { INLINE, VEE, FLAT }

    public EngineBlockDesign {
        Objects.requireNonNull(name, "name"); Objects.requireNonNull(layout, "layout"); Objects.requireNonNull(material, "material");
    }

    @Override public String type() { return TYPE; }
    public int banks() { return layout == Layout.INLINE ? 1 : 2; }
    public int cylindersPerBank() { return layout == Layout.INLINE ? cylinders : (cylinders + 1) / 2; }
    /** Bank angle in degrees: 0 for an inline engine, 180 for a flat one. */
    public double effectiveBankAngle() { return layout == Layout.INLINE ? 0 : layout == Layout.FLAT ? 180 : bankAngleDeg; }
    /** Length of a bank along the crank, m. */
    public double length() { return cylindersPerBank() * boreSpacingM + 2 * (0.02 + wallThicknessM); }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (cylinders < 1) { issues.add(DesignIssue.invalid("cylinders", "At least one cylinder")); }
        else { DesignIssue.range(issues, "cylinders", cylinders, 1, 16, "production engines; friction model fitted on V6-V10"); }
        if (layout != Layout.INLINE && cylinders % 2 != 0) { issues.add(DesignIssue.invalid("cylinders", "A vee or flat engine needs an even number of cylinders")); }
        if (layout == Layout.VEE && !(bankAngleDeg > 0 && bankAngleDeg < 180)) { issues.add(DesignIssue.invalid("bankAngleDeg", "Bank angle must be between 0 and 180 degrees")); }
        EngineComponent.Rules.positive(issues, "boreM", boreM);
        EngineComponent.Rules.positive(issues, "deckHeightM", deckHeightM);
        EngineComponent.Rules.positive(issues, "wallThicknessM", wallThicknessM);
        EngineComponent.Rules.positive(issues, "linerRoughness", linerRoughness);
        if (boreM > 0 && !(boreSpacingM > boreM)) { issues.add(DesignIssue.invalid("boreSpacingM", "Cylinders overlap: bore spacing must exceed the bore")); }
        if (boreM > 0 && boreSpacingM > boreM) { DesignIssue.range(issues, "bridgeM", boreSpacingM - boreM, 0.004, 0.05, "below about 4 mm the bores are siamesed (no coolant between them; not modelled)"); }
        if (boreM > 0) { DesignIssue.range(issues, "boreM", boreM, 0.05, 0.13, "automotive engines; Bonatesta's burn correlation is scaled from a 79 mm bore"); }
        if (cylinders >= 1) { EngineComponent.Rules.count(issues, "mainBearings", mainBearings, 2, cylindersPerBank() + 1); }
        if (wallThicknessM > 0) { DesignIssue.range(issues, "wallThicknessM", wallThicknessM, 0.003, 0.012, "cast wall thicknesses"); }
        if (linerRoughness > 0) { DesignIssue.range(issues, "linerRoughness", linerRoughness, 0.5, 1.5, "Sandoval's ring friction factors"); }
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    @Override public double massKg() {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double t = wallThicknessM, h = deckHeightM, width = boreM + 2 * t + 0.02, length = length();
        double barrels = cylinders * Math.PI * (boreM + t) * t * h;
        double jacket = banks() * (2 * length * h * t + 2 * width * h * t);
        double decks = banks() * length * width * t;
        double skirt = 2 * length * 0.5 * h * t;
        double bulkheads = mainBearings * banks() * width * 0.5 * h * 2 * t;
        return material.densityKgPerM3() * (barrels + jacket + decks + skirt + bulkheads);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("layout", layout.name().toLowerCase(), "cylinders", cylinders, "bankAngleDeg", bankAngleDeg, "boreM", boreM,
                "boreSpacingM", boreSpacingM, "deckHeightM", deckHeightM, "mainBearings", mainBearings, "material", material.toJson(),
                "wallThicknessM", wallThicknessM, "linerRoughness", linerRoughness, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static EngineBlockDesign read(String name, Fields f) {
        return new EngineBlockDesign(name, EngineComponent.Rules.choice(f, "layout", Layout.class, Layout.INLINE),
                f.integer("cylinders"), f.number("bankAngleDeg", 90), f.number("boreM"), f.number("boreSpacingM"), f.number("deckHeightM"),
                f.integer("mainBearings"), EngineComponent.Rules.material(f, "material", EngineMaterial.GREY_IRON),
                f.number("wallThicknessM", 0.006), f.number("linerRoughness", 1), EngineComponent.Rules.massOr(f));
    }
}

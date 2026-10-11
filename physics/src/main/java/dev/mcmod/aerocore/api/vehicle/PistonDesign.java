package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Piston (活塞), one of a set: diameter (the bore less its running clearance), compression height (pin axis to crown),
 * crown volume (a dome is positive and takes volume from the chamber, a dish negative), crown thickness, skirt length,
 * pin diameter, ring count and ring tension relative to the friction model's 1980s reference (F_t/F_to), material.
 * <p>
 * Estimated mass of one piston with its pin and rings: crown disc, ring belt (wall 4.5 % of the diameter), half-round
 * skirt (wall 2.5 %), two pin bosses (1.6 pin diameters across, 22 % of the diameter long), dome volume; a hollow steel
 * pin (bore 0.6 of its diameter, 85 % of the piston diameter long) and steel rings 4 % x 1.4 % of the diameter in
 * section. Crown strength: a simply supported circular plate under uniform pressure has its largest stress at the
 * centre, 3 (3 + nu) p a^2 / (8 t^2) (Roark's Formulas for Stress and Strain, table 11.2 case 10a, nu = 0.33), so the
 * crown's fatigue strength allows p = 8 t^2 sigma_f / (3 (3 + nu) a^2); the ring belt really clamps the crown, so this
 * is conservative.
 */
public record PistonDesign(String name, double diameterM, double compressionHeightM, double crownVolumeM3, double crownThicknessM,
                           double skirtLengthM, double pinDiameterM, int rings, double ringTensionRatio, EngineMaterial material,
                           double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:piston";
    private static final double STEEL = 7850, POISSON = 0.33;

    public PistonDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "diameterM", diameterM);
        EngineComponent.Rules.positive(issues, "compressionHeightM", compressionHeightM);
        EngineComponent.Rules.positive(issues, "crownThicknessM", crownThicknessM);
        EngineComponent.Rules.positive(issues, "skirtLengthM", skirtLengthM);
        EngineComponent.Rules.positive(issues, "pinDiameterM", pinDiameterM);
        EngineComponent.Rules.positive(issues, "ringTensionRatio", ringTensionRatio);
        if (!Double.isFinite(crownVolumeM3)) { issues.add(DesignIssue.invalid("crownVolumeM3", "Crown volume must be finite")); }
        EngineComponent.Rules.count(issues, "rings", rings, 1, 6);
        if (diameterM > 0 && pinDiameterM >= 0.6 * diameterM) { issues.add(DesignIssue.invalid("pinDiameterM", "The pin must be well under the piston diameter")); }
        if (diameterM > 0 && crownThicknessM >= compressionHeightM) { issues.add(DesignIssue.invalid("crownThicknessM", "The crown must be thinner than the compression height")); }
        if (ringTensionRatio > 0) { DesignIssue.range(issues, "ringTensionRatio", ringTensionRatio, 0.4, 1.5, "friction model's ring term"); }
        DesignIssue.range(issues, "rings", rings, 2, 4, "usual ring packs");
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Mass of the pin alone, kg. */
    public double pinMassKg() { return Math.PI / 4 * (1 - 0.36) * pinDiameterM * pinDiameterM * 0.85 * diameterM * STEEL; }
    /** Mass of the ring set, kg. */
    public double ringMassKg() { return rings * Math.PI * diameterM * 0.04 * diameterM * 0.014 * diameterM * STEEL; }

    /** One piston with pin and rings, kg. */
    @Override public double massKg() {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double d = diameterM, area = Math.PI * d * d / 4;
        double crown = area * crownThicknessM + Math.max(0, crownVolumeM3);
        double belt = Math.PI * d * Math.max(0, compressionHeightM - crownThicknessM) * 0.045 * d;
        double skirt = 0.5 * Math.PI * d * skirtLengthM * 0.025 * d;
        double bosses = 2 * Math.PI / 4 * (2.56 - 1) * pinDiameterM * pinDiameterM * 0.22 * d;
        return material.densityKgPerM3() * (crown + belt + skirt + bosses) + pinMassKg() + ringMassKg();
    }

    /** Peak cylinder pressure the crown carries at its fatigue strength, Pa (simply supported plate). */
    public double crownPressureLimit() {
        double a = diameterM / 2;
        return 8 * crownThicknessM * crownThicknessM * material.fatiguePa() / (3 * (3 + POISSON) * a * a);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("diameterM", diameterM, "compressionHeightM", compressionHeightM, "crownVolumeM3", crownVolumeM3,
                "crownThicknessM", crownThicknessM, "skirtLengthM", skirtLengthM, "pinDiameterM", pinDiameterM, "rings", rings,
                "ringTensionRatio", ringTensionRatio, "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static PistonDesign read(String name, Fields f) {
        return new PistonDesign(name, f.number("diameterM"), f.number("compressionHeightM"), f.number("crownVolumeM3", 0),
                f.number("crownThicknessM"), f.number("skirtLengthM"), f.number("pinDiameterM"), f.integer("rings", 3),
                f.number("ringTensionRatio", 1), EngineComponent.Rules.material(f, "material", EngineMaterial.ALUMINIUM_4032), EngineComponent.Rules.massOr(f));
    }
}

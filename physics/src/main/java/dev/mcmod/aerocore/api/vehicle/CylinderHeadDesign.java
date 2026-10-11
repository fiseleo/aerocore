package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Cylinder head (汽缸蓋): camshaft layout, valves per cylinder, combustion chamber (shape and volume), included angle
 * between the intake and exhaust valves (0 for parallel valves), port throats as a fraction of the valve head
 * diameter, port discharge coefficient (porting quality), spark plugs per cylinder and their offset from the bore
 * axis, head gasket thickness, material. Inclined valves need less room across the bore (their heads project as
 * ellipses), which is how pent-roof heads fit large valves.
 * <p>
 * Chamber surface (for heat transfer): a DOME is taken as the spherical cap of the chamber's volume over the bore
 * (pent-roof, hemispherical and wedge chambers); a FLAT chamber as a cylinder of the bore's diameter (bathtub, heron).
 * The discharge coefficient multiplies the smaller of curtain and throat areas (ValveFlow); 0.55-0.85 is the range of
 * production and racing ports this model treats as normal, a design choice rather than a measured range.
 * Estimated mass: the head's footprint over the block times its height (90 mm OHV, 110 mm SOHC, 130 mm DOHC) times
 * a solid fraction of 0.35, an estimate of a plain casting.
 */
public record CylinderHeadDesign(String name, Layout layout, int intakeValves, int exhaustValves, Chamber chamber, double chamberVolumeM3,
                                 double valveAngleDeg, double intakeThroatRatio, double exhaustThroatRatio, double portFlowCoefficient, int sparkPlugs,
                                 double plugOffsetM, double gasketThicknessM, EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:cylinder_head";

    public enum Layout { OHV, SOHC, DOHC }
    public enum Chamber { DOME, FLAT }

    public CylinderHeadDesign {
        Objects.requireNonNull(name, "name"); Objects.requireNonNull(layout, "layout"); Objects.requireNonNull(chamber, "chamber");
        Objects.requireNonNull(material, "material");
    }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.count(issues, "intakeValves", intakeValves, 1, 3);
        EngineComponent.Rules.count(issues, "exhaustValves", exhaustValves, 1, 3);
        EngineComponent.Rules.count(issues, "sparkPlugs", sparkPlugs, 1, 2);
        EngineComponent.Rules.positive(issues, "chamberVolumeM3", chamberVolumeM3);
        if (!(valveAngleDeg >= 0 && valveAngleDeg < 120)) { issues.add(DesignIssue.invalid("valveAngleDeg", "Included valve angle must be 0 to 120 degrees")); }
        else { DesignIssue.range(issues, "valveAngleDeg", valveAngleDeg, 0, 60, "production heads"); }
        EngineComponent.Rules.positive(issues, "gasketThicknessM", gasketThicknessM);
        EngineComponent.Rules.nonnegative(issues, "plugOffsetM", plugOffsetM);
        for (String key : new String[]{"intakeThroatRatio", "exhaustThroatRatio"}) {
            double r = key.startsWith("in") ? intakeThroatRatio : exhaustThroatRatio;
            if (!(r > 0.4 && r < 1)) { issues.add(DesignIssue.invalid(key, "The port throat must be 40-100 % of the valve head diameter")); }
            else { DesignIssue.range(issues, key, r, 0.8, 0.92, "usual throat-to-valve ratios"); }
        }
        if (!(portFlowCoefficient > 0 && portFlowCoefficient <= 1)) { issues.add(DesignIssue.invalid("portFlowCoefficient", "Discharge coefficient must be in (0, 1]")); }
        else { DesignIssue.range(issues, "portFlowCoefficient", portFlowCoefficient, 0.55, 0.85, "normal porting (a model choice)"); }
        if (gasketThicknessM > 0) { DesignIssue.range(issues, "gasketThicknessM", gasketThicknessM, 0.0003, 0.002, "head gaskets"); }
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Height of the head casting used for the mass estimate, m. */
    public double castingHeight() { return layout == Layout.OHV ? 0.09 : layout == Layout.SOHC ? 0.11 : 0.13; }

    /** Mass for a block (its footprint), kg. */
    public double massFor(EngineBlockDesign block) {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double width = block.boreM() + 2 * block.wallThicknessM() + 0.02;
        return material.densityKgPerM3() * block.banks() * block.length() * width * castingHeight() * 0.35;
    }
    /** Without its block a head has no footprint: the given mass, or 0. */
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    /** Chamber surface over the bore area (heat transfer), for a bore. */
    public double chamberAreaFactor(double boreM) {
        double a = boreM / 2, area = Math.PI * a * a;
        if (chamber == Chamber.FLAT) { return 1 + 2 * (chamberVolumeM3 / area) / a; }
        // spherical cap of volume V over a circle of radius a: V = pi h (3 a^2 + h^2) / 6
        double lo = 0, hi = 4 * a;
        for (int i = 0; i < 80; i++) {
            double h = 0.5 * (lo + hi);
            if (Math.PI * h * (3 * a * a + h * h) / 6 < chamberVolumeM3) { lo = h; } else { hi = h; }
        }
        double h = 0.5 * (lo + hi);
        return (a * a + h * h) / (a * a);
    }

    /** Longest distance from the nearest plug to the bore's edge (in the plane of the deck), m: the flame's travel. */
    public double flameTravel(double boreM) {
        double a = boreM / 2, worst = 0;
        for (int i = 0; i < 360; i++) {
            double phi = Math.toRadians(i + 0.5), x = a * Math.cos(phi), y = a * Math.sin(phi), nearest = Double.MAX_VALUE;
            for (int k = 0; k < sparkPlugs; k++) {
                // one plug sits plugOffset towards the exhaust side; two sit plugOffset either way along the cam axis
                double px = sparkPlugs == 1 ? plugOffsetM : 0, py = sparkPlugs == 1 ? 0 : (k == 0 ? plugOffsetM : -plugOffsetM);
                nearest = Math.min(nearest, Math.hypot(x - px, y - py));
            }
            worst = Math.max(worst, nearest);
        }
        return worst;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("layout", layout.name().toLowerCase(), "intakeValves", intakeValves, "exhaustValves", exhaustValves,
                "chamber", chamber.name().toLowerCase(), "chamberVolumeM3", chamberVolumeM3, "valveAngleDeg", valveAngleDeg, "intakeThroatRatio", intakeThroatRatio,
                "exhaustThroatRatio", exhaustThroatRatio, "portFlowCoefficient", portFlowCoefficient, "sparkPlugs", sparkPlugs,
                "plugOffsetM", plugOffsetM, "gasketThicknessM", gasketThicknessM, "material", material.toJson(),
                "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CylinderHeadDesign read(String name, Fields f) {
        Chamber chamber = EngineComponent.Rules.choice(f, "chamber", Chamber.class, Chamber.DOME);
        return new CylinderHeadDesign(name, EngineComponent.Rules.choice(f, "layout", Layout.class, Layout.DOHC),
                f.integer("intakeValves", 2), f.integer("exhaustValves", 2), chamber,
                f.number("chamberVolumeM3"), f.number("valveAngleDeg", chamber == Chamber.DOME ? 40 : 0), f.number("intakeThroatRatio", 0.87), f.number("exhaustThroatRatio", 0.87),
                f.number("portFlowCoefficient", 0.7), f.integer("sparkPlugs", 1), f.number("plugOffsetM", 0), f.number("gasketThicknessM", 0.0008),
                EngineComponent.Rules.material(f, "material", EngineMaterial.ALUMINIUM_A356), EngineComponent.Rules.massOr(f));
    }
}

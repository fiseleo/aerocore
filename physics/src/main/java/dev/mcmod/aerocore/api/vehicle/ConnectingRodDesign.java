package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Connecting rod (連桿), one of a set: centre-to-centre length, big-end bore and width (it must fit the crankpin), small-end
 * bore (it must fit the piston pin), beam cross-section area, material.
 * <p>
 * Estimated mass of one rod: beam (area times the length between the eyes), big-end ring with cap and bolts (outer
 * diameter 1.45 bores) and small-end eye (outer diameter 1.6 bores, 80 % of the big-end width). Its centre of mass
 * from those pieces splits it into the usual two-mass equivalent: the fraction xG/L at the small end reciprocates
 * with the piston, the rest rotates with the crankpin (an approximation that ignores the rod's own moment of inertia).
 * Strength: the beam carries the piston and small-end inertia force at TDC of the gas exchange, m r w^2 (1 + r/l),
 * at its fatigue strength, and the peak gas force at its yield strength (direct compression; buckling of an
 * unspecified section is not checked).
 */
public record ConnectingRodDesign(String name, double lengthM, double bigEndBoreM, double bigEndWidthM, double smallEndBoreM,
                                  double beamAreaM2, EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:connecting_rod";

    public ConnectingRodDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "lengthM", lengthM);
        EngineComponent.Rules.positive(issues, "bigEndBoreM", bigEndBoreM);
        EngineComponent.Rules.positive(issues, "bigEndWidthM", bigEndWidthM);
        EngineComponent.Rules.positive(issues, "smallEndBoreM", smallEndBoreM);
        EngineComponent.Rules.positive(issues, "beamAreaM2", beamAreaM2);
        if (lengthM > 0 && 0.725 * bigEndBoreM + 0.8 * smallEndBoreM >= lengthM) { issues.add(DesignIssue.invalid("lengthM", "The rod's eyes overlap: it is too short for its bores")); }
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    private double beamLength() { return Math.max(0, lengthM - 0.725 * bigEndBoreM - 0.8 * smallEndBoreM); }
    private double bigEndVolume() { return Math.PI / 4 * (1.45 * 1.45 - 1) * bigEndBoreM * bigEndBoreM * bigEndWidthM; }
    private double smallEndVolume() { return Math.PI / 4 * (1.6 * 1.6 - 1) * smallEndBoreM * smallEndBoreM * 0.8 * bigEndWidthM; }

    @Override public double massKg() {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        return material.densityKgPerM3() * (beamAreaM2 * beamLength() + bigEndVolume() + smallEndVolume());
    }

    /** Fraction of the rod's mass that reciprocates with the piston (xG / L). */
    public double reciprocatingFraction() {
        double beam = beamAreaM2 * beamLength(), big = bigEndVolume(), small = smallEndVolume(), total = beam + big + small;
        double beamCentre = 0.725 * bigEndBoreM + beamLength() / 2;
        return total > 0 ? (beam * beamCentre + small * lengthM) / (total * lengthM) : 1.0 / 3;
    }

    /** Highest speed (rad/s) at which the beam carries a reciprocating mass (kg) at a crank radius (m) at its fatigue strength. */
    public double fatigueSpeedLimit(double reciprocatingKg, double crankRadiusM) {
        double force = material.fatiguePa() * beamAreaM2;
        return Math.sqrt(force / (reciprocatingKg * crankRadiusM * (1 + crankRadiusM / lengthM)));
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("lengthM", lengthM, "bigEndBoreM", bigEndBoreM, "bigEndWidthM", bigEndWidthM, "smallEndBoreM", smallEndBoreM,
                "beamAreaM2", beamAreaM2, "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static ConnectingRodDesign read(String name, Fields f) {
        return new ConnectingRodDesign(name, f.number("lengthM"), f.number("bigEndBoreM"), f.number("bigEndWidthM"), f.number("smallEndBoreM"),
                f.number("beamAreaM2"), EngineComponent.Rules.material(f, "material", EngineMaterial.STEEL_4340), EngineComponent.Rules.massOr(f));
    }
}

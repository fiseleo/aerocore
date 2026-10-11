package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.Gas;
import dev.mcmod.aerocore.api.engine.Turbojet;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Compressor (壓縮機) of a jet engine: axial or centrifugal, stages, design pressure ratio and airflow (sea-level static),
 * polytropic efficiency, surge margin at the design point, tip diameter, hub-to-tip ratio at the face, design speed,
 * impeller blades (centrifugal), material.
 * <p>
 * What the parameters must satisfy (from the turbomachinery, not balance):
 * <ul>
 * <li>Work per stage: an axial stage's work over the square of its tip speed (stage loading) is about 0.2-0.45 in
 *   practice (Dixon and Hall, Fluid Mechanics and Thermodynamics of Turbomachinery); beyond 1 no blading turns the
 *   flow that far. A centrifugal impeller with radial blades and no inlet swirl gives at most sigma U^2, with Stanitz's
 *   slip factor sigma = 1 - 0.63 pi / Z.</li>
 * <li>Flow through the face (annulus, or a centrifugal impeller's eye, taken as 0.6 of its tip diameter): it chokes at
 *   Mach 1, about 241 kg/(s m^2) of corrected flow; faces are designed for Mach 0.5-0.7.</li>
 * <li>A rotating solid disc carries (3 + nu) / 8 rho U^2 at its centre (Roark, rotating discs, nu = 0.3); blades add
 *   rim load, so up to two thirds of the yield strength is treated as normal.</li>
 * </ul>
 * Estimated mass: 0.03 rho D^3 per axial stage, 0.08 rho D^3 per centrifugal stage (rotor, stators and casing; rough
 * estimates, set with the turbine's and the accessories' so that a J85-GE-17A-like and a JetCat P80-like engine weigh
 * within 25 % of their 181 kg and 1.32 kg); the rotor is taken as 20 % of it, a disc of half the tip radius's inertia.
 */
public record CompressorDesign(String name, Kind kind, int stages, double pressureRatio, double airflowKgPerS, double polytropicEfficiency,
                               double surgeMargin, double tipDiameterM, double hubTipRatio, double designRpm, int blades,
                               EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:compressor";
    public static final double CHOKED_FLUX = 241, FACE_MACH_07_FLUX = 219, EYE_RATIO = 0.6;

    public enum Kind { AXIAL, CENTRIFUGAL }

    public CompressorDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(kind, "kind"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    public double tipSpeed() { return Math.PI * tipDiameterM * designRpm / 60; }
    /** Work per kilogram at the design point (sea-level static), J/kg. */
    public double designWork() {
        Gas air = Gas.AIR;
        double t2 = Turbojet.T_REF;
        double t3 = air.temperatureAtEntropy(air.entropyFunction(t2) + air.gasConstant() * Math.log(pressureRatio) / polytropicEfficiency);
        return air.enthalpy(t3) - air.enthalpy(t2);
    }
    /** Flow area of the face, m^2. */
    public double faceArea() {
        double d = kind == Kind.CENTRIFUGAL ? EYE_RATIO * tipDiameterM : tipDiameterM;
        return Math.PI / 4 * d * d * (1 - hubTipRatio * hubTipRatio);
    }
    public double slipFactor() { return 1 - 0.63 * Math.PI / blades; }
    public double discStress() { return (3 + 0.3) / 8 * material.densityKgPerM3() * tipSpeed() * tipSpeed(); }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.count(issues, "stages", stages, 1, kind == Kind.AXIAL ? 30 : 3);
        if (!(pressureRatio > 1)) { issues.add(DesignIssue.invalid("pressureRatio", "A compressor must raise the pressure")); }
        EngineComponent.Rules.positive(issues, "airflowKgPerS", airflowKgPerS);
        if (!(polytropicEfficiency > 0.5 && polytropicEfficiency < 1)) { issues.add(DesignIssue.invalid("polytropicEfficiency", "Efficiency must be between 0.5 and 1")); }
        else { DesignIssue.range(issues, "polytropicEfficiency", polytropicEfficiency, 0.75, 0.92, "compressors from model engines to modern core compressors"); }
        if (!(surgeMargin > 0 && surgeMargin < 1)) { issues.add(DesignIssue.invalid("surgeMargin", "Surge margin must be between 0 and 1")); }
        else { DesignIssue.range(issues, "surgeMargin", surgeMargin, 0.1, 0.3, "usual design surge margins"); }
        EngineComponent.Rules.positive(issues, "tipDiameterM", tipDiameterM);
        EngineComponent.Rules.positive(issues, "designRpm", designRpm);
        if (!(hubTipRatio >= 0 && hubTipRatio < 0.9)) { issues.add(DesignIssue.invalid("hubTipRatio", "Hub-to-tip ratio must be in [0, 0.9)")); }
        if (kind == Kind.CENTRIFUGAL) { EngineComponent.Rules.count(issues, "blades", blades, 3, 60); }
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        if (issues.stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID)) { return issues; }
        double u = tipSpeed(), w = designWork();
        if (kind == Kind.AXIAL) {
            double loading = w / (stages * u * u);
            if (loading > 1) { issues.add(DesignIssue.invalid("stages", "Too few stages or too slow a rotor for this pressure ratio: stage loading " + loading)); }
            else { DesignIssue.range(issues, "stageLoading", loading, 0, 0.45, "axial stage loadings (work / tip speed^2), Dixon and Hall"); }
        } else {
            double limit = stages * slipFactor() * u * u;
            if (w > limit) { issues.add(DesignIssue.invalid("designRpm", "The impeller cannot do this work at this tip speed (slip-limited work " + limit + " J/kg)")); }
            else { DesignIssue.range(issues, "impellerWorkFraction", w / limit, 0, 0.9, "work against the slip-limited maximum"); }
        }
        double flux = airflowKgPerS / faceArea();
        if (flux > CHOKED_FLUX) { issues.add(DesignIssue.invalid("tipDiameterM", "The face is too small: it chokes below the design airflow")); }
        else { DesignIssue.range(issues, "faceFlux", flux, 0, FACE_MACH_07_FLUX, "faces designed for Mach 0.5-0.7"); }
        double stress = discStress();
        if (stress > material.yieldPa()) { issues.add(DesignIssue.invalid("designRpm", "The rotor would yield at this tip speed")); }
        else { DesignIssue.range(issues, "discStressOverYield", stress / material.yieldPa(), 0, 2.0 / 3, "rotors kept under two thirds of yield"); }
        return issues;
    }

    @Override public double massKg() {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double d3 = tipDiameterM * tipDiameterM * tipDiameterM;
        return (kind == Kind.AXIAL ? 0.03 : 0.08) * material.densityKgPerM3() * d3 * stages;
    }
    /** Moment of inertia of the rotor, kg m^2 (estimate). */
    public double rotorInertia() { return 0.2 * massKg() * 0.5 * Math.pow(tipDiameterM / 2, 2); }

    @Override public Map<String, Object> parameters() {
        return Fields.object("kind", kind.name().toLowerCase(), "stages", stages, "pressureRatio", pressureRatio, "airflowKgPerS", airflowKgPerS,
                "polytropicEfficiency", polytropicEfficiency, "surgeMargin", surgeMargin, "tipDiameterM", tipDiameterM, "hubTipRatio", hubTipRatio,
                "designRpm", designRpm, "blades", blades, "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CompressorDesign read(String name, Fields f) {
        return new CompressorDesign(name, EngineComponent.Rules.choice(f, "kind", Kind.class, Kind.AXIAL), f.integer("stages"), f.number("pressureRatio"),
                f.number("airflowKgPerS"), f.number("polytropicEfficiency", 0.87), f.number("surgeMargin", 0.2), f.number("tipDiameterM"),
                f.number("hubTipRatio", 0.45), f.number("designRpm"), f.integer("blades", 16),
                EngineComponent.Rules.material(f, "material", EngineMaterial.TITANIUM_6AL4V), EngineComponent.Rules.massOr(f));
    }
}

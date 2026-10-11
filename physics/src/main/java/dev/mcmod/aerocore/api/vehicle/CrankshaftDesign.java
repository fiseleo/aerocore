package dev.mcmod.aerocore.api.vehicle;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Crankshaft (曲軸): stroke, main journal and crankpin diameters and widths, counterweighting (counterweight mass
 * moment over that of the pins and webs it balances), flywheel moment of inertia, material.
 * <p>
 * Estimated for the block it goes in (mains = the block's main bearings; crankpins one per cylinder, shared by the
 * two rods of a vee pair): main journals and pins as solid cylinders, two webs per pin (thickness 40 % of the pin
 * width, as wide as the main journal, reaching from the axis past the pin), counterweights at the crank radius
 * opposite; the flywheel a steel disc of 0.14 m radius with the given inertia. Rotating inertia sums the pieces
 * about the crank axis. An estimate of a plain forged or billet crank.
 */
public record CrankshaftDesign(String name, double strokeM, double mainDiameterM, double mainWidthM, double pinDiameterM, double pinWidthM,
                               double counterweightRatio, double flywheelInertiaKgM2, EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:crankshaft";
    private static final double FLYWHEEL_RADIUS = 0.14;

    public CrankshaftDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "strokeM", strokeM);
        EngineComponent.Rules.positive(issues, "mainDiameterM", mainDiameterM);
        EngineComponent.Rules.positive(issues, "mainWidthM", mainWidthM);
        EngineComponent.Rules.positive(issues, "pinDiameterM", pinDiameterM);
        EngineComponent.Rules.positive(issues, "pinWidthM", pinWidthM);
        EngineComponent.Rules.nonnegative(issues, "counterweightRatio", counterweightRatio);
        EngineComponent.Rules.nonnegative(issues, "flywheelInertiaKgM2", flywheelInertiaKgM2);
        if (strokeM > 0) { DesignIssue.range(issues, "strokeM", strokeM, 0.04, 0.13, "automotive engines"); }
        DesignIssue.range(issues, "counterweightRatio", counterweightRatio, 0.3, 1.2, "usual counterweighting");
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    public double crankRadius() { return strokeM / 2; }
    /** Crankpins for a block: one per cylinder, or one per vee pair. */
    public static int pins(EngineBlockDesign block) { return block.layout() == EngineBlockDesign.Layout.VEE ? block.cylinders() / 2 : block.cylinders(); }

    private double webThickness() { return 0.4 * pinWidthM; }
    private double webVolume() { return webThickness() * mainDiameterM * (crankRadius() + (mainDiameterM + pinDiameterM) / 2); }

    /** Mass of the crank (with counterweights and flywheel) for a block, kg. */
    public double massFor(EngineBlockDesign block) {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        int pins = pins(block);
        double rho = material.densityKgPerM3();
        double mains = block.mainBearings() * Math.PI / 4 * mainDiameterM * mainDiameterM * mainWidthM * rho;
        double pin = Math.PI / 4 * pinDiameterM * pinDiameterM * pinWidthM * rho, web = webVolume() * rho;
        double eccentric = pins * (pin + 2 * web * 0.5);   // webs' centroid near half the crank radius
        double counterweights = counterweightRatio * eccentric;
        return mains + pins * (pin + 2 * web) + counterweights + flywheelMassKg();
    }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }
    public double flywheelMassKg() { return 2 * flywheelInertiaKgM2 / (FLYWHEEL_RADIUS * FLYWHEEL_RADIUS); }

    /** Moment of inertia of the crank with counterweights and flywheel about its axis, kg m^2 (rods and pistons not included). */
    public double rotatingInertia(EngineBlockDesign block) {
        int pins = pins(block);
        double rho = material.densityKgPerM3(), r = crankRadius();
        double mainMass = Math.PI / 4 * mainDiameterM * mainDiameterM * mainWidthM * rho;
        double pinMass = Math.PI / 4 * pinDiameterM * pinDiameterM * pinWidthM * rho, webMass = webVolume() * rho;
        double webReach = r + (mainDiameterM + pinDiameterM) / 2;
        double mains = block.mainBearings() * 0.5 * mainMass * mainDiameterM * mainDiameterM / 4;
        double pinsI = pins * (pinMass * r * r + 0.5 * pinMass * pinDiameterM * pinDiameterM / 4);
        double webs = pins * 2 * webMass * (webReach * webReach / 3 + mainDiameterM * mainDiameterM / 12);   // plate about its end
        double counterweights = counterweightRatio * pins * (pinMass + webMass) * r * r;
        return mains + pinsI + webs + counterweights + flywheelInertiaKgM2;
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("strokeM", strokeM, "mainDiameterM", mainDiameterM, "mainWidthM", mainWidthM, "pinDiameterM", pinDiameterM,
                "pinWidthM", pinWidthM, "counterweightRatio", counterweightRatio, "flywheelInertiaKgM2", flywheelInertiaKgM2,
                "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CrankshaftDesign read(String name, Fields f) {
        return new CrankshaftDesign(name, f.number("strokeM"), f.number("mainDiameterM"), f.number("mainWidthM"), f.number("pinDiameterM"),
                f.number("pinWidthM"), f.number("counterweightRatio", 0.7), f.number("flywheelInertiaKgM2", 0.12),
                EngineComponent.Rules.material(f, "material", EngineMaterial.STEEL_4340), EngineComponent.Rules.massOr(f));
    }
}

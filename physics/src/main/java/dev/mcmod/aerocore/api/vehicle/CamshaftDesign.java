package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.ValveLift;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Camshaft (凸輪軸) lobes, as they move the valves: intake and exhaust duration (crank degrees, seat to seat), lift at the
 * valve, centrelines (intake: degrees after gas-exchange TDC; exhaust: degrees before it), lift-curve fullness (the
 * exponent of ValveLift; 2 peaky, 3 typical, higher squarer and harder on the springs), bearings per camshaft and
 * material. Estimated mass of one camshaft: a 25 mm solid shaft the length of the block plus a lobe of 30 % of the
 * shaft volume per valve it drives.
 */
public record CamshaftDesign(String name, double intakeDurationDeg, double intakeLiftM, double intakeCentrelineDeg,
                             double exhaustDurationDeg, double exhaustLiftM, double exhaustCentrelineDeg, double fullness,
                             int bearingsPerShaft, EngineMaterial material, double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:camshaft";

    public CamshaftDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(material, "material"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        for (String k : new String[]{"intakeDurationDeg", "exhaustDurationDeg"}) {
            double d = k.startsWith("in") ? intakeDurationDeg : exhaustDurationDeg;
            if (!(d > 60 && d < 400)) { issues.add(DesignIssue.invalid(k, "Duration must be between 60 and 400 crank degrees")); }
            else { DesignIssue.range(issues, k, d, 200, 320, "road and race cams (seat to seat)"); }
        }
        EngineComponent.Rules.positive(issues, "intakeLiftM", intakeLiftM);
        EngineComponent.Rules.positive(issues, "exhaustLiftM", exhaustLiftM);
        if (!(fullness >= 2) || !Double.isFinite(fullness)) { issues.add(DesignIssue.invalid("fullness", "Fullness must be at least 2")); }
        else { DesignIssue.range(issues, "fullness", fullness, 2, 5, "cam profiles a follower can track"); }
        if (!Double.isFinite(intakeCentrelineDeg) || !Double.isFinite(exhaustCentrelineDeg)) { issues.add(DesignIssue.invalid("centreline", "Centrelines must be finite")); }
        else {
            DesignIssue.range(issues, "intakeCentrelineDeg", intakeCentrelineDeg, 90, 130, "usual intake centrelines");
            DesignIssue.range(issues, "exhaustCentrelineDeg", exhaustCentrelineDeg, 90, 130, "usual exhaust centrelines");
        }
        EngineComponent.Rules.count(issues, "bearingsPerShaft", bearingsPerShaft, 2, 12);
        material.check(issues, "material");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** Intake lift law with the cam phased later by phaseDeg (crank degrees), in EngineCycle's frame (firing TDC = 0). */
    public ValveLift intakeLift(double phaseDeg) {
        return new ValveLift(Math.toRadians(-360 + intakeCentrelineDeg + phaseDeg), Math.toRadians(intakeDurationDeg), intakeLiftM, fullness);
    }
    /** Exhaust lift law with the cam phased later by phaseDeg. */
    public ValveLift exhaustLift(double phaseDeg) {
        return new ValveLift(Math.toRadians(360 - exhaustCentrelineDeg + phaseDeg), Math.toRadians(exhaustDurationDeg), exhaustLiftM, fullness);
    }
    /** Valve overlap at the seats, crank degrees (negative when the events are apart). */
    public double overlapDeg() { return intakeDurationDeg / 2 - intakeCentrelineDeg + exhaustDurationDeg / 2 - exhaustCentrelineDeg; }

    /** Mass of one camshaft for a block and the number of valves it drives per cylinder, kg. */
    public double massFor(EngineBlockDesign block, int valvesPerCylinder) {
        if (!Double.isNaN(massGivenKg)) { return massGivenKg; }
        double shaft = Math.PI / 4 * 0.025 * 0.025 * block.length();
        return material.densityKgPerM3() * shaft * (1 + 0.3 * valvesPerCylinder * block.cylindersPerBank() / Math.max(1, block.cylindersPerBank()));
    }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("intakeDurationDeg", intakeDurationDeg, "intakeLiftM", intakeLiftM, "intakeCentrelineDeg", intakeCentrelineDeg,
                "exhaustDurationDeg", exhaustDurationDeg, "exhaustLiftM", exhaustLiftM, "exhaustCentrelineDeg", exhaustCentrelineDeg,
                "fullness", fullness, "bearingsPerShaft", bearingsPerShaft, "material", material.toJson(), "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static CamshaftDesign read(String name, Fields f) {
        return new CamshaftDesign(name, f.number("intakeDurationDeg"), f.number("intakeLiftM"), f.number("intakeCentrelineDeg", 110),
                f.number("exhaustDurationDeg"), f.number("exhaustLiftM"), f.number("exhaustCentrelineDeg", 110), f.number("fullness", 3),
                f.integer("bearingsPerShaft", 5), EngineComponent.Rules.material(f, "material", EngineMaterial.DUCTILE_IRON), EngineComponent.Rules.massOr(f));
    }
}

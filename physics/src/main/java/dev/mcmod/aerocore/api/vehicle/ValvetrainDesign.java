package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.engine.Friction;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Valve train (配氣系統): how the cams move the valves (direct-acting buckets, finger followers or rocker arms; an OHV
 * head always works through pushrods and rockers), flat or roller followers, valve springs (preload at the seat and
 * rate), the moving mass per valve besides the valve itself (follower, retainer, keepers and a third of the spring),
 * the cam drive, and variable cam phasing ranges (crank degrees each way; 0 = fixed timing).
 * The mechanism and followers pick the friction model's constants (Friction.Valvetrain); springs and moving mass set
 * the speed at which the valves float (ValveLift.floatSpeed). The cam drive only adds mass here.
 */
public record ValvetrainDesign(String name, Mechanism mechanism, boolean rollerFollowers, double springPreloadN, double springRateNPerM,
                               double followerMassKg, Drive drive, double intakePhaseRangeDeg, double exhaustPhaseRangeDeg,
                               double massGivenKg) implements EngineComponent {
    public static final String TYPE = "aerocore:valvetrain";

    public enum Mechanism { DIRECT_ACTING, FINGER_FOLLOWER, ROCKER_ARM }
    public enum Drive { CHAIN, BELT, GEAR }

    public ValvetrainDesign { Objects.requireNonNull(name, "name"); Objects.requireNonNull(mechanism, "mechanism"); Objects.requireNonNull(drive, "drive"); }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        EngineComponent.Rules.positive(issues, "springPreloadN", springPreloadN);
        EngineComponent.Rules.positive(issues, "springRateNPerM", springRateNPerM);
        EngineComponent.Rules.nonnegative(issues, "followerMassKg", followerMassKg);
        EngineComponent.Rules.nonnegative(issues, "intakePhaseRangeDeg", intakePhaseRangeDeg);
        EngineComponent.Rules.nonnegative(issues, "exhaustPhaseRangeDeg", exhaustPhaseRangeDeg);
        DesignIssue.range(issues, "intakePhaseRangeDeg", intakePhaseRangeDeg, 0, 60, "production cam phasers");
        DesignIssue.range(issues, "exhaustPhaseRangeDeg", exhaustPhaseRangeDeg, 0, 60, "production cam phasers");
        EngineComponent.Rules.mass(issues, massGivenKg);
        return issues;
    }

    /** The friction model's configuration for a head layout. DOHC with rocker arms takes the SOHC rocker-arm constants (Sandoval has none for it). */
    public Friction.Valvetrain friction(CylinderHeadDesign.Layout layout) {
        if (layout == CylinderHeadDesign.Layout.OHV) { return Friction.Valvetrain.OHV; }
        boolean dohc = layout == CylinderHeadDesign.Layout.DOHC;
        return switch (mechanism) {
            case DIRECT_ACTING -> dohc ? Friction.Valvetrain.DOHC_DIRECT_ACTING : Friction.Valvetrain.SOHC_DIRECT_ACTING;
            case FINGER_FOLLOWER -> dohc ? Friction.Valvetrain.DOHC_FINGER_FOLLOWER : Friction.Valvetrain.SOHC_FINGER_FOLLOWER;
            case ROCKER_ARM -> Friction.Valvetrain.SOHC_ROCKER_ARM;
        };
    }

    /** Followers, retainers and drive for an engine with this many valves, kg: the given mass, or followers plus 2 kg of drive (an estimate). */
    public double massFor(int valves) { return Double.isNaN(massGivenKg) ? valves * followerMassKg + 2 : massGivenKg; }
    @Override public double massKg() { return Double.isNaN(massGivenKg) ? 0 : massGivenKg; }

    @Override public Map<String, Object> parameters() {
        return Fields.object("mechanism", mechanism.name().toLowerCase(), "rollerFollowers", rollerFollowers, "springPreloadN", springPreloadN,
                "springRateNPerM", springRateNPerM, "followerMassKg", followerMassKg, "drive", drive.name().toLowerCase(),
                "intakePhaseRangeDeg", intakePhaseRangeDeg, "exhaustPhaseRangeDeg", exhaustPhaseRangeDeg, "massKg", EngineComponent.Rules.mass(massGivenKg));
    }

    static ValvetrainDesign read(String name, Fields f) {
        return new ValvetrainDesign(name, EngineComponent.Rules.choice(f, "mechanism", Mechanism.class, Mechanism.DIRECT_ACTING),
                f.bool("rollerFollowers", false), f.number("springPreloadN"), f.number("springRateNPerM"), f.number("followerMassKg", 0.05),
                EngineComponent.Rules.choice(f, "drive", Drive.class, Drive.CHAIN), f.number("intakePhaseRangeDeg", 0), f.number("exhaustPhaseRangeDeg", 0),
                EngineComponent.Rules.massOr(f));
    }
}

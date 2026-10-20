package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * A transfer case (0.24): the gearbox's output ("input") to the front and rear axles ("front", "rear"), through a
 * range gear ("range": high, or low on "range" > 0.5; it changes only below 1 m/s-ish output speed, 5 rad/s, else it
 * stays where it was, as real ones need the vehicle stopped or crawling).
 * <ul>
 * <li>Full-time: a centre differential, the front taking frontShare of the torque whatever the speeds (a planetary
 *     centre differential; 0.5 bevel), open, Torsen (bias ratio) or viscous, locking on "lock".</li>
 * <li>Part-time: "mode" 0 two-wheel high (rear only, the front axle free), 1 four high (front and rear locked
 *     together, no differential: on grippy ground the axles fight round corners), 2 four low (the same in low range).</li>
 * <li>On-demand: the primary axle (rear, or front when frontPrimary) driven, the other through a multi-plate coupling
 *     whose capacity rises with the slip between them (Haldex's first generation: about linear in the speed
 *     difference, couplingSlope N m per rad/s) or with a preload commanded on "couple" (0..1, later generations' pump),
 *     up to its largest capacity.</li>
 * </ul>
 * Shown values: range (1 high, 2 low), mode (part-time) or lock (full-time) or coupling torque share (on-demand).
 */
public final class TransferPart implements Part {
    /** Kinds of transfer case. */
    public enum Kind { FULL_TIME, PART_TIME, ON_DEMAND }
    /** Centre differentials. */
    public enum Centre { OPEN, TORSEN, VISCOUS }

    /**
     * A transfer case: kind, low range ratio (1 for none), efficiency, the front's share (full-time), centre
     * differential, its bias ratio (Torsen) or viscous coefficient (N m s/rad), whether it locks, the on-demand
     * coupling's slope (N m per rad/s of slip) and largest torque (N m), which axle is primary (on-demand), inertias of
     * input, range gear and each output (kg m^2), and whether the input lies across the vehicle (along -z).
     */
    public record Spec(Kind kind, double lowRatio, double efficiency, double frontShare, Centre centre, double biasRatio, double viscousNmsPerRad,
                       boolean lockable, double couplingSlope, double couplingMaxNm, boolean frontPrimary, double inputInertiaKgM2, double rangeInertiaKgM2,
                       double outputInertiaKgM2, boolean transverse) {
        public Spec {
            Objects.requireNonNull(kind, "kind"); Objects.requireNonNull(centre, "centre");
            if (!(lowRatio >= 1) || !(efficiency > 0 && efficiency <= 1) || !(frontShare > 0 && frontShare < 1)) { throw new IllegalArgumentException("Transfer values out of range"); }
            if (!(inputInertiaKgM2 > 0) || !(rangeInertiaKgM2 > 0) || !(outputInertiaKgM2 > 0)) { throw new IllegalArgumentException("Inertias must be positive"); }
        }
    }

    /** Output speed below which the range may change, rad/s. */
    public static final double RANGE_CHANGE = 5;

    private final PartDesign design;
    private final Spec spec;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    public TransferPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.spec = Objects.requireNonNull(spec, "spec");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public Spec spec() { return spec; }
    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }

    @Override public List<Rotor> rotors() {
        Vec3d in = spec.transverse() ? Vec3d.Z.scale(-1) : Vec3d.X;
        return List.of(new Rotor("input", in, spec.inputInertiaKgM2(), false), new Rotor("range", in, spec.rangeInertiaKgM2(), false),
                new Rotor("front", Vec3d.X, spec.outputInertiaKgM2(), false), new Rotor("rear", Vec3d.X, spec.outputInertiaKgM2(), false));
    }

    @Override public List<Link> links() { return List.of(new Link(0, 1), new Link(1, 2), new Link(1, 3), new Link(2, 3), new Link(0, 2), new Link(0, 3)); }

    @Override public List<String> inputs() {
        return switch (spec.kind()) {
            case FULL_TIME -> spec.lockable() ? List.of("range", "lock") : List.of("range");
            case PART_TIME -> List.of("mode");
            case ON_DEMAND -> List.of("range", "couple");
        };
    }

    @Override public int internalStates() { return 1; }
    @Override public double[] initialInternals() { return new double[]{1}; }
    @Override public int shownValues() { return 2; }

    @Override public void loads(PartStep step) {
        boolean wantLow = spec.kind() == Kind.PART_TIME ? step.input("mode", 0) > 1.5 : step.input("range", 0) > 0.5;
        int range = (int) step.internal(0);
        boolean slow = Math.abs(step.rotorSpeed(2)) < RANGE_CHANGE && Math.abs(step.rotorSpeed(3)) < RANGE_CHANGE;
        if (spec.lowRatio() > 1 && slow) { range = wantLow ? 2 : 1; }
        if (spec.lowRatio() <= 1) { range = 1; }
        step.row(new int[]{0, 1}, new double[]{1, -(range == 2 ? spec.lowRatio() : 1)}, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, spec.efficiency());
        double second = 0;
        switch (spec.kind()) {
            case FULL_TIME -> {
                int d = step.differential(1, 2, 3, 1, spec.frontShare(), 1);
                boolean locked = spec.lockable() && step.input("lock", 0) > 0.5;
                if (locked) { step.clutch(2, 3, 1, Double.POSITIVE_INFINITY); }
                else if (spec.centre() == Centre.TORSEN) { step.biasFriction(2, 3, 0, (spec.biasRatio() - 1) / (2 * (spec.biasRatio() + 1)), d); }
                else if (spec.centre() == Centre.VISCOUS) { step.viscous(2, 3, 1, spec.viscousNmsPerRad()); }
                second = locked ? 1 : 0;
            }
            case PART_TIME -> {
                int mode = (int) Math.round(Math.clamp(step.input("mode", 0), 0, 2));
                step.gear(1, 3, 1, 1);
                if (mode >= 1) { step.gear(1, 2, 1, 1); }
                second = mode;
            }
            case ON_DEMAND -> {
                int primary = spec.frontPrimary() ? 2 : 3, other = spec.frontPrimary() ? 3 : 2;
                step.gear(1, primary, 1, 1);
                double slip = Math.abs(step.rotorSpeed(primary) - step.rotorSpeed(other));
                double preload = Math.clamp(step.input("couple", 0), 0, 1) * spec.couplingMaxNm();
                double cap = Math.min(Math.max(preload, spec.couplingSlope() * slip), spec.couplingMaxNm());
                step.clutch(primary, other, 1, cap);
                second = spec.couplingMaxNm() > 0 ? cap / spec.couplingMaxNm() : 0;
            }
        }
        step.setInternal(0, range);
        step.show(0, range);
        step.show(1, second);
        step.diagnostic("range", range);
        step.diagnostic("mode", second);
    }
}

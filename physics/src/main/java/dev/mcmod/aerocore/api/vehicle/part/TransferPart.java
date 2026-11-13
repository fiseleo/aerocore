package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.damage.Wear;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
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
 *
 * <p>0.42: a case given a Wearing (wearing) has one condition, GEARS (0): teeth, chain and bearings, worn by Wear.running
 * with the Wearing's life at its rated input torque and speed, from the range gear's torque and the input's speed. Its
 * efficiency falls to TransmissionPart.gearEfficiency of the design's (a tenth less at the end), and at 1 it drives no
 * more: the range gear's row is not made (the input turns free; the outputs keep what joins them to each other). A part
 * wrapping it lists the condition first and passes after() on. Diagnostic "gearsUsed". Without a Wearing: no
 * conditions, as before 0.42.
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

    /** 0.42: the name of a wearing case's condition (index 0). */
    public static final String GEARS = "gears";
    private static final List<String> CONDITIONS = List.of(GEARS);

    /**
     * 0.42: what wears a transfer case (the caller's numbers): its life (s) at a rated input torque (N m) and speed
     * (rad/s), with Wear.running's exponents on speed and load.
     */
    public record Wearing(double lifeS, double ratedTorqueNm, double ratedSpeedRadPerS, double speedExponent, double loadExponent) {
        public Wearing {
            if (!(lifeS > 0) || !(ratedTorqueNm > 0) || !(ratedSpeedRadPerS > 0)) { throw new IllegalArgumentException("Life and rated torque and speed must be positive"); }
            if (!(speedExponent >= 0) || !Double.isFinite(speedExponent) || !(loadExponent >= 0) || !Double.isFinite(loadExponent)) {
                throw new IllegalArgumentException("Exponents must be finite and not negative");
            }
        }

        /** Typical exponents: a life in revolutions (1) falling with the cube of the load (ISO 281's ball bearings, as Wear.running says). */
        public static Wearing typical(double lifeS, double ratedTorqueNm, double ratedSpeedRadPerS) { return new Wearing(lifeS, ratedTorqueNm, ratedSpeedRadPerS, 1, 3); }
    }

    private final PartDesign design;
    private final Spec spec;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final Wearing wearing;

    public TransferPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues) { this(design, spec, mass, issues, null); }

    private TransferPart(PartDesign design, Spec spec, MassProperties mass, List<DesignIssue> issues, Wearing wearing) {
        this.design = Objects.requireNonNull(design, "design");
        this.spec = Objects.requireNonNull(spec, "spec");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        this.wearing = wearing;
    }

    /** 0.42: this case wearing by these numbers, with the condition GEARS; null: one that does not wear. A copy. */
    public TransferPart wearing(Wearing wearing) { return new TransferPart(design, spec, mass, issues, wearing); }
    /** 0.42: what wears this case, or null. */
    public Wearing wearing() { return wearing; }
    @Override public List<String> conditions() { return wearing == null ? List.of() : CONDITIONS; }

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
        if (wearing == null) {
            step.row(new int[]{0, 1}, new double[]{1, -(range == 2 ? spec.lowRatio() : 1)}, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, spec.efficiency());
        } else {
            // 0.42: worn gears lose a little more; used up they drive nothing. after() wears them by the range row
            double used = step.used(0);
            int row = -1;
            if (!(used >= 1)) {
                row = step.row(new int[]{0, 1}, new double[]{1, -(range == 2 ? spec.lowRatio() : 1)}, 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
                        used > 0 ? spec.efficiency() * TransmissionPart.gearEfficiency(used) : spec.efficiency());
            }
            step.carry(new RangeRow(row));
            step.diagnostic("gearsUsed", used);
        }
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

    /** 0.42: the range gear's row of a wearing case's step (-1: none, the gears being used up). */
    private record RangeRow(int row) {}

    /** 0.42: the gears wear by the torque through the range gear and the input's speed (Wear.running). */
    @Override public void after(PartAfter a) {
        if (wearing == null || !(a.carried() instanceof RangeRow r) || r.row() < 0) { return; }
        double torque = Math.abs(a.rowTorque(r.row())), speed = Math.abs(a.rotorSpeed(0));
        if (!(torque < Double.POSITIVE_INFINITY) || !(speed < Double.POSITIVE_INFINITY)) { return; }
        double worn = Wear.running(a.dt(), wearing.lifeS(), speed / wearing.ratedSpeedRadPerS(), wearing.speedExponent(), torque / wearing.ratedTorqueNm(),
                wearing.loadExponent());
        if (worn > 0) { a.wear(0, Math.min(worn, 1)); }
    }
}

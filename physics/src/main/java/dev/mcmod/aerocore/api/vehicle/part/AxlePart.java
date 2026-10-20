package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartAfter;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.wheel.BrakeControl;
import dev.mcmod.aerocore.api.wheel.WheelCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * An axle (0.24): two wheel corners (left at -z, right at +z of the part frame, x forward, y up; origin on the axle's
 * centreline), steered or not, driven through a differential or not, with an anti-roll bar, brakes and a handbrake.
 *
 * <ul>
 * <li>Rotors: "left" and "right" (the wheels, spinning about -z so that rolling forward is positive), and when driven
 *     "input" first (the differential's input: along the axle for a transverse drive, along +x for a propeller shaft),
 *     which a shaft joins.</li>
 * <li>Differential (driven): input = final ratio x mean of the wheels (PartStep.differential, bevel gears 0.5). Open;
 *     clutch-pack limited slip (preload + lock factor x the input's torque); Torsen (bias ratio TBR: the slower side may
 *     carry TBR times the faster's); viscous coupling between the wheels; spool (locked). Lockable ones lock on the
 *     "lock" input (above 0.5).</li>
 * <li>Steering (when steered): "steer" -1..1 (left positive) times the largest angle; the wheels' angles blend from
 *     parallel (0) to full Ackermann (1) about the zero-steer line (the rear axle of a two-axle car) at the given signed
 *     distance (0.33: negative for an axle behind it, a steered rear axle, whose wheels then meet the front ones' turn
 *     centre).</li>
 * <li>Anti-roll bar: its rate (N/m at the wheels) times the difference of the two compressions loads each wheel's
 *     suspension (explicit, from the step's start).</li>
 * <li>Brakes on "brake" (0..1) through an optional rule (ABS, BrakeControl); a handbrake on "handbrake" holds both
 *     wheels with its torque (mechanical: no fade).</li>
 * <li>0.33: air springs with a levelling valve (Suspension.Levelling) on "ride_height" (0 low, 0.5 the design height,
 *     1 high): each wheel's air scale is an internal state after the others, which the valve moves after each step.</li>
 * </ul>
 * The wheels' and axle's mass rides with the body (no unsprung mass). Shown values: each wheel's spin, steer and
 * compression (left first), then the input's angle when driven.
 */
public final class AxlePart implements Part {
    /** Differential kinds. */
    public enum Differential { OPEN, CLUTCH, TORSEN, VISCOUS, SPOOL }

    /**
     * A driven axle's differential: final ratio (input over carrier speed), efficiency, the input's inertia (kg m^2:
     * pinion, crown wheel, carrier at the input's speed), kind, clutch preload (N m) and lock factor (per N m of the
     * input's torque), Torsen bias ratio, viscous coefficient (N m s/rad between the wheels), whether it locks on demand,
     * and whether the input lies along the axle (transverse drive) rather than along +x.
     */
    public record Drive(double finalRatio, double efficiency, double inputInertiaKgM2, Differential differential, double preloadNm, double lockFactor,
                        double biasRatio, double viscousNmsPerRad, boolean lockable, boolean inputAlongAxle) {
        public Drive {
            Objects.requireNonNull(differential, "differential");
            if (!(finalRatio > 0) || !(efficiency > 0 && efficiency <= 1) || !(inputInertiaKgM2 > 0)) { throw new IllegalArgumentException("Drive values out of range"); }
        }
    }

    /**
     * Steering: the largest road-wheel angle (rad), Ackermann share 0..1 and the signed distance (m) from the zero-steer
     * line the Ackermann geometry is laid out for: positive ahead of it (a front axle: the wheelbase), negative behind it
     * (0.33: a steered rear axle), 0 for parallel steering.
     */
    public record Steering(double maxSteerRad, double ackermann, double wheelbaseM) {}

    private final PartDesign design;
    private final WheelCorner left, right;
    private final Drive drive;
    private final Steering steering;
    private final double antiRollNPerM, handbrakeNm;
    private final BrakeControl control;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final int wheel0, memory;

    public AxlePart(PartDesign design, WheelCorner left, WheelCorner right, Drive drive, Steering steering, double antiRollNPerM, double handbrakeNm,
                    BrakeControl control, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.left = Objects.requireNonNull(left, "left"); this.right = Objects.requireNonNull(right, "right");
        if (left.spec().side() != -1 || right.spec().side() != 1) { throw new IllegalArgumentException("The left wheel is on side -1, the right on +1"); }
        this.drive = drive; this.steering = steering; this.antiRollNPerM = antiRollNPerM; this.handbrakeNm = handbrakeNm; this.control = control;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        wheel0 = drive == null ? 0 : 1;
        memory = 2 * WheelCorner.STATES;
    }

    public WheelCorner left() { return left; }
    public WheelCorner right() { return right; }
    public Drive drive() { return drive; }
    public Steering steering() { return steering; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }

    @Override public List<Rotor> rotors() {
        List<Rotor> r = new ArrayList<>();
        if (drive != null) { r.add(new Rotor("input", drive.inputAlongAxle() ? Vec3d.Z.scale(-1) : Vec3d.X, drive.inputInertiaKgM2(), false)); }
        r.add(new Rotor("left", Vec3d.Z.scale(-1), left.inertiaKgM2(), false));
        r.add(new Rotor("right", Vec3d.Z.scale(-1), right.inertiaKgM2(), false));
        return r;
    }

    @Override public List<Link> links() { return drive == null ? List.of() : List.of(new Link(0, 1), new Link(0, 2), new Link(1, 2)); }

    @Override public List<String> inputs() {
        List<String> in = new ArrayList<>(List.of("brake"));
        if (steering != null) { in.add("steer"); }
        if (handbrakeNm > 0) { in.add("handbrake"); }
        if (drive != null && drive.lockable()) { in.add("lock"); }
        if (levelling() != null) { in.add("ride_height"); }
        return in;
    }

    /** 0.33: the air springs' levelling valve (the left wheel's suspension's), or null. */
    public dev.mcmod.aerocore.api.wheel.Suspension.Levelling levelling() { return left.spec().suspension().levelling(); }

    /** 0.33: where the wheels' air scales are among the internal states (after the brake rule's memory). */
    private int air() { return 2 * WheelCorner.STATES + (control == null ? 0 : 2 * control.memory()); }

    @Override public int internalStates() { return air() + (levelling() == null ? 0 : 2); }
    @Override public double[] initialInternals() {
        double[] s = new double[internalStates()];
        left.initial(s, 0);
        right.initial(s, WheelCorner.STATES);
        if (levelling() != null) { s[air()] = 1; s[air() + 1] = 1; }
        return s;
    }
    /** Shown: each side's spin, steer, compression; the drive shaft's angle if driven; 0.30: each side's slip speed and load (WheelCorner.EXTRA). */
    @Override public int shownValues() { return 2 * WheelCorner.SHOWN + (drive == null ? 0 : 1) + 2 * WheelCorner.EXTRA; }

    /** 0.30: where the sides' slip speeds and loads start among the shown values (left, then right). */
    public int extraShown() { return 2 * WheelCorner.SHOWN + (drive == null ? 0 : 1); }

    /** What a step carries to after() (0.33: the ride height's target compression, NaN without levelling). */
    private record Carry(WheelCorner.Carry left, WheelCorner.Carry right, double target) {}

    /** The road-wheel angles {left, right} (rad, left positive) for a steering input. */
    public double[] steerAngles(double input) {
        if (steering == null) { return new double[]{0, 0}; }
        double d = Math.clamp(input, -1, 1) * steering.maxSteerRad(), wheelbase = Math.abs(steering.wheelbaseM());
        if (Math.abs(d) < 1e-9 || wheelbase == 0) { return new double[]{d, d}; }
        double track = left.spec().trackM() > 0 ? left.spec().trackM() : Math.abs(right.spec().droopHub().z() - left.spec().droopHub().z());
        double radius = wheelbase / Math.tan(Math.abs(d)), sign = Math.signum(d);
        double inner = Math.atan(wheelbase / (radius - track / 2)), outer = Math.atan(wheelbase / (radius + track / 2));
        double a = steering.ackermann();
        // the turn centre is to the left (the left wheel the inner one) when an axle ahead of the zero-steer line steers
        // left, or one behind it steers right (0.33; before, the sign of the angle alone: a front axle's)
        boolean leftInner = d * steering.wheelbaseM() > 0;
        double l = sign * (Math.abs(d) + a * ((leftInner ? inner : outer) - Math.abs(d)));
        double r = sign * (Math.abs(d) + a * ((leftInner ? outer : inner) - Math.abs(d)));
        return new double[]{l, r};
    }

    /** 0.33: the wheels follow a twisting frame. */
    @Override public boolean probes() { return true; }

    @Override public void probe(PartStep step) {
        double[] steer = steerAngles(step.input("steer", 0));
        double roll = antiRollNPerM * (step.internal(0) - step.internal(WheelCorner.STATES));
        double airL = levelling() == null ? 1 : step.internal(air()), airR = levelling() == null ? 1 : step.internal(air() + 1);
        step.keep(new WheelCorner.Probe[]{left.probe(step, 0, steer[0], roll, airL), right.probe(step, WheelCorner.STATES, steer[1], -roll, airR)});
    }

    @Override public void loads(PartStep step) {
        double[] steer = steerAngles(step.input("steer", 0));
        double brake = Math.clamp(step.input("brake", 0), 0, 1), hand = handbrakeNm * Math.clamp(step.input("handbrake", 0), 0, 1);
        double roll = antiRollNPerM * (step.internal(0) - step.internal(WheelCorner.STATES));
        int m = control == null ? 0 : control.memory();
        WheelCorner.Probe[] probes = step.kept() instanceof WheelCorner.Probe[] k ? k : new WheelCorner.Probe[2];
        double airL = levelling() == null ? 1 : step.internal(air()), airR = levelling() == null ? 1 : step.internal(air() + 1);
        WheelCorner.Carry l = left.loads(step, wheel0, 0, 0, steer[0], brake, control, memory, hand, roll, "left", probes[0], airL);
        WheelCorner.Carry r = right.loads(step, wheel0 + 1, WheelCorner.STATES, WheelCorner.SHOWN, steer[1], brake, control, memory + m, hand, -roll, "right", probes[1], airR);
        if (drive != null) {
            int d = step.differential(0, 1, 2, drive.finalRatio(), 0.5, drive.efficiency());
            boolean locked = drive.differential() == Differential.SPOOL || (drive.lockable() && step.input("lock", 0) > 0.5);
            if (locked) {
                step.clutch(1, 2, 1, Double.POSITIVE_INFINITY);
            } else {
                switch (drive.differential()) {
                    case CLUTCH -> step.biasFriction(1, 2, drive.preloadNm(), drive.lockFactor(), d);
                    case TORSEN -> step.biasFriction(1, 2, 0, drive.finalRatio() * (drive.biasRatio() - 1) / (2 * (drive.biasRatio() + 1)), d);
                    case VISCOUS -> step.viscous(1, 2, 1, drive.viscousNmsPerRad());
                    default -> { }
                }
            }
            step.diagnostic("locked", locked ? 1 : 0);
        }
        step.diagnostic("speedMps", step.state().velocity().length());
        double target = Double.NaN;
        if (levelling() != null) {
            target = levelling().target(left.spec().suspension().designCompressionM(), step.input("ride_height", 0.5));
            step.diagnostic("rideTargetM", target);
        }
        step.carry(new Carry(l, r, target));
    }

    @Override public void after(PartAfter a) {
        Carry c = (Carry) a.carried();
        if (c == null) { return; }
        left.after(a, c.left(), wheel0, 0, 0, extraShown());
        right.after(a, c.right(), wheel0 + 1, WheelCorner.STATES, WheelCorner.SHOWN, extraShown() + WheelCorner.EXTRA);
        // 0.33: the levelling valve fills or vents each air spring toward the ride height's compression
        if (levelling() != null && !Double.isNaN(c.target())) {
            for (int k = 0; k < 2; k++) {
                int i = air() + k;
                a.setInternal(i, levelling().next(a.internal(i), a.internal(k * WheelCorner.STATES), c.target(), a.dt()));
            }
        }
        if (drive != null && Math.abs(a.rotorSpeed(0)) >= WheelCorner.SHOWN_CREEP_RAD_S) {
            a.show(2 * WheelCorner.SHOWN, Math.IEEEremainder(a.shown(2 * WheelCorner.SHOWN) + a.rotorSpeed(0) * a.dt(), 2 * Math.PI));
        }
    }
}

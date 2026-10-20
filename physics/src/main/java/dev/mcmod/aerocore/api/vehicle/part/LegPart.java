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

import java.util.List;
import java.util.Objects;

/**
 * A landing-gear leg (0.24): one wheel corner (side 0 on the centreline, or a main leg's side for the camber sign) on
 * its strut (the corner's suspension: an oleo-pneumatic strut, a spring-steel leg, or stiff), steered from "steer",
 * castering freely or fixed, braked from "brake" (bind it to the side's own brake channel for differential braking).
 *
 * <p>A castering wheel turns toward the way its pivot moves over the ground, lagging by its trail: each step the angle
 * closes on the direction of travel by speed x dt / trail (a kinematic caster with no shimmy). Rotor "wheel" spins about
 * -z (rolling forward positive). Shown values: spin, steer, compression; 0.30: slip speed and load (WheelCorner.EXTRA),
 * then how far it is folded away (0 down, 1 up).
 *
 * <p>0.30: a retractable leg (Retraction) folds away on "retract" (above 0.5 up, else down) at a constant rate over its
 * time, but not while weight is on its wheel (in contact, or its strut compressed more than 2 cm). Once it starts to
 * move it casts no rays and touches nothing: the wheel spins down on its brake. While down it drags as a bluff body:
 * 1/2 rho V^2 Cd A along the air past it, less as it folds (Cd about 0.55 to 1.0 on the frontal area of a wheel and its
 * strut: Hoerner, Fluid-Dynamic Drag, ch. 13); a fixed leg has no drag here, as before 0.30 (its drag is a design's to
 * add to its body). A fixed leg's inputs and internal states are those of 0.29, so saved vehicles read back.
 *
 * <p>0.36: a telescoping leg (Telescope: a trailer's landing leg) draws its strut up by its stroke on "retract" (above
 * 0.5 up, else down) at a constant rate over its time, loaded or not: let down under a coupled trailer it lifts the
 * trailer off the fifth wheel, drawn up it lets the trailer down onto it. Its wheel keeps its rays and contact, its
 * attachment lifted by the stroke drawn up (WheelCorner's mount lift); the last shown value is the share drawn up.
 * A leg folds (Retraction) or telescopes, not both.
 */
public final class LegPart implements Part {
    /** How the wheel is steered. */
    public enum Steer { FIXED, STEERABLE, CASTERING }

    private final PartDesign design;
    private final WheelCorner wheel;
    private final Steer steer;
    private final double maxSteerRad, trailM;
    private final BrakeControl control;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final Retraction retraction;
    private final Telescope telescope;

    /**
     * 0.30: how a leg folds away: the time from down to up (s), the axis it swings about through its mount (part frame)
     * and the angle (rad; both for drawing it), and its frontal area (m^2) and drag coefficient while down.
     */
    public record Retraction(double timeS, Vec3d axis, double angleRad, double frontalAreaM2, double dragCoefficient) {
        public Retraction {
            if (!(timeS > 0) || !(frontalAreaM2 >= 0) || !(dragCoefficient >= 0)) { throw new IllegalArgumentException("Bad retraction"); }
            Objects.requireNonNull(axis, "axis");
        }
    }

    public LegPart(PartDesign design, WheelCorner wheel, Steer steer, double maxSteerRad, double trailM, BrakeControl control, MassProperties mass,
                   List<DesignIssue> issues) {
        this(design, wheel, steer, maxSteerRad, trailM, control, mass, issues, null);
    }

    /** 0.36: how a leg's strut draws up: its stroke (m) and the time from down to up (s). */
    public record Telescope(double strokeM, double timeS) {
        public Telescope {
            if (!(strokeM > 0) || !Double.isFinite(strokeM) || !(timeS > 0) || !Double.isFinite(timeS)) { throw new IllegalArgumentException("Bad telescope"); }
        }
    }

    public LegPart(PartDesign design, WheelCorner wheel, Steer steer, double maxSteerRad, double trailM, BrakeControl control, MassProperties mass,
                   List<DesignIssue> issues, Retraction retraction) {
        this(design, wheel, steer, maxSteerRad, trailM, control, mass, issues, retraction, null);
    }

    /** 0.36: with a telescoping strut (retraction then null). */
    public LegPart(PartDesign design, WheelCorner wheel, Steer steer, double maxSteerRad, double trailM, BrakeControl control, MassProperties mass,
                   List<DesignIssue> issues, Retraction retraction, Telescope telescope) {
        if (retraction != null && telescope != null) { throw new IllegalArgumentException("A leg folds or telescopes, not both"); }
        this.retraction = retraction;
        this.telescope = telescope;
        this.design = Objects.requireNonNull(design, "design");
        this.wheel = Objects.requireNonNull(wheel, "wheel");
        this.steer = Objects.requireNonNull(steer, "steer");
        this.maxSteerRad = maxSteerRad; this.trailM = Math.max(trailM, 0.01); this.control = control;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public WheelCorner wheel() { return wheel; }
    public Steer steer() { return steer; }
    /** 0.30: how it folds away, or null for a fixed leg. */
    public Retraction retraction() { return retraction; }
    /** 0.36: how its strut draws up, or null. */
    public Telescope telescope() { return telescope; }
    /** 0.36: whether "retract" moves it (folding or telescoping). */
    private boolean moves() { return retraction != null || telescope != null; }

    /** 0.30: where the retraction's fraction is kept among the internal states (retractable legs only). */
    private int retractState() { return WheelCorner.STATES + 1 + (control == null ? 0 : control.memory()); }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor("wheel", Vec3d.Z.scale(-1), wheel.inertiaKgM2(), false)); }
    @Override public List<String> inputs() {
        List<String> in = new java.util.ArrayList<>(steer == Steer.STEERABLE ? List.of("brake", "steer") : List.of("brake"));
        if (moves()) { in.add("retract"); }
        return in;
    }

    /** Wheel states, then the caster angle, then the brake rule's memory; 0.30: a retractable leg's fraction folded. */
    @Override public int internalStates() { return retractState() + (moves() ? 1 : 0); }
    @Override public double[] initialInternals() {
        double[] s = new double[internalStates()];
        wheel.initial(s, 0);
        return s;
    }
    @Override public int shownValues() { return WheelCorner.SHOWN + WheelCorner.EXTRA + 1; }

    /** What a step carries to after(): the wheel's, and whether it was told to fold away. */
    private record Carry(WheelCorner.Carry wheel, boolean up) {}

    @Override public void loads(PartStep step) {
        double angle = switch (steer) {
            case FIXED -> 0;
            case STEERABLE -> Math.clamp(step.input("steer", 0), -1, 1) * maxSteerRad;
            case CASTERING -> step.internal(WheelCorner.STATES);
        };
        double brake = Math.clamp(step.input("brake", 0), 0, 1), folded = retraction == null ? 0 : step.internal(retractState());
        double drawn = telescope == null ? 0 : telescope.strokeM() * step.internal(retractState());
        WheelCorner.Carry c = folded > 0
                ? wheel.free(step, 0, 0, 0, angle, brake, control, WheelCorner.STATES + 1, 0, 0, "wheel")
                : drawn != 0 ? wheel.loads(step, 0, 0, 0, angle, brake, control, WheelCorner.STATES + 1, 0, 0, "wheel", null, 1, drawn)
                : wheel.loads(step, 0, 0, 0, angle, brake, control, WheelCorner.STATES + 1, 0, 0, "wheel");
        if (retraction != null && folded < 1 && retraction.frontalAreaM2() > 0) {
            // a bluff body in the air while down (less as it folds)
            Vec3d at = wheel.hub(step.internal(0));
            dev.mcmod.aerocore.api.FluidState air = step.environment().point(step.point(at)).air();
            Vec3d v = step.velocity(at).subtract(air.velocity());
            double speed = v.length();
            if (speed > 1e-6) {
                Vec3d drag = v.scale(-0.5 * air.densityKgPerM3() * speed * retraction.dragCoefficient() * retraction.frontalAreaM2() * (1 - folded));
                step.force(new dev.mcmod.aerocore.api.AppliedForce(dev.mcmod.aerocore.api.AppliedForce.Kind.DRAG, step.offset(at), drag));
                step.diagnostic("gearDragN", drag.length());
            }
        }
        step.carry(new Carry(c, moves() && step.input("retract", 0) > 0.5));
    }

    @Override public void after(PartAfter a) {
        Carry carried = (Carry) a.carried();
        if (carried == null) { return; }
        WheelCorner.Carry c = carried.wheel();
        wheel.after(a, c, 0, 0, 0, WheelCorner.SHOWN);
        if (retraction != null) {
            double folded = a.internal(retractState());
            boolean loaded = c.contact() || a.internal(0) > WEIGHT_ON_WHEELS_M;
            double target = carried.up() && !(folded == 0 && loaded) ? 1 : 0;
            double next = target > folded ? Math.min(1, folded + a.dt() / retraction.timeS()) : Math.max(0, folded - a.dt() / retraction.timeS());
            a.setInternal(retractState(), next);
            a.show(WheelCorner.SHOWN + WheelCorner.EXTRA, PartStep.stepped(a.shown(WheelCorner.SHOWN + WheelCorner.EXTRA), next, 0.01));
        }
        if (telescope != null) {
            // drawn up or let down at its rate, loaded or not
            double drawn = a.internal(retractState()), target = carried.up() ? 1 : 0;
            double next = target > drawn ? Math.min(1, drawn + a.dt() / telescope.timeS()) : Math.max(0, drawn - a.dt() / telescope.timeS());
            a.setInternal(retractState(), next);
            a.show(WheelCorner.SHOWN + WheelCorner.EXTRA, PartStep.stepped(a.shown(WheelCorner.SHOWN + WheelCorner.EXTRA), next, 0.01));
        }
        if (steer == Steer.CASTERING) {
            // the pivot's velocity over the ground in the part's level plane
            Vec3d v = a.partDirection(a.velocity(wheel.hub(a.internal(0))).subtract(c.groundVelocity()));
            double speed = Math.hypot(v.x(), v.z()), angle = a.internal(WheelCorner.STATES);
            if (speed > 0.05) {
                double target = v.x() >= 0 ? Math.atan2(-v.z(), v.x()) : Math.atan2(v.z(), -v.x());
                target = Math.clamp(target, -maxSteerRad, maxSteerRad);
                angle += (target - angle) * Math.min(1, speed * a.dt() / trailM);
            }
            a.setInternal(WheelCorner.STATES, angle);
            a.show(1, PartStep.stepped(a.shown(1), angle, WheelCorner.SHOWN_STEER_RAD));
        }
    }

    /** 0.30: a strut compressed more than this (m) has weight on it: the gear stays down. */
    static final double WEIGHT_ON_WHEELS_M = 0.02;
}

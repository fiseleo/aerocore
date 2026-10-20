package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.aero.Wing;
import dev.mcmod.aerocore.api.gasdynamics.AeroHeating;
import dev.mcmod.aerocore.api.body.BodyAerodynamics;
import dev.mcmod.aerocore.api.body.BodyShape;
import dev.mcmod.aerocore.api.body.RoadAero;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Mount;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.vehicle.WakeSource;
import dev.mcmod.aerocore.api.vehicle.FrameTwist;

import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A body made of modules (BodyShape; its origin at the rear end on the reference line): air loads from its shape
 * (BodyAerodynamics) wherever it is, and in the water those of a hard-chine planing hull (PlaningHullPart, steps
 * included) or of a displacement hull (HullPart without its own windage), whichever its bottom is, from the sections
 * that keep the water out (an open module only up to its sill). Mass, mount points and seats come from the design;
 * so do its fuel tanks (0.26), whose fuel is not in its mass: the vehicle carries that (the displacement hull's
 * design waterline counts their first fill). 0.33: a car body that carries its axles itself (a unibody) may twist as a
 * frame (FrameTwist, from its shell: BodyStructure.torsion), the stations' twists its internal states. 0.34: a road
 * vehicle's body may carry an aero kit (RoadAero: floor and diffuser, splitter, lip spoiler, rear wing, drag pieces),
 * its loads added to the shape's with the ground's heights below cast straight down; without one nothing changes.
 */
public final class BodyPart implements Part {
    /** Which water model: the bottom's own (hard-chined: planing), or forced. */
    public enum Hydro { AUTO, PLANING, DISPLACEMENT }

    private final PartDesign design;
    private final BodyShape shape;
    private final BodyAerodynamics aero;
    private final Part water;
    private final boolean planing;
    private final MassProperties mass;
    private final Map<String, Mount> mountPoints;
    private final List<Vec3d> seats;
    private final List<Tank> tanks;
    private final List<DesignIssue> issues;
    private final FrameTwist frame;
    private final RoadAero kit;

    public BodyPart(PartDesign design, BodyShape shape, double extraDragAreaM2, Hydro hydro, int radiationSegments, MassProperties mass,
                    Map<String, Mount> mountPoints, List<Vec3d> seats, List<DesignIssue> issues) {
        this(design, shape, extraDragAreaM2, hydro, radiationSegments, mass, mountPoints, seats, List.of(), issues);
    }

    /** A body with fuel tanks (0.26): its mass without their fuel. */
    public BodyPart(PartDesign design, BodyShape shape, double extraDragAreaM2, Hydro hydro, int radiationSegments, MassProperties mass,
                    Map<String, Mount> mountPoints, List<Vec3d> seats, List<Tank> tanks, List<DesignIssue> issues) {
        this(design, shape, extraDragAreaM2, hydro, radiationSegments, mass, mountPoints, seats, tanks, issues, null);
    }

    /** 0.33: a body that twists as a frame (null: rigid). */
    public BodyPart(PartDesign design, BodyShape shape, double extraDragAreaM2, Hydro hydro, int radiationSegments, MassProperties mass,
                    Map<String, Mount> mountPoints, List<Vec3d> seats, List<Tank> tanks, List<DesignIssue> issues, FrameTwist frame) {
        this(design, shape, extraDragAreaM2, hydro, radiationSegments, mass, mountPoints, seats, tanks, issues, frame, null);
    }

    /** 0.34: a body with an aero kit (null or empty: none). */
    public BodyPart(PartDesign design, BodyShape shape, double extraDragAreaM2, Hydro hydro, int radiationSegments, MassProperties mass,
                    Map<String, Mount> mountPoints, List<Vec3d> seats, List<Tank> tanks, List<DesignIssue> issues, FrameTwist frame, RoadAero kit) {
        this.kit = kit == null || kit.isEmpty() ? null : kit;
        this.frame = frame;
        this.design = Objects.requireNonNull(design, "design");
        this.shape = Objects.requireNonNull(shape, "shape");
        this.aero = new BodyAerodynamics(shape, extraDragAreaM2);
        this.mass = Objects.requireNonNull(mass, "mass");
        this.mountPoints = Map.copyOf(mountPoints);
        this.seats = List.copyOf(seats);
        this.tanks = List.copyOf(tanks);
        this.issues = List.copyOf(issues);
        this.planing = hydro == Hydro.PLANING || hydro == Hydro.AUTO && shape.hardChined();
        BodyShape.Particulars p = shape.particulars();
        if (planing) {
            var hull = shape.planingHull();
            water = new PlaningHullPart(design, hull, hull.chineBeam(), Math.toDegrees(hull.deadrise()), hull.depth(), radiationSegments, mass, List.of());
        } else {
            // the resistance coefficients' design waterline: at its own mass, or half its sealed volume for a body that sinks
            var hull = shape.waterHull();
            double fuel = 0;
            for (Tank t : this.tanks) { fuel += t.fuel().isEmpty() ? 0 : t.initialKg(); }
            double displacement = Math.min(mass.massKg() + fuel, 0.5 * hull.totalVolume() * dev.mcmod.aerocore.api.FluidState.FRESH_WATER_20C.densityKgPerM3());
            water = new HullPart(design, hull, HullModel.Type.SURFACE, Math.max(displacement, 1e-3), 0, radiationSegments, mass, List.of(), false);
        }
    }

    public BodyShape shape() { return shape; }
    public BodyAerodynamics aerodynamics() { return aero; }
    /** The aero kit (0.34), or null. */
    public RoadAero kit() { return kit; }
    /** Whether the water sees a planing hull (else a displacement hull). */
    public boolean planing() { return planing; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return water.collision(); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Vec3d> radiationPoints() { return water.radiationPoints(); }
    @Override public List<WorldBox> deck() { return water.deck(); }
    @Override public List<Vec3d> seats() { return seats; }
    @Override public List<Tank> tanks() { return tanks; }
    @Override public WakeSource wake() { return water.wake(); }
    @Override public Map<String, Mount> mountPoints() { return mountPoints; }
    @Override public void warmUp() { water.warmUp(); aero.warmUp(); if (kit != null) { kit.warmUp(); } }
    @Override public FrameTwist frame() { return frame; }
    @Override public int internalStates() { return frame == null ? 0 : frame.stations(); }

    @Override public void loads(PartStep step) {
        if (frame != null) { step.diagnostic("twistDeg", Math.toDegrees(step.internal(frame.stations() - 1) - step.internal(0))); }
        water.loads(step);
        // the air past the body, at its rear end on the reference line
        FluidState air = step.environment().point(step.point(Vec3d.ZERO)).air();
        Vec3d velocity = step.partDirection(step.velocity(Vec3d.ZERO).subtract(air.velocity()));
        Vec3d rate = step.partDirection(step.angularVelocity());
        BodyAerodynamics.Loads l = aero.loads(velocity, rate, new Wing.Air(air.densityKgPerM3(), air.dynamicViscosityPaS(), air.speedOfSoundMps()));
        if (l.airspeedMps() > 0) {
            Vec3d force = step.axis(l.force()), origin = step.offset(Vec3d.ZERO);
            step.load(new Wrench(force, step.axis(l.moment()).add(origin.cross(force))), AppliedForce.Kind.DRAG);
            Vec3d c = new Vec3d(shape.particulars().centroidXM(), shape.particulars().centroidYM(), 0);
            step.display(new AppliedForce(AppliedForce.Kind.DRAG, step.offset(c), force));
        }
        step.diagnostic("airspeedMps", l.airspeedMps());
        step.diagnostic("airDragN", l.dragN());
        step.diagnostic("airLiftN", l.liftN());
        step.diagnostic("airSideN", l.sideN());
        if (kit != null) { kitLoads(step, velocity, air.densityKgPerM3()); }
        // skin temperatures in radiative equilibrium (0.28): the nose and halfway along, for the vehicle mod's warnings
        double airK = air.speedOfSoundMps() * air.speedOfSoundMps() / (1.4 * 287.05287), speed = l.airspeedMps();
        double nose = airK, skin = airK;
        if (speed > 0.8 * air.speedOfSoundMps() && air.densityKgPerM3() > 0) {
            double radius = Math.max(Math.sqrt(shape.particulars().noseAreaM2() / Math.PI), MIN_NOSE_RADIUS);
            nose = AeroHeating.noseTemperature(air.densityKgPerM3(), airK, speed, radius, EMISSIVITY);
            skin = AeroHeating.surfaceTemperature(air.densityKgPerM3(), airK, speed, 0.5 * shape.particulars().lengthM(), EMISSIVITY);
        }
        step.diagnostic("noseTemperatureK", nose);
        step.diagnostic("skinTemperatureK", skin);
    }

    /** How far below the body the kit's floor looks for the ground (m). */
    static final double GROUND_REACH = RoadAero.FADE_TO + 0.5;

    private void kitLoads(PartStep step, Vec3d velocity, double density) {
        var ground = step.environment().ground();
        Vec3d down = Vec3d.Y.scale(-1);
        RoadAero.Loads k = kit.loads(velocity.scale(-1), density, p -> {
            var hit = ground.cast(step.point(p), down, GROUND_REACH);
            return hit == null ? Double.POSITIVE_INFINITY : hit.distanceM();
        });
        if (k.force().equals(Vec3d.ZERO)) { step.diagnostic("kitDownforceN", 0); step.diagnostic("kitDragN", 0); return; }
        Vec3d force = step.axis(k.force()), origin = step.offset(Vec3d.ZERO);
        step.load(new Wrench(force, step.axis(k.moment()).add(origin.cross(force))), AppliedForce.Kind.LIFT);
        step.diagnostic("kitDownforceN", k.downforceN());
        step.diagnostic("kitDragN", k.dragN());
    }

    /** A pointed nose's radius for its heating (no real nose is sharper), and the skin's emissivity (painted or oxidised metal). */
    static final double MIN_NOSE_RADIUS = 0.02, EMISSIVITY = 0.8;
}

package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.Propeller;
import dev.mcmod.aerocore.api.PropellerSolver;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.aero.AirPropeller;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * An air propeller whose hub is at the part origin, pushing along the part's +X when turning positively about it, from
 * AirPropeller's table over blade setting, Mach number and pitch angle (built on the first step or in warmUp).
 * "pitch" (0..1) sets the blade between its fine and coarse stops; a part wrapping it (a constant-speed governor) can
 * give the setting itself. Loads only from the air: the disk's share in water (the immersion of a sphere of its
 * radius) gives nothing (the blades would break). It goes first in a step: surfaces in its race read its thrust.
 */
public final class AirPropellerPart implements Part {
    /** Blade settings tabulated between the stops, and the Mach numbers (resultant at 0.7 R). */
    public static final int SETTINGS = 7;
    public static final double[] MACHS = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    private final PartDesign design;
    private final AirPropeller.Geometry geometry;
    private final double fineDeg, coarseDeg, spinInertia;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private volatile AirPropeller.Table table;

    /** fineDeg, coarseDeg: blade settings (deg added to the design's blade angles) at pitch 0 and 1. */
    public AirPropellerPart(PartDesign design, AirPropeller.Geometry geometry, double fineDeg, double coarseDeg, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.geometry = Objects.requireNonNull(geometry, "geometry");
        if (!(coarseDeg >= fineDeg) || !Double.isFinite(fineDeg + coarseDeg)) { throw new IllegalArgumentException("Coarse stop must not be below the fine stop"); }
        this.fineDeg = fineDeg; this.coarseDeg = coarseDeg;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        this.spinInertia = Math.max(mass.xx(), 1e-9);
    }

    public AirPropeller.Geometry geometry() { return geometry; }
    public double fineDeg() { return fineDeg; }
    public double coarseDeg() { return coarseDeg; }

    /** The coefficient table, built on first use (a client only drawing the propeller never builds it). */
    public AirPropeller.Table table() {
        AirPropeller.Table t = table;
        if (t == null) {
            synchronized (this) {
                t = table;
                if (t == null) {
                    // Reynolds number on the diameter at a 150 m/s resultant at sea level
                    double reynolds = 1.225 * 150 * geometry.diameterM() / 1.789e-5;
                    int settings = coarseDeg > fineDeg ? SETTINGS : 1;
                    table = t = new AirPropeller(geometry).tabulate(new AirPropeller.Grid(Math.toRadians(fineDeg), Math.toRadians(coarseDeg), settings, MACHS, reynolds));
                }
            }
        }
        return t;
    }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, geometry.diameterM() / 2)); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, spinInertia, false)); }
    @Override public List<String> inputs() { return List.of("pitch"); }

    /** 0.30: shown: the blade setting now (rad added to the design's blade angles, 0.001 steps), for drawing the blades. */
    @Override public int shownValues() { return 1; }
    @Override public boolean early() { return true; }
    @Override public void warmUp() { table(); }

    @Override public void loads(PartStep step) {
        double pitch = Math.clamp(step.input("pitch", 0), 0, 1);
        loads(step, Math.toRadians(fineDeg + pitch * (coarseDeg - fineDeg)));
    }

    /** The loads at a blade setting (rad, held between the stops). */
    public void loads(PartStep step, double setting) {
        if (step.shownCount() >= 1) { step.show(0, PartStep.stepped(step.shown(0), setting, 1e-3)); }   // a part wrapping it may not show it
        double s = Math.clamp(setting, Math.toRadians(fineDeg), Math.toRadians(coarseDeg));
        double d = geometry.diameterM(), rps = step.rotorSpeed(0) / (2 * Math.PI);
        Vec3d hub = step.point(Vec3d.ZERO);
        FloatingBodyDynamics.Contact disk = step.environment().sphere(hub, d / 2);
        double share = 1 - disk.fraction();
        FluidState air = disk.air();
        Vec3d axisWorld = step.axis(Vec3d.X);
        double advance = step.velocity(Vec3d.ZERO).subtract(air.velocity()).dot(axisWorld);
        double mach = Math.hypot(advance, 0.7 * Math.PI * rps * d) / air.speedOfSoundMps();
        Propeller propeller = new Propeller(step.vehiclePoint(Vec3d.ZERO), step.mount().rotation().rotate(Vec3d.X), d, table().model(s, mach));
        PropellerSolver.Load load = PropellerSolver.load(step.state(), step.properties(), propeller, rps, air);
        double dn = Math.max(1e-3, 1e-3 * Math.abs(rps));
        double ahead = PropellerSolver.load(step.state(), step.properties(), propeller, rps + dn, air).torqueNm();
        double torque = share * load.torqueNm(), slope = share * (ahead - load.torqueNm()) / (dn * 2 * Math.PI);
        Vec3d force = load.force().force().scale(share);
        if (force.lengthSquared() > 0) { step.force(new AppliedForce(AppliedForce.Kind.THRUST, load.force().offset(), force)); }
        step.rotorTorque(0, -torque, -slope, true);
        step.thrust(0, share * load.thrustN(), d);
        step.diagnostic("rps", rps);
        step.diagnostic("thrustN", share * load.thrustN());
        step.diagnostic("torqueNm", torque);
        step.diagnostic("shaftPowerW", share * load.shaftPowerW());
        step.diagnostic("advanceMps", advance);
        step.diagnostic("bladeSettingDeg", Math.toDegrees(s));
        step.diagnostic("tipMach", Math.hypot(advance, Math.PI * rps * d) / air.speedOfSoundMps());
    }
}

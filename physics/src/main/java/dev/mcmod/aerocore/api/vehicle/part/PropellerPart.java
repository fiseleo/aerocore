package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.Propeller;
import dev.mcmod.aerocore.api.PropellerModel;
import dev.mcmod.aerocore.api.PropellerSolver;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * A propeller whose hub is at the part origin, pushing along the part's +X when turning positively about it, with a
 * four-quadrant model (PropellerModels.fourQuadrant, built on the first step or in warmUp). Loads are blended by the
 * disk's immersion in water and air (PropellerSolver; no ventilation of single blades, no hull wake). The fluid torque
 * acts on the rotor; a motor joined by a shaft must supply it, and the vehicle feels it through the shaft train. It
 * goes first in a step: control surfaces in its race read its thrust.
 */
public final class PropellerPart implements Part {
    private final PartDesign design;
    private final double diameterM, pitchRatio, hubRatio, chordRatio;
    private final int blades;
    private final Supplier<PropellerModel> source;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final double spinInertia;
    private volatile PropellerModel model;

    /** blades, pitch, hub and chord ratios describe the geometry for display; the model supplier builds the four-quadrant table. */
    public PropellerPart(PartDesign design, double diameterM, int blades, double pitchRatio, double hubRatio, double chordRatio,
                         Supplier<PropellerModel> model, MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        if (!(diameterM > 0)) { throw new IllegalArgumentException("Diameter must be positive"); }
        this.diameterM = diameterM; this.blades = blades; this.pitchRatio = pitchRatio; this.hubRatio = hubRatio; this.chordRatio = chordRatio;
        this.source = Objects.requireNonNull(model, "model");
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
        this.spinInertia = Math.max(mass.xx(), 1e-9);
    }

    public double diameterM() { return diameterM; }
    public int blades() { return blades; }
    public double pitchRatio() { return pitchRatio; }
    public double hubRatio() { return hubRatio; }
    public double chordRatio() { return chordRatio; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, diameterM / 2)); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Rotor> rotors() { return List.of(new Rotor(Vec3d.X, spinInertia, false)); }
    @Override public boolean early() { return true; }
    @Override public void warmUp() { model(); }

    /** The four-quadrant table, built on the first step (a client only drawing the propeller never builds it). */
    private PropellerModel model() {
        PropellerModel m = model;
        if (m == null) {
            synchronized (this) {
                m = model;
                if (m == null) { model = m = source.get(); }
            }
        }
        return m;
    }

    @Override public void loads(PartStep step) {
        double d = diameterM, rps = step.rotorSpeed(0) / (2 * Math.PI);
        Vec3d hub = step.point(Vec3d.ZERO);
        FloatingBodyDynamics.Contact disk = step.environment().sphere(hub, d / 2);
        Propeller propeller = new Propeller(step.vehiclePoint(Vec3d.ZERO), step.mount().rotation().rotate(Vec3d.X), d, model());
        PropellerSolver.Load water = PropellerSolver.load(step.state(), step.properties(), propeller, rps, disk.water());
        PropellerSolver.Load air = PropellerSolver.load(step.state(), step.properties(), propeller, rps, disk.air());
        double f = disk.fraction(), g = 1 - f;
        Vec3d force = water.force().force().scale(f).add(air.force().force().scale(g));
        double torque = f * water.torqueNm() + g * air.torqueNm();
        // Torque slope with speed, for the shaft train's implicit step.
        double dn = Math.max(1e-3, 1e-3 * Math.abs(rps));
        double ahead = f * PropellerSolver.load(step.state(), step.properties(), propeller, rps + dn, disk.water()).torqueNm()
                + g * PropellerSolver.load(step.state(), step.properties(), propeller, rps + dn, disk.air()).torqueNm();
        double slope = (ahead - torque) / (dn * 2 * Math.PI);
        if (force.lengthSquared() > 0) { step.force(new AppliedForce(AppliedForce.Kind.THRUST, water.force().offset(), force)); }
        // The fluid turns the rotor with -Q about the axis.
        step.rotorTorque(0, -torque, -slope, true);
        step.thrust(f * water.thrustN(), g * air.thrustN(), d);
        step.diagnostic("rps", rps);
        step.diagnostic("thrustN", f * water.thrustN() + g * air.thrustN());
        step.diagnostic("torqueNm", torque);
        step.diagnostic("advanceMps", f * water.advanceSpeedMps() + g * air.advanceSpeedMps());
        step.diagnostic("shaftPowerW", f * water.shaftPowerW() + g * air.shaftPowerW());
        step.diagnostic("immersion", f);
    }
}

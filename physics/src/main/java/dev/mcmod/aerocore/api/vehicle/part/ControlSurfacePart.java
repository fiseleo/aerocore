package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AirfoilPolar;
import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.Rudder;
import dev.mcmod.aerocore.api.SurfaceForces;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.Wrench;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * An all-moving control surface (rudder, elevator, fin) whose centre of pressure is at the part origin: span along the
 * part's +Y, chord along its +X (leading edge forward), turning about +Y by "deflection" (-1..1) times the maximum
 * deflection; positive moves the trailing edge toward +Z. Loads from AeroSolver through Rudder.load with the given
 * section polar (Rudder.lowAspectPolar against a hull, Rudder.freeEndsPolar free at both ends), blended by immersion
 * in water and air; inside a propeller's race (Connection.Slipstream) the inflow is raised by its slipstream.
 */
public final class ControlSurfacePart implements Part {
    private final PartDesign design;
    private final double chordM, spanM, maxDeflectionDeg, thicknessRatio;
    private final Rudder surface;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    public ControlSurfacePart(PartDesign design, double chordM, double spanM, double maxDeflectionDeg, double thicknessRatio, AirfoilPolar polar,
                              MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.chordM = chordM; this.spanM = spanM; this.maxDeflectionDeg = maxDeflectionDeg; this.thicknessRatio = thicknessRatio;
        this.surface = new Rudder(Vec3d.ZERO, chordM, spanM, polar);
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public double chordM() { return chordM; }
    public double spanM() { return spanM; }
    public double maxDeflectionDeg() { return maxDeflectionDeg; }
    public double thicknessRatio() { return thicknessRatio; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, 0.5 * Math.min(chordM, spanM))); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<String> inputs() { return List.of("deflection"); }

    @Override public void loads(PartStep step) {
        double angle = Math.clamp(step.input("deflection", 0), -1, 1) * Math.toRadians(maxDeflectionDeg);
        FloatingBodyDynamics.Contact contact = step.environment().sphere(step.point(Vec3d.ZERO), spanM / 2);
        double f = contact.fraction();
        double[] race = step.slipstream();
        double waterThrust = race == null ? 0 : race[0], airThrust = race == null ? 0 : race[1], diameter = race == null ? 0 : race[2];
        Wrench total = Wrench.ZERO;
        if (f > 0) {
            SurfaceForces w = surface.load(step.partState(), step.partProperties(), angle, contact.water(), waterThrust, diameter);
            total = total.add(scale(w.wrench(), f));
        }
        if (f < 1) {
            SurfaceForces a = surface.load(step.partState(), step.partProperties(), angle, contact.air(), airThrust, diameter);
            total = total.add(scale(a.wrench(), 1 - f));
        }
        if (total.force().lengthSquared() > 0 || total.torque().lengthSquared() > 0) {
            step.load(total, AppliedForce.Kind.LIFT);
            step.display(new AppliedForce(AppliedForce.Kind.LIFT, step.offset(Vec3d.ZERO), total.force()));
        }
        step.diagnostic("deflectionDeg", Math.toDegrees(angle));
        step.diagnostic("forceN", total.force().length());
        step.diagnostic("immersion", f);
    }

    private static Wrench scale(Wrench w, double k) { return new Wrench(w.force().scale(k), w.torque().scale(k)); }
}

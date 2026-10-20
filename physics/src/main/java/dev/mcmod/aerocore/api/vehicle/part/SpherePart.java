package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.BuoyantSphere;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FloatingBodyDynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WaveRadiation;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.List;
import java.util.Objects;

/**
 * A spherical float (part origin at its centre), as one float of FloatingBodyDynamics: exact cap buoyancy in air and
 * water with the wave pressure, smooth-sphere standard drag (Clift et al.), added mass C_a rho V_wet with the Morison
 * excitation, exact hemisphere heave diffraction when it pierces a wavy surface, and heave radiation as a point
 * absorber of its waterplane radius (WaveRadiation, together with every other part's points).
 */
public final class SpherePart implements Part {
    private final PartDesign design;
    private final double radiusM;
    private final MassProperties mass;

    public SpherePart(PartDesign design, double radiusM, MassProperties mass) {
        this.design = Objects.requireNonNull(design, "design");
        if (!(radiusM > 0) || !Double.isFinite(radiusM)) { throw new IllegalArgumentException("Radius must be positive"); }
        this.radiusM = radiusM;
        this.mass = Objects.requireNonNull(mass, "mass");
    }

    public double radiusM() { return radiusM; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return List.of(new CollisionSphere(Vec3d.ZERO, radiusM)); }
    @Override public List<DesignIssue> issues() { return List.of(); }
    @Override public List<Vec3d> radiationPoints() { return List.of(Vec3d.ZERO); }

    @Override public void loads(PartStep step) {
        double r = radiusM;
        Vec3d centre = step.point(Vec3d.ZERO);
        FloatingBodyDynamics.Contact contact = step.environment().sphere(centre, r);
        FloatingBodyDynamics.SphereLoads loads = FloatingBodyDynamics.sphereLoads(step.state(), step.properties(), step.angularVelocity(),
                new BuoyantSphere(step.vehiclePoint(Vec3d.ZERO), r), contact, step.gravity(), FloatingBodyDynamics.STANDARD_DRAG);
        step.force(loads.buoyancy());
        step.pointDrag(loads.airDrag());
        step.pointDrag(loads.waterDrag());
        if (loads.addedMass() != null) { step.pointMass(loads.addedMass(), loads.excitation()); }
        // The exact heave diffraction of a surface-piercing hemisphere, paired with the contact leaving out the Morison term.
        double last = step.lastRadiationRadius(0);
        if (contact.surfaceDiffraction() && last > 0 && !step.environment().waves().isEmpty()) {
            double f = WaveRadiation.heaveDiffraction(step.environment().waves(), centre.x(), centre.z(), step.time(), last, contact.water().densityKgPerM3());
            step.force(new AppliedForce(AppliedForce.Kind.INERTIA, step.offset(Vec3d.ZERO), Vec3d.Y.scale(f)));
        }
        step.radiationRadius(0, WaveRadiation.waterplaneRadius(r, loads.wetFraction()), 1);
        step.diagnostic("wetFraction", loads.wetFraction());
    }
}

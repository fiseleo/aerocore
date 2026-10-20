package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.PlaningDynamics;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;
import dev.mcmod.aerocore.api.vehicle.WakeSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A hard-chine planing hull (PlaningHull, transom at the part origin): PlaningDynamics gives its loads (Zarnick 2D+t
 * with Wagner wetting, Garme's near-transom correction, ITTC friction, whisker spray); heave radiation from
 * equal-length segments while the transom is wet, faded out with its ventilation. Contact spheres along the keel,
 * chines and deck edges; a pressure-patch wake.
 */
public final class PlaningHullPart implements Part {
    private final PartDesign design;
    private final PlaningHull planing;
    private final double chineBeamM, deadriseDeg, depthM;
    private final int radiationSegments;
    private final MassProperties mass;
    private final List<DesignIssue> issues;

    public PlaningHullPart(PartDesign design, PlaningHull planing, double chineBeamM, double deadriseDeg, double depthM, int radiationSegments,
                           MassProperties mass, List<DesignIssue> issues) {
        this.design = Objects.requireNonNull(design, "design");
        this.planing = Objects.requireNonNull(planing, "planing");
        this.chineBeamM = chineBeamM; this.deadriseDeg = deadriseDeg; this.depthM = depthM;
        if (radiationSegments < 1) { throw new IllegalArgumentException("Radiation segments must be at least 1"); }
        this.radiationSegments = radiationSegments;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public PlaningHull planing() { return planing; }
    public double chineBeamM() { return chineBeamM; }
    public double deadriseDeg() { return deadriseDeg; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Vec3d> radiationPoints() { return HullDynamics.radiationPoints(planing.hull(), radiationSegments); }
    @Override public List<WorldBox> deck() { return HullPart.deckOf(planing.hull()); }
    @Override public WakeSource wake() { return new WakeSource.Planing(chineBeamM, Math.toRadians(deadriseDeg), "wettedLengthM"); }

    /** Spheres along the keel, both chines and both deck edges at every fourth station. */
    @Override public List<CollisionSphere> collision() {
        List<CollisionSphere> shape = new ArrayList<>();
        SlenderHull hull = planing.hull();
        double tan = Math.tan(Math.toRadians(deadriseDeg));
        for (int i = 0; i < hull.stations(); i += 4) {
            double x = hull.sections().get(i).xM(), b = planing.chineHalfBeam(i), keel = planing.keelY(i);
            double r = Math.max(0.04, Math.min(0.12, b / 3)), chineY = Math.min(keel + b * tan, depthM - r);
            shape.add(new CollisionSphere(new Vec3d(x, keel + r, 0), r));
            for (int side = -1; side <= 1; side += 2) {
                shape.add(new CollisionSphere(new Vec3d(x, chineY + r * 0.5, side * Math.max(0, b - r)), r));
                shape.add(new CollisionSphere(new Vec3d(x, depthM - r, side * Math.max(0, b - r)), r));
            }
        }
        return shape;
    }

    @Override public void loads(PartStep step) {
        PlaningDynamics.Loads loads = PlaningDynamics.loads(step.partState(), step.partProperties(), planing, step.environment()::point, step.gravity(), false);
        step.load(loads.hydro(), AppliedForce.Kind.BUOYANCY);
        step.load(loads.dynamic(), AppliedForce.Kind.LIFT);
        step.load(loads.friction(), AppliedForce.Kind.DRAG);
        for (AppliedForce f : loads.forces()) { step.display(f); }
        step.inertia(loads.excitation());
        step.addedMass(HullDynamics.rotate(loads.addedBody(), step.partState().orientation()));
        PlaningDynamics.Diagnostics d = loads.diagnostics();
        // Zero-speed heave radiation while the transom is wet; the 2D+t loads take over as it ventilates.
        double[] radii = HullDynamics.segmentRadii(planing.hull(), radiationSegments, d.waterlineBreadth());
        for (int k = 0; k < radii.length; k++) { step.radiationRadius(k, radii[k], 1 - d.ventilation()); }
        step.diagnostic("wettedLengthM", d.wettedLengthM());
        step.diagnostic("surgeMps", d.surgeMps());
        step.diagnostic("frictionN", d.frictionN());
        step.diagnostic("ventilation", d.ventilation());
        step.diagnostic("wetVolumeM3", d.wetVolumeM3());
        step.diagnostic("dragPowerW", d.dragPowerW());
    }
}

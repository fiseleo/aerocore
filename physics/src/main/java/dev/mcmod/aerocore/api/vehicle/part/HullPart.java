package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.FluidState;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.HullModel;
import dev.mcmod.aerocore.api.HullSection;
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
 * A displacement hull (or a submerged body) of SlenderHull sections in the part frame. HullDynamics gives its loads:
 * strip buoyancy on the local surface, ITTC-1957 friction with Holtrop-Mennen form factor plus Michell wave resistance
 * (surface hulls), strip added mass with the ideal-fluid terms, the Clarke-calibrated lift element and Hoerner
 * cross-flow drag; heave radiation from equal-length segments as point absorbers. The resistance and manoeuvring
 * tables (HullModel, about a second for the Michell table) are built on the first step or in warmUp. A surface hull
 * makes a Michell thin-ship wake at its design displacement.
 */
public final class HullPart implements Part {
    private final PartDesign design;
    private final SlenderHull hull;
    private final HullModel.Type type;
    private final double designDisplacementKg, sternShape;
    private final int radiationSegments;
    private final MassProperties mass;
    private final List<DesignIssue> issues;
    private final boolean windage;
    private volatile HullModel model;

    public HullPart(PartDesign design, SlenderHull hull, HullModel.Type type, double designDisplacementKg, double sternShape, int radiationSegments,
                    MassProperties mass, List<DesignIssue> issues) {
        this(design, hull, type, designDisplacementKg, sternShape, radiationSegments, mass, issues, true);
    }

    /** windage: whether the dry part takes its own wind loads (false inside a part that has its own air loads, BodyPart). */
    public HullPart(PartDesign design, SlenderHull hull, HullModel.Type type, double designDisplacementKg, double sternShape, int radiationSegments,
                    MassProperties mass, List<DesignIssue> issues, boolean windage) {
        this.windage = windage;
        this.design = Objects.requireNonNull(design, "design");
        this.hull = Objects.requireNonNull(hull, "hull");
        this.type = Objects.requireNonNull(type, "type");
        this.designDisplacementKg = designDisplacementKg;
        this.sternShape = sternShape;
        if (radiationSegments < 1) { throw new IllegalArgumentException("Radiation segments must be at least 1"); }
        this.radiationSegments = radiationSegments;
        this.mass = Objects.requireNonNull(mass, "mass");
        this.issues = List.copyOf(issues);
    }

    public SlenderHull hull() { return hull; }
    public HullModel.Type type() { return type; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return HullModel.collisionShape(hull); }
    @Override public List<DesignIssue> issues() { return issues; }
    @Override public List<Vec3d> radiationPoints() { return HullDynamics.radiationPoints(hull, radiationSegments); }
    @Override public void warmUp() { model(); }

    @Override public WakeSource wake() {
        return type == HullModel.Type.SURFACE ? new WakeSource.ThinShip(hull, designDisplacementKg / FluidState.FRESH_WATER_20C.densityKgPerM3()) : null;
    }

    /** The resistance and manoeuvring coefficients, built when first needed: a client only drawing the hull never builds them. */
    private HullModel model() {
        HullModel m = model;
        if (m == null) {
            synchronized (this) {
                m = model;
                if (m == null) { model = m = new HullModel(hull, type, designDisplacementKg, FluidState.FRESH_WATER_20C, sternShape); }
            }
        }
        return m;
    }

    @Override public List<WorldBox> deck() { return deckOf(hull); }

    /** Standing boxes over every strip of a hull, in its frame. */
    static List<WorldBox> deckOf(SlenderHull hull) {
        List<WorldBox> boxes = new ArrayList<>();
        for (int i = 0; i < hull.stations(); i++) {
            HullSection s = hull.sections().get(i);
            double half = 0.5 * hull.stripLength(i), width = s.maxHalfBreadth();
            boxes.add(new WorldBox(new Vec3d(s.xM() - half, s.minY(), -width), new Vec3d(s.xM() + half, s.maxY(), width)));
        }
        return boxes;
    }

    @Override public void loads(PartStep step) {
        HullDynamics.Loads loads = HullDynamics.loads(step.partState(), step.partProperties(), model(), step.environment()::point, step.gravity(), false, windage);
        for (var f : loads.forces()) { step.force(f); }
        step.inertia(loads.excitation().add(loads.kirchhoff()));
        step.addedMass(HullDynamics.rotate(loads.addedBody(), step.partState().orientation()));
        double[] radii = HullDynamics.segmentRadii(hull, radiationSegments, loads.breadth());
        for (int k = 0; k < radii.length; k++) { step.radiationRadius(k, radii[k], 1); }
        step.diagnostic("wetVolumeM3", loads.wetVolume());
        step.diagnostic("surgeMps", loads.surge());
        step.diagnostic("resistanceN", loads.resistance());
        step.diagnostic("dragPowerW", loads.dragPower());
    }
}

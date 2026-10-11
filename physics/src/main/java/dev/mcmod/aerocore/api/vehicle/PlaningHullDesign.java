package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.HullSection;
import dev.mcmod.aerocore.api.PlaningDynamics;
import dev.mcmod.aerocore.api.PlaningHull;
import dev.mcmod.aerocore.api.SlenderHull;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A hard-chine planing hull of Fridsma's (1969) series shape (PlaningHull.fridsma): transom at the part origin, keel of
 * the prismatic part at y = 0, length L forward, chine beam B, deadrise beta, depth D. PlaningDynamics gives its loads
 * (Zarnick 2D+t with Wagner wetting, Garme's near-transom correction, ITTC friction, whisker spray), validated against
 * Fridsma's tank tests at 20 deg deadrise; heave radiation from equal-length segments while the transom is wet, faded
 * out with its ventilation. Fridsma tested deadrise 10, 20 and 30 deg and L/B 4 to 7; 2D+t needs speed coefficients
 * C_V above about 2 (it under-predicts trim and resistance before planing). Mass: a thin shell, or a given mass.
 */
public record PlaningHullDesign(String name, double lengthM, double chineBeamM, double deadriseDeg, double depthM, int stations,
                                int radiationSegments, MassSpec mass) implements PartDesign {
    public static final String TYPE = "aerocore:planing_hull";
    private static final String FRIDSMA = "Fridsma 1969 tank series";

    public PlaningHullDesign {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(mass, "mass");
    }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        MotorDesign.positive(issues, "lengthM", lengthM);
        MotorDesign.positive(issues, "chineBeamM", chineBeamM);
        MotorDesign.positive(issues, "depthM", depthM);
        if (!(deadriseDeg > 0 && deadriseDeg < 45)) { issues.add(DesignIssue.invalid("deadriseDeg", "Deadrise must be in (0, 45) degrees")); }
        if (lengthM > 0 && chineBeamM > 0 && !(lengthM > 1.5 * chineBeamM)) { issues.add(DesignIssue.invalid("lengthM", "Length must exceed 1.5 beams (the bow takes one beam)")); }
        if (stations < 8 || stations > 200) { issues.add(DesignIssue.invalid("stations", "Use 8 to 200 stations")); }
        if (radiationSegments < 1 || radiationSegments > 16) { issues.add(DesignIssue.invalid("radiationSegments", "Radiation segments must be 1 to 16")); }
        mass.check(issues, "mass", true);
        if (issues.isEmpty()) {
            DesignIssue.range(issues, "deadriseDeg", deadriseDeg, 10, 30, FRIDSMA);
            DesignIssue.range(issues, "lengthBeamRatio", lengthM / chineBeamM, 4, 7, FRIDSMA);
        }
        return issues;
    }

    @Override public Part compile() {
        List<DesignIssue> issues = check();
        DesignException.throwIfInvalid(issues);
        return new Compiled(this, PlaningHull.fridsma(lengthM, chineBeamM, Math.toRadians(deadriseDeg), depthM, stations), issues);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("lengthM", lengthM, "chineBeamM", chineBeamM, "deadriseDeg", deadriseDeg, "depthM", depthM, "stations", stations,
                "radiationSegments", radiationSegments, "mass", mass.toJson());
    }

    static PartDesign read(Fields f) {
        return new PlaningHullDesign(f.string("name", "planing hull"), f.number("lengthM"), f.number("chineBeamM"), f.number("deadriseDeg"),
                f.number("depthM"), f.integer("stations", 48), f.integer("radiationSegments", 6), MassSpec.fromJson(f.child("mass")));
    }

    private static final class Compiled implements Part {
        private final PlaningHullDesign design;
        private final PlaningHull planing;
        private final List<DesignIssue> issues;
        private final MassProperties mass;

        Compiled(PlaningHullDesign design, PlaningHull planing, List<DesignIssue> issues) {
            this.design = design; this.planing = planing;
            this.issues = issues.stream().filter(i -> i.kind() == DesignIssue.Kind.EXTRAPOLATION).toList();
            this.mass = design.mass.resolve(t -> Shapes.hullShell(planing.hull(), Double.isFinite(t) ? t : 1e-3));
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<DesignIssue> issues() { return issues; }
        @Override public List<Vec3d> radiationPoints() { return HullDynamics.radiationPoints(planing.hull(), design.radiationSegments); }

        /** Spheres along the keel, both chines and both deck edges at every fourth station. */
        @Override public List<CollisionSphere> collision() {
            List<CollisionSphere> shape = new ArrayList<>();
            SlenderHull hull = planing.hull();
            double depth = design.depthM, tan = Math.tan(Math.toRadians(design.deadriseDeg));
            for (int i = 0; i < hull.stations(); i += 4) {
                double x = hull.sections().get(i).xM(), b = planing.chineHalfBeam(i), keel = planing.keelY(i);
                double r = Math.max(0.04, Math.min(0.12, b / 3)), chineY = Math.min(keel + b * tan, depth - r);
                shape.add(new CollisionSphere(new Vec3d(x, keel + r, 0), r));
                for (int side = -1; side <= 1; side += 2) {
                    shape.add(new CollisionSphere(new Vec3d(x, chineY + r * 0.5, side * Math.max(0, b - r)), r));
                    shape.add(new CollisionSphere(new Vec3d(x, depth - r, side * Math.max(0, b - r)), r));
                }
            }
            return shape;
        }

        @Override public List<WorldBox> deck() {
            List<WorldBox> boxes = new ArrayList<>();
            SlenderHull hull = planing.hull();
            for (int i = 0; i < hull.stations(); i++) {
                HullSection s = hull.sections().get(i);
                double half = 0.5 * hull.stripLength(i), width = s.maxHalfBreadth();
                boxes.add(new WorldBox(new Vec3d(s.xM() - half, s.minY(), -width), new Vec3d(s.xM() + half, s.maxY(), width)));
            }
            return boxes;
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
            double[] radii = HullDynamics.segmentRadii(planing.hull(), design.radiationSegments, d.waterlineBreadth());
            for (int k = 0; k < radii.length; k++) { step.radiationRadius(k, radii[k], 1 - d.ventilation()); }
            step.diagnostic("wettedLengthM", d.wettedLengthM());
            step.diagnostic("surgeMps", d.surgeMps());
            step.diagnostic("frictionN", d.frictionN());
            step.diagnostic("ventilation", d.ventilation());
            step.diagnostic("wetVolumeM3", d.wetVolumeM3());
            step.diagnostic("dragPowerW", d.dragPowerW());
        }
    }
}

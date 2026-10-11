package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A box-shaped body (cabin, fuselage, pontoon, chassis) centred on the part origin, sides along the part axes.
 * Split into cells^3 equal cells, each sampled at its centre:
 * <ul>
 * <li>Buoyancy when closed: air and water by the cell's wet fraction, the water depth over the cell's height measured
 *     vertically through the cell centre (exact for a level box in still water as cells grow fine).</li>
 * <li>Drag: -1/2 rho C_D A_p(u) |u| u with A_p(u) = sum A_i |u . e_i|, the box's projected area across the relative
 *     flow, shared equally by the cells (so turning is damped too); C_D 1.05 is a cube face-on (Hoerner 1965,
 *     Fluid-Dynamic Drag 3-16). Taken implicitly at the step's start speed, as sphere drag.</li>
 * </ul>
 * Not modelled: added mass, wave radiation and Froude-Krylov beyond the local hydrostatic pressure (a box pontoon
 * in waves is under-damped), lift. Mass: a hollow box of the given wall (outer minus inner box), or solid.
 */
public record BoxDesign(String name, Vec3d sizeM, double dragCoefficient, boolean closed, int cells, MassSpec mass) implements PartDesign {
    public static final String TYPE = "aerocore:box";

    public BoxDesign {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(sizeM, "sizeM");
        Objects.requireNonNull(mass, "mass");
    }

    @Override public String type() { return TYPE; }

    @Override public List<DesignIssue> check() {
        List<DesignIssue> issues = new ArrayList<>();
        if (!(sizeM.x() > 0 && sizeM.y() > 0 && sizeM.z() > 0) || !Double.isFinite(sizeM.x() + sizeM.y() + sizeM.z())) {
            issues.add(DesignIssue.invalid("sizeM", "Every side must be positive"));
        }
        if (!(dragCoefficient >= 0) || !Double.isFinite(dragCoefficient)) { issues.add(DesignIssue.invalid("dragCoefficient", "Drag coefficient must be zero or positive")); }
        if (cells < 1 || cells > 6) { issues.add(DesignIssue.invalid("cells", "Cells per side must be 1 to 6")); }
        mass.check(issues, "mass", false);
        return issues;
    }

    @Override public Part compile() {
        DesignException.throwIfInvalid(check());
        return new Compiled(this);
    }

    @Override public Map<String, Object> parameters() {
        return Fields.object("sizeM", Fields.vector(sizeM), "dragCoefficient", dragCoefficient, "closed", closed, "cells", cells, "mass", mass.toJson());
    }

    static PartDesign read(Fields f) {
        return new BoxDesign(f.string("name", "box"), f.vec("sizeM"), f.number("dragCoefficient", 1.05), f.bool("closed", true),
                f.integer("cells", 2), MassSpec.fromJson(f.child("mass")));
    }

    private static final class Compiled implements Part {
        private final BoxDesign design;
        private final MassProperties mass;
        private final List<Vec3d> cells = new ArrayList<>();

        Compiled(BoxDesign design) {
            this.design = design;
            this.mass = design.mass.resolve(t -> Shapes.box(design.sizeM, t));
            int n = design.cells;
            Vec3d s = design.sizeM;
            for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) for (int k = 0; k < n; k++) {
                cells.add(new Vec3d(s.x() * ((i + 0.5) / n - 0.5), s.y() * ((j + 0.5) / n - 0.5), s.z() * ((k + 0.5) / n - 0.5)));
            }
        }

        @Override public PartDesign design() { return design; }
        @Override public MassProperties mass() { return mass; }
        @Override public List<CollisionSphere> collision() { return Shapes.boxSpheres(design.sizeM); }
        @Override public List<DesignIssue> issues() { return List.of(); }
        @Override public List<WorldBox> deck() { return List.of(new WorldBox(design.sizeM.scale(-0.5), design.sizeM.scale(0.5))); }

        @Override public void loads(PartStep step) {
            Vec3d s = design.sizeM;
            int n = design.cells, count = cells.size();
            Vec3d ex = step.axis(Vec3d.X), ey = step.axis(Vec3d.Y), ez = step.axis(Vec3d.Z);
            double cellVolume = s.x() * s.y() * s.z() / count;
            // A cell's vertical extent, as its sides lean.
            double height = (Math.abs(ex.y()) * s.x() + Math.abs(ey.y()) * s.y() + Math.abs(ez.y()) * s.z()) / n;
            double g = step.gravity().length();
            Vec3d up = g > 0 ? step.gravity().scale(-1 / g) : Vec3d.Y;
            double submerged = 0;
            for (Vec3d cell : cells) {
                Vec3d r = step.offset(cell), at = step.state().position().add(r);
                HullDynamics.Water water = step.environment().point(at);
                double f = water.hasWater() ? Math.clamp((water.surfaceY() - (at.y() - height / 2)) / height, 0, 1) : 0;
                double rhoWater = water.water().densityKgPerM3(), rhoAir = water.air().densityKgPerM3();
                submerged += f * cellVolume;
                if (design.closed) {
                    Vec3d buoyancy = up.scale(g * cellVolume * (rhoWater * f + rhoAir * (1 - f)));
                    step.force(new AppliedForce(AppliedForce.Kind.BUOYANCY, r, buoyancy));
                }
                // Drag on the projected area across the flow, each medium by its share of the cell.
                Vec3d velocity = step.state().velocity().add(step.angularVelocity().cross(r));
                drag(step, r, velocity, water.water().velocity(), rhoWater * f, ex, ey, ez, count);
                drag(step, r, velocity, water.air().velocity(), rhoAir * (1 - f), ex, ey, ez, count);
            }
            step.diagnostic("submergedM3", submerged);
        }

        private void drag(PartStep step, Vec3d r, Vec3d velocity, Vec3d flow, double density, Vec3d ex, Vec3d ey, Vec3d ez, int count) {
            if (density <= 0 || design.dragCoefficient == 0) { return; }
            Vec3d u = velocity.subtract(flow);
            double speed = u.length();
            if (speed == 0) { return; }
            Vec3d s = design.sizeM, d = u.scale(1 / speed);
            double projected = s.y() * s.z() * Math.abs(d.dot(ex)) + s.x() * s.z() * Math.abs(d.dot(ey)) + s.x() * s.y() * Math.abs(d.dot(ez));
            double resistance = 0.5 * density * design.dragCoefficient * projected / count * speed;
            step.pointDrag(new RigidBodyDynamics.PointDrag(r, flow, resistance, 0));
        }
    }
}

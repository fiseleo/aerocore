package dev.mcmod.aerocore.api.vehicle.part;

import dev.mcmod.aerocore.api.AppliedForce;
import dev.mcmod.aerocore.api.CollisionSphere;
import dev.mcmod.aerocore.api.HullDynamics;
import dev.mcmod.aerocore.api.RigidBodyDynamics;
import dev.mcmod.aerocore.api.Vec3d;
import dev.mcmod.aerocore.api.WorldBox;
import dev.mcmod.aerocore.api.vehicle.DesignIssue;
import dev.mcmod.aerocore.api.vehicle.MassProperties;
import dev.mcmod.aerocore.api.vehicle.MassShapes;
import dev.mcmod.aerocore.api.vehicle.Part;
import dev.mcmod.aerocore.api.vehicle.PartDesign;
import dev.mcmod.aerocore.api.vehicle.PartStep;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A rectangular box centred at the part origin (a deck, a cabin, a pontoon), split into cells^3 cells: each cell is
 * buoyant (if closed) by its wetted fraction of height in water and air, and drags with a drag coefficient on its share
 * of the box's area projected across the flow, each medium by its share. People can stand on it.
 */
public final class BoxPart implements Part {
    private final PartDesign design;
    private final Vec3d sizeM;
    private final double dragCoefficient;
    private final boolean closed;
    private final int cells;
    private final MassProperties mass;
    private final List<Vec3d> centres = new ArrayList<>();

    public BoxPart(PartDesign design, Vec3d sizeM, double dragCoefficient, boolean closed, int cells, MassProperties mass) {
        this.design = Objects.requireNonNull(design, "design");
        this.sizeM = Objects.requireNonNull(sizeM, "sizeM");
        if (cells < 1) { throw new IllegalArgumentException("A box needs at least one cell"); }
        this.dragCoefficient = dragCoefficient; this.closed = closed; this.cells = cells;
        this.mass = Objects.requireNonNull(mass, "mass");
        for (int i = 0; i < cells; i++) for (int j = 0; j < cells; j++) for (int k = 0; k < cells; k++) {
            centres.add(new Vec3d(sizeM.x() * ((i + 0.5) / cells - 0.5), sizeM.y() * ((j + 0.5) / cells - 0.5), sizeM.z() * ((k + 0.5) / cells - 0.5)));
        }
    }

    public Vec3d sizeM() { return sizeM; }

    @Override public PartDesign design() { return design; }
    @Override public MassProperties mass() { return mass; }
    @Override public List<CollisionSphere> collision() { return MassShapes.boxSpheres(sizeM); }
    @Override public List<DesignIssue> issues() { return List.of(); }
    @Override public List<WorldBox> deck() { return List.of(new WorldBox(sizeM.scale(-0.5), sizeM.scale(0.5))); }

    @Override public void loads(PartStep step) {
        Vec3d s = sizeM;
        int n = cells, count = centres.size();
        Vec3d ex = step.axis(Vec3d.X), ey = step.axis(Vec3d.Y), ez = step.axis(Vec3d.Z);
        double cellVolume = s.x() * s.y() * s.z() / count;
        // A cell's vertical extent, as its sides lean.
        double height = (Math.abs(ex.y()) * s.x() + Math.abs(ey.y()) * s.y() + Math.abs(ez.y()) * s.z()) / n;
        double g = step.gravity().length();
        Vec3d up = g > 0 ? step.gravity().scale(-1 / g) : Vec3d.Y;
        double submerged = 0;
        for (Vec3d cell : centres) {
            Vec3d r = step.offset(cell), at = step.state().position().add(r);
            HullDynamics.Water water = step.environment().point(at);
            double f = water.hasWater() ? Math.clamp((water.surfaceY() - (at.y() - height / 2)) / height, 0, 1) : 0;
            double rhoWater = water.water().densityKgPerM3(), rhoAir = water.air().densityKgPerM3();
            submerged += f * cellVolume;
            if (closed) {
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
        if (density <= 0 || dragCoefficient == 0) { return; }
        Vec3d u = velocity.subtract(flow);
        double speed = u.length();
        if (speed == 0) { return; }
        Vec3d s = sizeM, d = u.scale(1 / speed);
        double projected = s.y() * s.z() * Math.abs(d.dot(ex)) + s.x() * s.z() * Math.abs(d.dot(ey)) + s.x() * s.y() * Math.abs(d.dot(ez));
        double resistance = 0.5 * density * dragCoefficient * projected / count * speed;
        step.pointDrag(new RigidBodyDynamics.PointDrag(r, flow, resistance, 0));
    }
}

package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;

/**
 * 0.42: a unit cell of the world whose top is a plane (a road ramp block): solid from its floor y0 up to
 * y0 + h0 + gx (x - x0) + gz (z - z0) over its footprint [x0, x0 + 1] x [z0, z0 + 1]. h0 is the top's height above the
 * floor at the corner (x0, z0), gx and gz its rise per metre along +x and +z.
 *
 * <p>A block's collision shape can only stand for a slope as steps, and a wheel's ray on steps meets treads and risers,
 * never a tilted face: the normal force stays vertical and each riser is a small kerb. Offline a car coasting up steps
 * of a 1:4 run loses far more than the height's energy (from 8 m/s it leaves at 2.1 m/s against 5.3 on the slope; hdfv
 * 0.25's RampDriveTest). The game's ground (WorldGround) asks such blocks for this plane instead and keeps the shape for
 * the cell's sides.
 */
public record SlopedCell(double x0, double y0, double z0, double h0, double gx, double gz) {
    private static final double EDGE = 1e-9;

    public SlopedCell {
        if (!Double.isFinite(x0) || !Double.isFinite(y0) || !Double.isFinite(z0) || !Double.isFinite(h0) || !Double.isFinite(gx) || !Double.isFinite(gz)) {
            throw new IllegalArgumentException("A sloped cell's numbers must be finite");
        }
    }

    /** The top's unit normal, out of the ground. */
    public Vec3d normal() { return new Vec3d(-gx, 1, -gz).normalized(); }

    /** The top's height (world y) over a point of the footprint. */
    public double height(double x, double z) { return y0 + h0 + gx * (x - x0) + gz * (z - z0); }

    private boolean over(double x, double z) { return x >= x0 - EDGE && x <= x0 + 1 + EDGE && z >= z0 - EDGE && z <= z0 + 1 + EDGE; }

    /**
     * Where a ray (a unit direction, at most maxM) meets the sloped top from above, or null when it does not within the
     * footprint and the cell's height: it may still meet the cell's sides, which are the caller's (the block's shape).
     * A ray that starts in the solid under the top hits at once (distance 0).
     */
    public Ground.Hit cast(Vec3d from, Vec3d direction, double maxM, GroundContact.Surface surface) {
        Vec3d n = normal();
        double above = n.dot(from.subtract(new Vec3d(x0, y0 + h0, z0)));
        if (above <= 0) { return over(from.x(), from.z()) && from.y() >= y0 - EDGE ? new Ground.Hit(from, n, 0, surface) : null; }
        double closing = -n.dot(direction);
        if (!(closing > 0)) { return null; }
        double t = above / closing;
        if (!(t <= maxM)) { return null; }
        Vec3d p = from.add(direction.scale(t));
        if (!over(p.x(), p.z()) || p.y() < y0 - EDGE || p.y() > y0 + 1 + EDGE) { return null; }
        return new Ground.Hit(p, n, t, surface);
    }
}

package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;
import java.util.Objects;

/**
 * Ground made of axis-aligned boxes (0.24), for offline tracks: steps, kerbs, ramps built of small steps like the
 * game's road ramp blocks. Rays are tested against every box by slabs (fine for the tens of boxes of a test track); the
 * normal is the face the ray entered, and a ray starting inside a box hits at once on the face nearest its start.
 */
public final class BoxGround implements Ground {
    /** One box: corners (world, min <= max) and surface. */
    public record Box(Vec3d min, Vec3d max, GroundContact.Surface surface) {
        public Box {
            Objects.requireNonNull(min, "min"); Objects.requireNonNull(max, "max"); Objects.requireNonNull(surface, "surface");
            if (!(min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z())) { throw new IllegalArgumentException("Box minimum must not exceed maximum"); }
        }
    }

    private final List<Box> boxes;

    public BoxGround(List<Box> boxes) { this.boxes = List.copyOf(boxes); }

    public List<Box> boxes() { return boxes; }

    /**
     * A staircase along +x from x0 rising rise per step of run (both m), steps of them, across z from z0 to z1, on
     * a floor at y0: the ramp of a road ramp block is this with 1/16 m steps.
     */
    public static List<Box> stairs(double x0, double y0, double z0, double z1, double run, double rise, int steps, GroundContact.Surface surface) {
        java.util.ArrayList<Box> out = new java.util.ArrayList<>();
        for (int i = 0; i < steps; i++) {
            out.add(new Box(new Vec3d(x0 + i * run, y0, z0), new Vec3d(x0 + steps * run, y0 + (i + 1) * rise, z1), surface));
        }
        return out;
    }

    @Override public Hit cast(Vec3d from, Vec3d direction, double maxM) {
        Hit best = null;
        for (Box b : boxes) {
            Hit hit = cast(b, from, direction, best == null ? maxM : best.distanceM());
            if (hit != null && (best == null || hit.distanceM() < best.distanceM())) { best = hit; }
        }
        return best;
    }

    private static Hit cast(Box b, Vec3d from, Vec3d d, double maxM) {
        double[] o = {from.x(), from.y(), from.z()}, v = {d.x(), d.y(), d.z()};
        double[] lo = {b.min().x(), b.min().y(), b.min().z()}, hi = {b.max().x(), b.max().y(), b.max().z()};
        double enter = Double.NEGATIVE_INFINITY, exit = Double.POSITIVE_INFINITY;
        int axis = -1;
        for (int k = 0; k < 3; k++) {
            if (v[k] == 0) {
                if (o[k] < lo[k] || o[k] > hi[k]) { return null; }
                continue;
            }
            double t1 = (lo[k] - o[k]) / v[k], t2 = (hi[k] - o[k]) / v[k];
            double near = Math.min(t1, t2), far = Math.max(t1, t2);
            if (near > enter) { enter = near; axis = k; }
            exit = Math.min(exit, far);
        }
        if (enter > exit || exit < 0 || enter > maxM) { return null; }
        if (enter <= 0) {
            // starting inside: the face nearest the start
            int face = 0;
            double nearest = Double.POSITIVE_INFINITY, sign = 1;
            for (int k = 0; k < 3; k++) {
                if (o[k] - lo[k] < nearest) { nearest = o[k] - lo[k]; face = k; sign = -1; }
                if (hi[k] - o[k] < nearest) { nearest = hi[k] - o[k]; face = k; sign = 1; }
            }
            return new Hit(from, unit(face, sign), 0, b.surface());
        }
        return new Hit(from.add(d.scale(enter)), unit(axis, -Math.signum(v[axis])), enter, b.surface());
    }

    private static Vec3d unit(int axis, double sign) {
        return switch (axis) { case 0 -> new Vec3d(sign, 0, 0); case 1 -> new Vec3d(0, sign, 0); default -> new Vec3d(0, 0, sign); };
    }
}

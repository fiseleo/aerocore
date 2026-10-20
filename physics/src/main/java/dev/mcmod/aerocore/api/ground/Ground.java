package dev.mcmod.aerocore.api.ground;

import dev.mcmod.aerocore.api.GroundContact;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;
import java.util.Objects;

/**
 * The ground under wheels and feet (0.24): what a ray from a point meets first. Wheels cast rays (a fan around the
 * rim) to find their contact patch, slope and surface. The game's ground is the world's blocks (WorldGround in the
 * neoforge module); offline benches and tests use the shapes here: flat, a plane (a slope), boxes (steps, kerbs,
 * ramps of steps) and a split floor (different grip left and right). A ray that starts inside the ground hits at once
 * (distance 0) on the face nearest its start.
 */
@FunctionalInterface
public interface Ground {
    /** The first ground along the ray from a world point in a unit direction, within maxM metres; null when there is none. */
    Hit cast(Vec3d from, Vec3d direction, double maxM);

    /**
     * Where a ray met the ground: the world point, the ground's unit normal there (out of the ground), the distance along
     * the ray (m), the surface (grip, rolling resistance, soil) and the ground's own velocity (zero for the world).
     */
    record Hit(Vec3d point, Vec3d normal, double distanceM, GroundContact.Surface surface, Vec3d velocity) {
        public Hit {
            Objects.requireNonNull(point, "point"); Objects.requireNonNull(normal, "normal");
            Objects.requireNonNull(surface, "surface"); Objects.requireNonNull(velocity, "velocity");
            normal = normal.normalized();
            if (!(distanceM >= 0) || !Double.isFinite(distanceM)) { throw new IllegalArgumentException("Hit distance must be finite and not negative"); }
        }
        /** A hit on ground that stands still. */
        public Hit(Vec3d point, Vec3d normal, double distanceM, GroundContact.Surface surface) { this(point, normal, distanceM, surface, Vec3d.ZERO); }
    }

    /** No ground anywhere (water, air). */
    Ground NONE = (from, direction, maxM) -> null;

    /** A level floor at height y. */
    static Ground flat(double y, GroundContact.Surface surface) { return plane(new Vec3d(0, y, 0), Vec3d.Y, surface); }

    /** A plane through a point with a normal pointing out of the ground (a slope: normal (sin a, cos a, 0) for a hill rising toward -x). */
    static Ground plane(Vec3d point, Vec3d normal, GroundContact.Surface surface) {
        Objects.requireNonNull(surface, "surface");
        Vec3d n = normal.normalized();
        return (from, direction, maxM) -> {
            double height = n.dot(from.subtract(point));
            if (height <= 0) { return new Hit(from, n, 0, surface); }
            double closing = -n.dot(direction);
            if (closing <= 0) { return null; }
            double t = height / closing;
            return t > maxM ? null : new Hit(from.add(direction.scale(t)), n, t, surface);
        };
    }

    /** A level floor at height y whose surface is negativeZ where z < 0 and positiveZ elsewhere (split friction). */
    static Ground split(double y, GroundContact.Surface negativeZ, GroundContact.Surface positiveZ) {
        Ground low = flat(y, negativeZ);
        return (from, direction, maxM) -> {
            Hit hit = low.cast(from, direction, maxM);
            return hit == null || hit.point().z() < 0 ? hit : new Hit(hit.point(), hit.normal(), hit.distanceM(), positiveZ, hit.velocity());
        };
    }

    /** Axis-aligned boxes (steps, kerbs, ramps of small steps), each with its surface. */
    static Ground boxes(List<BoxGround.Box> boxes) { return new BoxGround(boxes); }

    /** The nearest hit of several grounds (a floor with boxes on it). */
    static Ground nearest(Ground... grounds) {
        List<Ground> all = List.of(grounds);
        return (from, direction, maxM) -> {
            Hit best = null;
            for (Ground g : all) {
                Hit hit = g.cast(from, direction, best == null ? maxM : best.distanceM());
                if (hit != null && (best == null || hit.distanceM() < best.distanceM())) { best = hit; }
            }
            return best;
        };
    }
}

package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Static axis-aligned obstacle in world metres, for example one Minecraft collision AABB. */
public record WorldBox(Vec3d min, Vec3d max) {
    public WorldBox {
        Objects.requireNonNull(min, "min");
        Objects.requireNonNull(max, "max");
        if (min.x() > max.x() || min.y() > max.y() || min.z() > max.z()) {
            throw new IllegalArgumentException("Box minimum must not exceed maximum");
        }
    }
    public boolean intersects(WorldBox other) {
        return min.x() <= other.max.x() && max.x() >= other.min.x() && min.y() <= other.max.y()
                && max.y() >= other.min.y() && min.z() <= other.max.z() && max.z() >= other.min.z();
    }
    /** Strict interior test; points on a face are outside. */
    public boolean containsInterior(Vec3d p) {
        return p.x() > min.x() && p.x() < max.x() && p.y() > min.y() && p.y() < max.y() && p.z() > min.z() && p.z() < max.z();
    }
    public Vec3d closestPoint(Vec3d p) {
        return new Vec3d(Math.clamp(p.x(), min.x(), max.x()), Math.clamp(p.y(), min.y(), max.y()), Math.clamp(p.z(), min.z(), max.z()));
    }
}

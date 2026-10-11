package dev.mcmod.aerocore.api;

/** Immutable finite vector; the receiving API specifies world or body coordinates. Frames are right-handed. */
public record Vec3d(double x, double y, double z) {
    public static final Vec3d ZERO = new Vec3d(0, 0, 0);
    public static final Vec3d X = new Vec3d(1, 0, 0);
    public static final Vec3d Y = new Vec3d(0, 1, 0);
    public static final Vec3d Z = new Vec3d(0, 0, 1);

    public Vec3d {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            throw new IllegalArgumentException("Vector components must be finite");
        }
    }

    public Vec3d add(Vec3d other) { return new Vec3d(x + other.x, y + other.y, z + other.z); }
    public Vec3d subtract(Vec3d other) { return new Vec3d(x - other.x, y - other.y, z - other.z); }
    public Vec3d scale(double factor) { return new Vec3d(x * factor, y * factor, z * factor); }
    public double dot(Vec3d other) { return x * other.x + y * other.y + z * other.z; }
    public Vec3d cross(Vec3d other) {
        return new Vec3d(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }
    public double lengthSquared() { return dot(this); }
    /** Euclidean length; sqrt of the sum of squares (hypot only where that overflows: hypot is several times slower and this is in every contact loop). */
    public double length() {
        double squared = x * x + y * y + z * z;
        return squared < Double.POSITIVE_INFINITY ? Math.sqrt(squared) : Math.hypot(Math.hypot(x, y), z);
    }
    public Vec3d normalized() {
        double length = length();
        if (length < 1e-12) { throw new IllegalArgumentException("Direction must be nonzero"); }
        return scale(1.0 / length);
    }
}

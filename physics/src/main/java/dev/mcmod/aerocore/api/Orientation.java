package dev.mcmod.aerocore.api;

/** Unit quaternion (w,x,y,z), actively mapping body coordinates to world coordinates. */
public record Orientation(double w, double x, double y, double z) {
    public static final Orientation IDENTITY = new Orientation(1, 0, 0, 0);
    public Orientation {
        double norm = Math.hypot(Math.hypot(w, x), Math.hypot(y, z));
        if (!Double.isFinite(norm) || norm < 1e-12) { throw new IllegalArgumentException("Invalid quaternion"); }
        w /= norm; x /= norm; y /= norm; z /= norm;
    }
    public static Orientation rotationVector(Vec3d radians) {
        double angle = radians.length();
        double factor = angle < 1e-8 ? 0.5 - angle * angle / 48 : Math.sin(angle / 2) / angle;
        return new Orientation(Math.cos(angle / 2), radians.x() * factor, radians.y() * factor, radians.z() * factor);
    }
    public Orientation multiply(Orientation b) {
        return new Orientation(w*b.w-x*b.x-y*b.y-z*b.z, w*b.x+x*b.w+y*b.z-z*b.y,
                w*b.y-x*b.z+y*b.w+z*b.x, w*b.z+x*b.y-y*b.x+z*b.w);
    }
    public Orientation inverse() { return new Orientation(w, -x, -y, -z); }
    public Vec3d rotate(Vec3d v) {
        Vec3d q = new Vec3d(x, y, z), t = q.cross(v).scale(2);
        return v.add(t.scale(w)).add(q.cross(t));
    }
    public Orientation advanceWorld(Vec3d omega, double seconds) {
        return rotationVector(omega.scale(seconds)).multiply(this);
    }
    /** Shortest-arc normalized interpolation for client presentation only. */
    public Orientation interpolate(Orientation b, double t) {
        if (!Double.isFinite(t) || t < 0 || t > 1) { throw new IllegalArgumentException("Interpolation must be in [0,1]"); }
        double sign = w*b.w+x*b.x+y*b.y+z*b.z < 0 ? -1 : 1;
        return new Orientation(w*(1-t)+sign*b.w*t, x*(1-t)+sign*b.x*t, y*(1-t)+sign*b.y*t, z*(1-t)+sign*b.z*t);
    }
}

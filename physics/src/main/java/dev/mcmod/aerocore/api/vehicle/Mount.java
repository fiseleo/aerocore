package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.Map;
import java.util.Objects;

/**
 * Where a part sits on a vehicle: the part's frame origin at position (vehicle frame, m) and turned by yaw about +Y,
 * then pitch about the turned +Z (positive lifts the part's +X), then roll about the turned +X (positive lowers its
 * +Z side), in degrees. Vehicle and part frames share the convention x forward, y up, z starboard.
 */
public record Mount(Vec3d position, double yawDeg, double pitchDeg, double rollDeg) {
    public static final Mount ORIGIN = new Mount(Vec3d.ZERO, 0, 0, 0);

    public Mount {
        Objects.requireNonNull(position, "position");
        if (!Double.isFinite(yawDeg + pitchDeg + rollDeg)) { throw new IllegalArgumentException("Mount angles must be finite"); }
    }

    public static Mount at(Vec3d position) { return new Mount(position, 0, 0, 0); }

    /** Part frame to vehicle frame. */
    public Orientation rotation() {
        return Orientation.rotationVector(Vec3d.Y.scale(Math.toRadians(yawDeg)))
                .multiply(Orientation.rotationVector(Vec3d.Z.scale(Math.toRadians(pitchDeg))))
                .multiply(Orientation.rotationVector(Vec3d.X.scale(Math.toRadians(rollDeg))));
    }

    /** A part-frame point in the vehicle frame. */
    public Vec3d apply(Vec3d partPoint) { return position.add(rotation().rotate(partPoint)); }

    Map<String, Object> toJson() {
        Map<String, Object> m = Fields.object("position", Fields.vector(position));
        if (yawDeg != 0 || pitchDeg != 0 || rollDeg != 0) { m.put("rotation", Fields.object("yaw", yawDeg, "pitch", pitchDeg, "roll", rollDeg)); }
        return m;
    }

    static Mount fromJson(Fields f) {
        Vec3d p = f.vec("position", Vec3d.ZERO);
        if (!f.has("rotation")) { return at(p); }
        Fields r = f.child("rotation");
        return new Mount(p, r.number("yaw", 0), r.number("pitch", 0), r.number("roll", 0));
    }
}

package dev.mcmod.aerocore.api.vehicle;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

import java.util.Map;
import java.util.Objects;

/**
 * Where a part sits on a vehicle: the part's frame origin at position (vehicle frame, m) and turned by yaw about +Y,
 * then pitch about the turned +Z (positive lifts the part's +X), then roll about the turned +X (positive lowers its
 * +Z side), in degrees. Vehicle and part frames share the convention x forward, y up, z starboard.
 * <p>Attached (since 0.23): with a part and one of its mount points (Part.mountPoints, e.g. a body's "wing" or "mast"),
 * position and angles are an offset in that point's frame, and the vehicle places the part where the point is.
 */
public record Mount(Vec3d position, double yawDeg, double pitchDeg, double rollDeg, String attachPart, String attachPoint) {
    public static final Mount ORIGIN = new Mount(Vec3d.ZERO, 0, 0, 0);

    public Mount {
        Objects.requireNonNull(position, "position");
        if (!Double.isFinite(yawDeg + pitchDeg + rollDeg)) { throw new IllegalArgumentException("Mount angles must be finite"); }
        if ((attachPart == null) != (attachPoint == null)) { throw new IllegalArgumentException("An attachment names both a part and its point"); }
    }

    public Mount(Vec3d position, double yawDeg, double pitchDeg, double rollDeg) { this(position, yawDeg, pitchDeg, rollDeg, null, null); }

    public static Mount at(Vec3d position) { return new Mount(position, 0, 0, 0); }

    /** On another part's mount point, with no offset. */
    public static Mount on(String part, String point) { return new Mount(Vec3d.ZERO, 0, 0, 0, part, point); }

    /** On another part's mount point, offset (in the point's frame) by this mount's position and angles. */
    public static Mount on(String part, String point, Mount offset) {
        return new Mount(offset.position, offset.yawDeg, offset.pitchDeg, offset.rollDeg, part, point);
    }

    public boolean attached() { return attachPart != null; }

    /** The same placement without its attachment (the offset alone). */
    public Mount detached() { return new Mount(position, yawDeg, pitchDeg, rollDeg); }

    /** Part frame to vehicle frame. */
    public Orientation rotation() {
        return Orientation.rotationVector(Vec3d.Y.scale(Math.toRadians(yawDeg)))
                .multiply(Orientation.rotationVector(Vec3d.Z.scale(Math.toRadians(pitchDeg))))
                .multiply(Orientation.rotationVector(Vec3d.X.scale(Math.toRadians(rollDeg))));
    }

    /** A part-frame point in the vehicle frame. */
    public Vec3d apply(Vec3d partPoint) { return position.add(rotation().rotate(partPoint)); }

    /** A placement given in this one's frame (as a mount point's frame), in the frame this one is given in. */
    public Mount then(Mount inner) {
        return of(apply(inner.position), rotation().multiply(inner.rotation()));
    }

    /** A placement from a position and a rotation (the angles recovered from it). */
    public static Mount of(Vec3d position, Orientation rotation) {
        // R = Ry(yaw) Rz(pitch) Rx(roll): R X = (cos y cos p, sin p, -sin y cos p); (R Y).y = cos p cos r; (R Z).y = -cos p sin r
        Vec3d ex = rotation.rotate(Vec3d.X), ey = rotation.rotate(Vec3d.Y), ez = rotation.rotate(Vec3d.Z);
        double pitch = Math.asin(Math.clamp(ex.y(), -1, 1));
        double yaw, roll;
        if (Math.abs(ex.y()) < 1 - 1e-9) {
            yaw = Math.atan2(-ex.z(), ex.x());
            roll = Math.atan2(-ez.y(), ey.y());
        } else {
            // pointing straight up or down: put the turn into the yaw
            yaw = Math.atan2(ez.x(), ez.z());
            roll = 0;
        }
        return new Mount(position, Math.toDegrees(yaw), Math.toDegrees(pitch), Math.toDegrees(roll));
    }

    Map<String, Object> toJson() {
        Map<String, Object> m = Fields.object("position", Fields.vector(position));
        if (yawDeg != 0 || pitchDeg != 0 || rollDeg != 0) { m.put("rotation", Fields.object("yaw", yawDeg, "pitch", pitchDeg, "roll", rollDeg)); }
        if (attached()) { m.put("attach", Fields.object("part", attachPart, "point", attachPoint)); }
        return m;
    }

    static Mount fromJson(Fields f) {
        Vec3d p = f.vec("position", Vec3d.ZERO);
        double yaw = 0, pitch = 0, roll = 0;
        if (f.has("rotation")) {
            Fields r = f.child("rotation");
            yaw = r.number("yaw", 0); pitch = r.number("pitch", 0); roll = r.number("roll", 0);
        }
        String part = null, point = null;
        if (f.has("attach")) {
            Fields a = f.child("attach");
            part = a.string("part"); point = a.string("point");
        }
        return new Mount(p, yaw, pitch, roll, part, point);
    }
}

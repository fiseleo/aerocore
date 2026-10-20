package dev.mcmod.aerocore.api.mesh;

import dev.mcmod.aerocore.api.Orientation;
import dev.mcmod.aerocore.api.Vec3d;

/**
 * A lamp on a part (0.30): where it is and which way it shines in the mesh's frame, the piece it moves with
 * (MeshBuilder.piece; 0 the fixed part), its size and colour, and how it shows: only its lens glowing (an instrument
 * light), a point that also shows a halo seen from afar (a navigation light, a beacon), or a beam (a headlight: a
 * halo, a cone of light in the dark and a pool where it lands). Its lens is the quads tagged with its number
 * (MeshBuilder.lamp). How bright it is at any moment is a rule of the part's renderer (on, off, flashing); the role is
 * the renderer's own code for which rule (a tail light, a strobe).
 * <p>
 * A sector light shows only within sectorDeg of its axis (a navigation light's red to port); 360 shows all round. A
 * beam spreads beamDeg (its half angle) and reaches rangeM.
 */
public record Lamp(int id, int piece, Vec3d at, Vec3d axis, double radiusM, int argb, Kind kind, double sectorDeg, double beamDeg, double rangeM, int role) {
    public enum Kind { LENS, POINT, BEAM }

    public Lamp {
        if (id < 0 || id > 254) { throw new IllegalArgumentException("A lamp's number is 0-254"); }
        if (axis.lengthSquared() < 1e-18) { throw new IllegalArgumentException("A lamp needs an axis"); }
        axis = axis.normalized();
    }

    /** A lens-only lamp (glows, no halo of its own; 0.39: with beautified shading on, each lit patch of its lens gets a soft halo, LensPatches). */
    public static Lamp lens(int id, int piece, Vec3d at, Vec3d axis, double radiusM, int argb, int role) {
        return new Lamp(id, piece, at, axis, radiusM, argb, Kind.LENS, 360, 0, 0, role);
    }

    /** A point light (a halo), shown within a sector about its axis (360: all round). */
    public static Lamp point(int id, int piece, Vec3d at, Vec3d axis, double radiusM, int argb, double sectorDeg, int role) {
        return new Lamp(id, piece, at, axis, radiusM, argb, Kind.POINT, sectorDeg, 0, 0, role);
    }

    /** A beam: a halo seen from ahead, a cone of half angle beamDeg and a pool where it lands within rangeM. */
    public static Lamp beam(int id, int piece, Vec3d at, Vec3d axis, double radiusM, int argb, double beamDeg, double rangeM, int role) {
        return new Lamp(id, piece, at, axis, radiusM, argb, Kind.BEAM, 2 * Math.max(beamDeg, 30), beamDeg, rangeM, role);
    }

    /** The lamp turned and moved with its mesh. */
    public Lamp moved(Orientation rotation, Vec3d offset) {
        return new Lamp(id, piece, offset.add(rotation.rotate(at)), rotation.rotate(axis), radiusM, argb, kind, sectorDeg, beamDeg, rangeM, role);
    }

    /** The lamp scaled with its mesh, then turned and moved (MeshOps.placed). */
    public Lamp placed(Vec3d scale, Orientation rotation, Vec3d offset) {
        Vec3d p = new Vec3d(at.x() * scale.x(), at.y() * scale.y(), at.z() * scale.z());
        Vec3d a = new Vec3d(axis.x() * scale.x(), axis.y() * scale.y(), axis.z() * scale.z());
        double k = Math.cbrt(Math.abs(scale.x() * scale.y() * scale.z()));
        return new Lamp(id, piece, offset.add(rotation.rotate(p)), rotation.rotate(a), radiusM * k, argb, kind, sectorDeg, beamDeg, rangeM * k, role);
    }

    /** The lamp mirrored with its mesh across an axis' plane (0 x, 1 y, 2 z). */
    public Lamp mirrored(int across) {
        return new Lamp(id, piece, reflect(at, across), reflect(axis, across), radiusM, argb, kind, sectorDeg, beamDeg, rangeM, role);
    }

    /** The same lamp with another number, piece, colour or role (a mirrored copy made the starboard light). */
    public Lamp with(int newId, int newPiece, int newArgb, int newRole) {
        return new Lamp(newId, newPiece, at, axis, radiusM, newArgb, kind, sectorDeg, beamDeg, rangeM, newRole);
    }

    private static Vec3d reflect(Vec3d v, int axis) {
        return axis == 0 ? new Vec3d(-v.x(), v.y(), v.z()) : axis == 1 ? new Vec3d(v.x(), -v.y(), v.z()) : new Vec3d(v.x(), v.y(), -v.z());
    }
}

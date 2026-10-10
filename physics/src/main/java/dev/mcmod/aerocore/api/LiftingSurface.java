package dev.mcmod.aerocore.api;

import java.util.Objects;

/**
 * World-space geometry. Offset is from the center of mass to the force application point (m).
 * Chord points toward the leading edge; normal points toward the positive-lift side.
 * Rotate local geometry into world space before each solve. Area is in m^2.
 */
public record LiftingSurface(Vec3d offset, Vec3d chord, Vec3d normal,
                             double areaM2, double chordLengthM, AirfoilPolar polar) {
    /** Compatibility constructor; use an explicit chord length for Reynolds number and Cm. */
    public LiftingSurface(Vec3d offset, Vec3d chord, Vec3d normal, double areaM2, AirfoilPolar polar) {
        this(offset, chord, normal, areaM2, 1, polar);
    }
    public LiftingSurface {
        Objects.requireNonNull(offset, "offset");
        chord = Objects.requireNonNull(chord, "chord").normalized();
        normal = Objects.requireNonNull(normal, "normal").normalized();
        if (Math.abs(chord.dot(normal)) > 1e-8) {
            throw new IllegalArgumentException("Chord and normal must be perpendicular");
        }
        Checks.positive(areaM2, "Wing area");
        Checks.positive(chordLengthM, "Chord length");
        Objects.requireNonNull(polar, "polar");
    }
    public Vec3d span() { return chord.cross(normal).normalized(); }
}

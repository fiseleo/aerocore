package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Displaced volume and its centroid, expressed as an offset from the body COM. */
public record Immersion(double displacedVolumeM3, Vec3d centerOfBuoyancy, double fraction) {
    public Immersion {
        Checks.nonnegative(displacedVolumeM3, "Displaced volume");
        Objects.requireNonNull(centerOfBuoyancy, "centerOfBuoyancy");
        if (!Double.isFinite(fraction) || fraction < 0 || fraction > 1) {
            throw new IllegalArgumentException("Immersion fraction must be in [0, 1]");
        }
    }
}

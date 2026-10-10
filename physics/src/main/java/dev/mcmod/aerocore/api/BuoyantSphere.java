package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Spherical displacement element; offset is from the body COM to the sphere center, in metres. */
public record BuoyantSphere(Vec3d offset, double radiusM) {
    public BuoyantSphere {
        Objects.requireNonNull(offset, "offset");
        Checks.positive(radiusM, "Radius");
    }
    public double volumeM3() { return 4.0 / 3 * Math.PI * radiusM * radiusM * radiusM; }
}

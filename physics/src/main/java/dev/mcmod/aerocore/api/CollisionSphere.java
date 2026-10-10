package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Contact primitive in model-local metres. Approximate boxes with spheres at their corners. */
public record CollisionSphere(Vec3d centerLocal, double radiusM) {
    public CollisionSphere {
        Objects.requireNonNull(centerLocal, "centerLocal");
        Checks.positive(radiusM, "Collision radius");
    }
}

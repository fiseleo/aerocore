package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Center-of-mass velocity in m/s and world-space angular velocity in rad/s. */
public record BodyMotion(Vec3d linearVelocity, Vec3d angularVelocity) {
    public BodyMotion {
        Objects.requireNonNull(linearVelocity, "linearVelocity");
        Objects.requireNonNull(angularVelocity, "angularVelocity");
    }
    public Vec3d velocityAt(Vec3d offsetFromCenterOfMass) {
        return linearVelocity.add(angularVelocity.cross(offsetFromCenterOfMass));
    }
}


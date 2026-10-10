package dev.mcmod.aerocore.api;

import java.util.Objects;

/** COM location in the model frame; inertia expressed about that COM, aligned with model axes. */
public record RigidBodyProperties(double massKg, Vec3d centerOfMassLocal, InertiaTensor inertia) {
    public RigidBodyProperties {
        Checks.positive(massKg, "Mass"); Objects.requireNonNull(centerOfMassLocal); Objects.requireNonNull(inertia);
    }
}

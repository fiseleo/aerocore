package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Center-of-mass position (m) and velocity (m/s); rotation and collisions are separate. */
public record TranslationState(Vec3d position, Vec3d velocity) {
    public TranslationState {
        Objects.requireNonNull(position, "position");
        Objects.requireNonNull(velocity, "velocity");
    }
}

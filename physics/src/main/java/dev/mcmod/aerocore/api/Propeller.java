package dev.mcmod.aerocore.api;

import java.util.Objects;

/**
 * Propeller disk in model-local coordinates: hub position (m), unit thrust axis, diameter (m).
 * Positive rev/s turns right-handed about the axis (clockwise seen from behind) and pushes along it.
 */
public record Propeller(Vec3d positionLocal, Vec3d axisLocal, double diameterM, PropellerModel model) {
    public Propeller {
        Objects.requireNonNull(positionLocal, "positionLocal");
        axisLocal = Objects.requireNonNull(axisLocal, "axisLocal").normalized();
        Checks.positive(diameterM, "Propeller diameter");
        Objects.requireNonNull(model, "model");
    }
}

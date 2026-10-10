package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Air density in kg/m^3 and air velocity in world-space m/s (not a 'wind-from' direction). */
public record AirState(double densityKgPerM3, Vec3d windVelocity) {
    public static final AirState STILL_SEA_LEVEL = new AirState(1.225, Vec3d.ZERO);
    public AirState {
        Checks.nonnegative(densityKgPerM3, "Air density");
        Objects.requireNonNull(windVelocity, "windVelocity");
    }

    /** Legacy air inputs use sea-level viscosity/sound speed. Use Atmosphere for altitude effects. */
    public FluidState asFluid() {
        return new FluidState(densityKgPerM3, 1.7894e-5, 340.294, windVelocity);
    }
}

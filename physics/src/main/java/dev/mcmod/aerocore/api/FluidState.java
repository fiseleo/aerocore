package dev.mcmod.aerocore.api;

import java.util.Objects;

/** A local single-phase fluid sample in SI units. Velocity points in the direction of flow. */
public record FluidState(double densityKgPerM3, double dynamicViscosityPaS,
                         double speedOfSoundMps, Vec3d velocity) {
    /** Approximate fresh-water properties at 20 C; no temperature/salinity equation of state. */
    public static final FluidState FRESH_WATER_20C = new FluidState(998.2, 1.002e-3, 1482, Vec3d.ZERO);

    public FluidState {
        Checks.nonnegative(densityKgPerM3, "Fluid density");
        Checks.positive(dynamicViscosityPaS, "Dynamic viscosity");
        Checks.positive(speedOfSoundMps, "Speed of sound");
        Objects.requireNonNull(velocity, "velocity");
    }

    public FluidState withVelocity(Vec3d newVelocity) {
        return new FluidState(densityKgPerM3, dynamicViscosityPaS, speedOfSoundMps, newVelocity);
    }
}

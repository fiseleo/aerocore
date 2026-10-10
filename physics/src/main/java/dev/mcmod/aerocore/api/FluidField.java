package dev.mcmod.aerocore.api;

/** Caller-provided field for spatial wind/current variations. Return local SI properties. */
@FunctionalInterface
public interface FluidField {
    FluidState sample(Vec3d worldPositionM, double timeSeconds);
}

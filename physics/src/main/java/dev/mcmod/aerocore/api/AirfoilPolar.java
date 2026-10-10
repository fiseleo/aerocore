package dev.mcmod.aerocore.api;

/** Implement this interface to supply measured or custom polars. Input is in [-pi, pi] radians. */
@FunctionalInterface
public interface AirfoilPolar {
    AeroCoefficients sample(double angleOfAttackRadians);

    /** Override for measured Reynolds/Mach-dependent data; the default remains angle-only. */
    default AeroCoefficients sample(double angleOfAttackRadians, FlowConditions flow) {
        return sample(angleOfAttackRadians);
    }
}

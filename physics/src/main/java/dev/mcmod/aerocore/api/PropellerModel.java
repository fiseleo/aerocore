package dev.mcmod.aerocore.api;

/**
 * Supplies four-quadrant coefficients at the hydrodynamic pitch angle beta = atan2(Va, 0.7 pi n D),
 * in [-pi, pi]. [0, pi/2] is ahead rotation with forward advance; n is in rev/s, positive for right-hand
 * rotation about the thrust axis. Implementations must be thread-safe if shared.
 */
@FunctionalInterface
public interface PropellerModel {
    PropellerCoefficients sample(double hydrodynamicPitchRadians);
}

package dev.mcmod.aerocore.api;

import java.util.Objects;

/**
 * Sphere in a wave field. fraction is the geometric wet volume fraction; pressureFraction is the vertical
 * Froude-Krylov force divided by rho g V (equal to fraction in still water). Offsets are world vectors from
 * the sphere center. velocity and acceleration are wet-volume averages of the orbital field (m/s, m/s^2).
 */
public record WaveImmersion(double fraction, Vec3d wetCentroidOffset, double pressureFraction,
                            Vec3d pressureCentroidOffset, Vec3d velocity, Vec3d acceleration) {
    public WaveImmersion {
        if (!Double.isFinite(fraction) || fraction < 0 || fraction > 1) { throw new IllegalArgumentException("Wet fraction must be in [0,1]"); }
        if (!Double.isFinite(pressureFraction)) { throw new IllegalArgumentException("Pressure fraction must be finite"); }
        Objects.requireNonNull(wetCentroidOffset); Objects.requireNonNull(pressureCentroidOffset);
        Objects.requireNonNull(velocity); Objects.requireNonNull(acceleration);
    }
}

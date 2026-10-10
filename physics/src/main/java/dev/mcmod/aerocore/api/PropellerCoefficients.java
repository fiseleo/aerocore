package dev.mcmod.aerocore.api;

/**
 * Four-quadrant open-water coefficients (Carlton, Marine Propellers and Propulsion; ITTC):
 * C_T* = T / (q A0), C_Q* = Q / (q A0 D), q = rho Vr^2 / 2, A0 = pi D^2 / 4, Vr^2 = Va^2 + (0.7 pi n D)^2.
 * Thrust is positive along the propeller axis; torque is the shaft torque needed to keep it turning.
 */
public record PropellerCoefficients(double thrust, double torque) {
    public PropellerCoefficients {
        if (!Double.isFinite(thrust) || !Double.isFinite(torque)) {
            throw new IllegalArgumentException("Propeller coefficients must be finite");
        }
    }
    public PropellerCoefficients negate() { return new PropellerCoefficients(-thrust, -torque); }
}

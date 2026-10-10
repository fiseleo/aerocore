package dev.mcmod.aerocore.api;

/**
 * Game-oriented symmetric polar: linear attached lift, smoothly blended to a flat-plate
 * approximation after stall. Coefficients are tunable, not measured airfoil data.
 */
public record SymmetricAirfoil(double liftSlopePerRadian, double stallAngleRadians,
                               double stallBlendRadians, double zeroLiftDrag,
                               double aspectRatio, double efficiency) implements AirfoilPolar {
    public static final SymmetricAirfoil DEFAULT = new SymmetricAirfoil(
            5.5, Math.toRadians(15), Math.toRadians(10), 0.025, 6, 0.8);

    public SymmetricAirfoil {
        Checks.positive(liftSlopePerRadian, "Lift slope");
        Checks.positive(stallAngleRadians, "Stall angle");
        Checks.positive(stallBlendRadians, "Stall blend width");
        if (stallAngleRadians + stallBlendRadians >= Math.PI / 2) {
            throw new IllegalArgumentException("Stall blend must finish before 90 degrees");
        }
        Checks.nonnegative(zeroLiftDrag, "Zero-lift drag");
        Checks.positive(aspectRatio, "Aspect ratio");
        Checks.positive(efficiency, "Efficiency");
        if (efficiency > 1) { throw new IllegalArgumentException("Efficiency must be <= 1"); }
    }

    @Override
    public AeroCoefficients sample(double angleOfAttackRadians) {
        if (!Double.isFinite(angleOfAttackRadians) || Math.abs(angleOfAttackRadians) > Math.PI) {
            throw new IllegalArgumentException("Angle of attack must be in [-pi, pi]");
        }
        double alpha = angleOfAttackRadians;
        double blend = Math.clamp((Math.abs(alpha) - stallAngleRadians) / stallBlendRadians, 0, 1);
        blend = blend * blend * (3 - 2 * blend);
        double cl = (1 - blend) * liftSlopePerRadian * alpha + blend * Math.sin(2 * alpha);
        double cd = zeroLiftDrag + cl * cl / (Math.PI * aspectRatio * efficiency)
                + blend * 1.8 * Math.pow(Math.sin(alpha), 2);
        return new AeroCoefficients(cl, cd, Math.abs(alpha) > stallAngleRadians);
    }
}


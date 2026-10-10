package dev.mcmod.aerocore.api;

/** Per-surface diagnostics. Pressure is in Pa and angle of attack is in radians. */
public record SurfaceForces(Wrench wrench, Vec3d lift, Vec3d drag,
                            double angleOfAttackRadians, double dynamicPressurePa,
                            AeroCoefficients coefficients, FlowConditions flow) {
    public SurfaceForces(Wrench wrench, Vec3d lift, Vec3d drag, double angleOfAttackRadians,
                         double dynamicPressurePa, AeroCoefficients coefficients) {
        this(wrench, lift, drag, angleOfAttackRadians, dynamicPressurePa, coefficients, FlowConditions.ZERO);
    }
    public static final SurfaceForces ZERO = new SurfaceForces(Wrench.ZERO, Vec3d.ZERO, Vec3d.ZERO,
            0, 0, new AeroCoefficients(0, 0, false));
}

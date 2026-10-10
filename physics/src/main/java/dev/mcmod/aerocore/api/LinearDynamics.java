package dev.mcmod.aerocore.api;

/** Translational integration helpers in SI units. No automatic gravity or collision response. */
public final class LinearDynamics {
    private LinearDynamics() {}

    /** Exact for a force held constant over this step; resample changing forces in small substeps. */
    public static TranslationState constantForce(TranslationState state, Vec3d force, double massKg, double dtSeconds) {
        Checks.positive(massKg, "Mass");
        Checks.positive(dtSeconds, "Time step");
        Vec3d deltaVelocity = force.scale(dtSeconds / massKg);
        return new TranslationState(state.position().add(state.velocity().add(deltaVelocity.scale(0.5)).scale(dtSeconds)),
                state.velocity().add(deltaVelocity));
    }

    /**
     * Exact velocity update for isolated quadratic drag in a constant uniform fluid.
     * Relative velocity cannot reverse from drag alone, even at a large time step.
     */
    public static Vec3d quadraticDrag(Vec3d velocity, FluidState fluid, double areaM2,
                                     double cd, double massKg, double dtSeconds) {
        Checks.nonnegative(areaM2, "Drag area");
        Checks.nonnegative(cd, "Drag coefficient");
        Checks.positive(massKg, "Mass");
        Checks.positive(dtSeconds, "Time step");
        Vec3d relative = velocity.subtract(fluid.velocity());
        double denominator = 1 + 0.5 * fluid.densityKgPerM3() * areaM2 * cd * relative.length() * dtSeconds / massKg;
        if (!Double.isFinite(denominator)) { throw new IllegalArgumentException("Drag inputs exceed numerical range"); }
        return fluid.velocity().add(relative.scale(1 / denominator));
    }
}

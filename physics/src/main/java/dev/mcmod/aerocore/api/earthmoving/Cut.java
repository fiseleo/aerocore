package dev.mcmod.aerocore.api.earthmoving;

/**
 * What a tool meets in a cut (the answer of {@link SoilCutting}, {@link RockCutting#edge} and {@link Digging}).
 *
 * @param horizontalN     resistance against the travel, N: the tool must be pushed this hard (never negative)
 * @param liftN           upward force of the material on the tool, N: positive pushes the tool out of the ground,
 *                        negative pulls it in (a sharp, low-rake edge in soil is drawn in)
 * @param failureAngleRad angle of the failure plane ahead of the tool from the horizontal, rad (the beta that gives the
 *                        least force); 0 for rock, which is cut by specific energy
 * @param bankM3PerS      volume the tool's cross-section sweeps, bank m^3/s (depth x width x speed): what it cuts off
 * @param disturbedWidthM width of the failed material at the surface, m: the tool's width, plus the side crescents of
 *                        a narrow tine
 */
public record Cut(double horizontalN, double liftN, double failureAngleRad, double bankM3PerS, double disturbedWidthM) {
    /** Nothing in the way. */
    public static final Cut NONE = new Cut(0, 0, 0, 0, 0);

    /** Cutting power at a travel speed (m/s), W: the horizontal resistance times the speed. */
    public double powerW(double speedMps) { return horizontalN * Math.abs(speedMps); }

    /** Bank volume cut in a time step (s), m^3. */
    public double bankM3(double dtS) { return bankM3PerS * dtS; }
}

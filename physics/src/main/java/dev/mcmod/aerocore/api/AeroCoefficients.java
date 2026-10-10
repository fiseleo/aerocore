package dev.mcmod.aerocore.api;

/** Dimensionless coefficients, based on the lifting surface's reference area. */
public record AeroCoefficients(double lift, double drag, double pitchingMoment, boolean stalled) {
    /** Compatibility constructor: no intrinsic pitching moment. */
    public AeroCoefficients(double lift, double drag, boolean stalled) {
        this(lift, drag, 0, stalled);
    }
    public AeroCoefficients {
        if (!Double.isFinite(lift)) { throw new IllegalArgumentException("Lift coefficient must be finite"); }
        Checks.nonnegative(drag, "Drag coefficient");
        if (!Double.isFinite(pitchingMoment)) { throw new IllegalArgumentException("Moment coefficient must be finite"); }
    }
}

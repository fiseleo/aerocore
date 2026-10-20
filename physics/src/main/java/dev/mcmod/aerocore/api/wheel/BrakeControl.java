package dev.mcmod.aerocore.api.wheel;

/**
 * A rule between the driver's brake demand and a wheel's brake (0.24): anti-lock braking, brake proportioning. The
 * rules themselves belong to the vehicle mod; AeroCore calls them each step with what the wheel's tyre feels. memory
 * is the wheel's own state (memory() values, kept between steps as the part's internal states).
 */
@FunctionalInterface
public interface BrakeControl {
    /**
     * The brake demand (0..1) the wheel gets from the driver's demand, its transient slip ratio (negative braking),
     * the vehicle's speed over the ground at the wheel (m/s), its memory (read and write) and the step (s).
     */
    double demand(double driverDemand, double slipRatio, double speedMps, double[] memory, double dt);

    /** Values of memory a wheel keeps. */
    default int memory() { return 0; }
}

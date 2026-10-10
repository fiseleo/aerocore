package dev.mcmod.aerocore.api;

/**
 * Linear (Airy) progressive wave over a flat bottom; Dean and Dalrymple, Water Wave Mechanics for
 * Engineers and Scientists (1991), ch. 3-4. Elevation eta = A cos(k (x cos d + z sin d) - omega t + phase)
 * with dispersion omega^2 = g k tanh(k h). World x/z are horizontal, y is up; callers supply the still-water
 * level. Depth may be +infinity (deep water). Limited to small steepness and non-breaking heights; there is
 * no reflection, refraction, shoaling, breaking or wave-current interaction.
 */
public record LinearWave(double amplitudeM, double wavelengthM, double directionRadians, double depthM, double phaseRadians) {
    /** Height-to-length limit kept well below the Michell breaking steepness (about 1/7). */
    public static final double MAX_STEEPNESS = 0.1;
    /** McCowan solitary-wave breaking index H/h = 0.78, used as a shallow-water upper bound. */
    public static final double MAX_HEIGHT_TO_DEPTH = 0.78;

    public LinearWave {
        Checks.nonnegative(amplitudeM, "Wave amplitude");
        Checks.positive(wavelengthM, "Wavelength");
        if (!Double.isFinite(directionRadians) || !Double.isFinite(phaseRadians)) {
            throw new IllegalArgumentException("Wave direction and phase must be finite");
        }
        if (Double.isNaN(depthM) || depthM <= 0) { throw new IllegalArgumentException("Water depth must be positive or +infinity"); }
        if (2 * amplitudeM / wavelengthM > MAX_STEEPNESS) {
            throw new IllegalArgumentException("Wave too steep for linear theory: H/L must be <= " + MAX_STEEPNESS);
        }
        if (Double.isFinite(depthM) && 2 * amplitudeM > MAX_HEIGHT_TO_DEPTH * depthM) {
            throw new IllegalArgumentException("Wave height exceeds the breaking limit H <= 0.78 h");
        }
    }

    public double wavenumber() { return 2 * Math.PI / wavelengthM; }
    /** Standard gravity is used, matching the rest of the API. */
    public double angularFrequency() {
        double k = wavenumber();
        return Math.sqrt(Atmosphere.GRAVITY_MPS2 * k * Math.tanh(k * depthM));
    }
    public double periodS() { return 2 * Math.PI / angularFrequency(); }
    public double phaseSpeedMps() { return angularFrequency() / wavenumber(); }
    /** Unit horizontal propagation direction: 0 rad is +X, pi/2 is +Z. */
    public Vec3d direction() { return new Vec3d(Math.cos(directionRadians), 0, Math.sin(directionRadians)); }

    public double elevation(double x, double z, double t) { return amplitudeM * Math.cos(phase(x, z, t)); }

    /** Horizontal surface slope (d eta/dx, 0, d eta/dz), dimensionless; its magnitude never exceeds A k. */
    public Vec3d slope(double x, double z, double t) {
        double factor = -amplitudeM * wavenumber() * Math.sin(phase(x, z, t));
        return new Vec3d(factor * Math.cos(directionRadians), 0, factor * Math.sin(directionRadians));
    }

    /** The same wave with its amplitude multiplied by a factor in [0, 1]. */
    public LinearWave scaled(double factor) {
        if (!Double.isFinite(factor) || factor < 0 || factor > 1) { throw new IllegalArgumentException("Amplitude factor must be in [0, 1]"); }
        return new LinearWave(amplitudeM * factor, wavelengthM, directionRadians, depthM, phaseRadians);
    }

    /**
     * Orbital velocity in m/s. Depth below still water is clamped to [0, h]; above still water the
     * surface value is held constant (first-order theory is defined only up to the mean level).
     */
    public Vec3d velocity(Vec3d point, double stillWaterY, double t) {
        double[] depth = depthFactors(point.y() - stillWaterY);
        double theta = phase(point.x(), point.z(), t), speed = amplitudeM * angularFrequency();
        return direction().scale(speed * depth[0] * Math.cos(theta)).add(Vec3d.Y.scale(speed * depth[1] * Math.sin(theta)));
    }

    /**
     * Local fluid acceleration du/dt in m/s^2 (linear: the convective term is second order). It satisfies the
     * linearized Euler equation du/dt = -g grad(dynamicPressureHead). Same depth clamping as velocity().
     */
    public Vec3d acceleration(Vec3d point, double stillWaterY, double t) {
        double[] depth = depthFactors(point.y() - stillWaterY);
        double theta = phase(point.x(), point.z(), t), omega = angularFrequency(), scale = amplitudeM * omega * omega;
        return direction().scale(scale * depth[0] * Math.sin(theta)).add(Vec3d.Y.scale(-scale * depth[1] * Math.cos(theta)));
    }

    /**
     * Dynamic pressure head p_d/(rho g) = A K(z) cos(theta), K = cosh(k(z+h))/cosh(kh), in metres of water.
     * Total linear pressure below still water is rho g (head - z). It decays with depth (the Smith effect),
     * which is what the instantaneous-surface hydrostatic approximation omits. Above still water K = 1.
     */
    public double dynamicPressureHead(Vec3d point, double stillWaterY, double t) {
        double k = wavenumber(), z = clampDepth(point.y() - stillWaterY);
        double image = Double.isFinite(depthM) ? Math.exp(-k * (z + 2 * depthM)) : 0;
        double denominator = Double.isFinite(depthM) ? 1 + Math.exp(-2 * k * depthM) : 1;
        return amplitudeM * (Math.exp(k * z) + image) / denominator * Math.cos(phase(point.x(), point.z(), t));
    }

    /** cosh(k(z+h))/sinh(kh) and sinh(k(z+h))/sinh(kh) at the clamped depth, finite in deep water. */
    private double[] depthFactors(double heightAboveStill) {
        double k = wavenumber(), z = clampDepth(heightAboveStill);
        boolean finite = Double.isFinite(depthM);
        double decay = Math.exp(k * z), image = finite ? Math.exp(-k * (z + 2 * depthM)) : 0;
        double denominator = finite ? -Math.expm1(-2 * k * depthM) : 1;
        return new double[]{(decay + image) / denominator, (decay - image) / denominator};
    }

    private double clampDepth(double heightAboveStill) {
        double z = Math.min(heightAboveStill, 0);
        return Double.isFinite(depthM) ? Math.max(z, -depthM) : z;
    }

    /** Phase theta at a horizontal position and time, so that eta = A cos(theta). */
    public double phaseAt(double x, double z, double t) { return phase(x, z, t); }

    private double phase(double x, double z, double t) {
        return wavenumber() * (x * Math.cos(directionRadians) + z * Math.sin(directionRadians))
                - angularFrequency() * t + phaseRadians;
    }
}

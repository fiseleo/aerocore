package dev.mcmod.aerocore.api;

import java.util.List;

/** Linear superposition of independent Airy components; the summed sea must remain small-amplitude. */
public record WaveField(List<LinearWave> components) {
    public static final WaveField NONE = new WaveField(List.of());
    public WaveField { components = List.copyOf(components); }
    public boolean isEmpty() { return components.isEmpty(); }
    /** Upper bound of |eta|, used to widen water sampling around the still level. */
    public double maxAmplitude() { return components.stream().mapToDouble(LinearWave::amplitudeM).sum(); }
    public double elevation(double x, double z, double t) {
        double sum = 0;
        for (LinearWave wave : components) { sum += wave.elevation(x, z, t); }
        return sum;
    }
    /** Summed surface slope (d eta/dx, 0, d eta/dz). */
    public Vec3d slope(double x, double z, double t) {
        Vec3d sum = Vec3d.ZERO;
        for (LinearWave wave : components) { sum = sum.add(wave.slope(x, z, t)); }
        return sum;
    }
    /** Every component's amplitude multiplied by a factor in [0, 1]; 0 gives still water. */
    public WaveField scaled(double factor) {
        if (factor == 1) { return this; }
        if (factor == 0) { return NONE; }
        return new WaveField(components.stream().map(wave -> wave.scaled(factor)).toList());
    }
    /** Shortest component wavelength, or +infinity for still water. */
    public double shortestWavelength() {
        return components.stream().mapToDouble(LinearWave::wavelengthM).min().orElse(Double.POSITIVE_INFINITY);
    }
    public Vec3d velocity(Vec3d point, double stillWaterY, double t) {
        Vec3d sum = Vec3d.ZERO;
        for (LinearWave wave : components) { sum = sum.add(wave.velocity(point, stillWaterY, t)); }
        return sum;
    }
    public Vec3d acceleration(Vec3d point, double stillWaterY, double t) {
        Vec3d sum = Vec3d.ZERO;
        for (LinearWave wave : components) { sum = sum.add(wave.acceleration(point, stillWaterY, t)); }
        return sum;
    }
    /** Summed dynamic pressure head p_d/(rho g) in metres; see LinearWave. */
    public double dynamicPressureHead(Vec3d point, double stillWaterY, double t) {
        double sum = 0;
        for (LinearWave wave : components) { sum += wave.dynamicPressureHead(point, stillWaterY, t); }
        return sum;
    }
}

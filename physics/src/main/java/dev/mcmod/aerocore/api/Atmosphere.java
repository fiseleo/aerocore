package dev.mcmod.aerocore.api;

/** Dry standard atmosphere (ISA), geopotential altitude -500..32000 m (0.28: the 20-32 km layer added). Not a weather simulation. */
public final class Atmosphere {
    public static final double GRAVITY_MPS2 = 9.80665;
    private static final double GAS_CONSTANT = 287.05287;
    private static final double LAPSE = -0.0065, UPPER_LAPSE = 0.001;
    private Atmosphere() {}

    public record Sample(double temperatureKelvin, double pressurePa, FluidState fluid) {}

    public static Sample atAltitude(double geopotentialAltitudeM, Vec3d wind) {
        if (!Double.isFinite(geopotentialAltitudeM) || geopotentialAltitudeM < -500 || geopotentialAltitudeM > 32000) {
            throw new IllegalArgumentException("Geopotential altitude must be in [-500, 32000] m");
        }
        double h = Math.min(geopotentialAltitudeM, 11000);
        double temperature = 288.15 + LAPSE * h;
        double pressure = 101325 * Math.pow(temperature / 288.15, -GRAVITY_MPS2 / (GAS_CONSTANT * LAPSE));
        if (geopotentialAltitudeM > 11000) {
            pressure *= Math.exp(-GRAVITY_MPS2 * (Math.min(geopotentialAltitudeM, 20000) - 11000) / (GAS_CONSTANT * temperature));
        }
        if (geopotentialAltitudeM > 20000) {
            // the lower stratosphere warms at 1 K/km
            double t20 = temperature;
            temperature = t20 + UPPER_LAPSE * (geopotentialAltitudeM - 20000);
            pressure *= Math.pow(temperature / t20, -GRAVITY_MPS2 / (GAS_CONSTANT * UPPER_LAPSE));
        }
        double density = pressure / (GAS_CONSTANT * temperature);
        double viscosity = 1.458e-6 * Math.pow(temperature, 1.5) / (temperature + 110.4);
        return new Sample(temperature, pressure,
                new FluidState(density, viscosity, Math.sqrt(1.4 * GAS_CONSTANT * temperature), wind));
    }
}

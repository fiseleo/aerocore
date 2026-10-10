package dev.mcmod.aerocore.api;

/** Dry standard atmosphere, geopotential altitude -500..20000 m. Not a weather simulation. */
public final class Atmosphere {
    public static final double GRAVITY_MPS2 = 9.80665;
    private static final double GAS_CONSTANT = 287.05287;
    private static final double LAPSE = -0.0065;
    private Atmosphere() {}

    public record Sample(double temperatureKelvin, double pressurePa, FluidState fluid) {}

    public static Sample atAltitude(double geopotentialAltitudeM, Vec3d wind) {
        if (!Double.isFinite(geopotentialAltitudeM) || geopotentialAltitudeM < -500 || geopotentialAltitudeM > 20000) {
            throw new IllegalArgumentException("Geopotential altitude must be in [-500, 20000] m");
        }
        double h = Math.min(geopotentialAltitudeM, 11000);
        double temperature = 288.15 + LAPSE * h;
        double pressure = 101325 * Math.pow(temperature / 288.15, -GRAVITY_MPS2 / (GAS_CONSTANT * LAPSE));
        if (geopotentialAltitudeM > 11000) {
            pressure *= Math.exp(-GRAVITY_MPS2 * (geopotentialAltitudeM - 11000) / (GAS_CONSTANT * temperature));
        }
        double density = pressure / (GAS_CONSTANT * temperature);
        double viscosity = 1.458e-6 * Math.pow(temperature, 1.5) / (temperature + 110.4);
        return new Sample(temperature, pressure,
                new FluidState(density, viscosity, Math.sqrt(1.4 * GAS_CONSTANT * temperature), wind));
    }
}

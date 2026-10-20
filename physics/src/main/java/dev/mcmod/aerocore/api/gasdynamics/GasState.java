package dev.mcmod.aerocore.api.gasdynamics;

/** A stream's static temperature (K), static pressure (Pa) and speed (m/s); the gas gives the rest. */
public record GasState(double temperatureK, double pressurePa, double speedMps) {
    public GasState {
        if (!(temperatureK > 0) || !(pressurePa > 0) || !(speedMps >= 0) || !Double.isFinite(temperatureK) || !Double.isFinite(pressurePa)
                || !Double.isFinite(speedMps)) {
            throw new IllegalArgumentException("A gas state needs positive temperature and pressure and a finite nonnegative speed");
        }
    }

    public double density(Thermo gas) { return pressurePa / (gas.gasConstant() * temperatureK); }
    public double mach(Thermo gas) { return speedMps / gas.speedOfSound(temperatureK); }
    /** Mass flux rho V, kg/(s m^2). */
    public double massFlux(Thermo gas) { return density(gas) * speedMps; }
    /** Stagnation temperature: h(Tt) = h(T) + V^2 / 2, K. */
    public double totalTemperature(Thermo gas) { return gas.temperatureAtEnthalpy(gas.enthalpy(temperatureK) + 0.5 * speedMps * speedMps); }
    /** Stagnation pressure, isentropic: p exp((phi(Tt) - phi(T)) / R), Pa. */
    public double totalPressure(Thermo gas) {
        return pressurePa * Math.exp((gas.entropyFunction(totalTemperature(gas)) - gas.entropyFunction(temperatureK)) / gas.gasConstant());
    }
    /** Stream thrust per unit area, p + rho V^2, Pa. */
    public double impulsePerArea(Thermo gas) { return pressurePa + density(gas) * speedMps * speedMps; }
}

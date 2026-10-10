package dev.mcmod.aerocore.api;

import java.util.Objects;

/** Equilibrium material properties in SI; temperature is absolute, pressure is absolute. */
public record ThermalFluidState(double temperatureKelvin, double pressurePa, double specificHeatCp,
                                double thermalConductivityWPerMK, FluidState fluid) {
    public ThermalFluidState {
        Checks.positive(temperatureKelvin,"Temperature"); Checks.positive(pressurePa,"Pressure");
        Checks.positive(specificHeatCp,"Specific heat"); Checks.positive(thermalConductivityWPerMK,"Conductivity");
        Objects.requireNonNull(fluid);
    }
    public double prandtlNumber() { return specificHeatCp*fluid.dynamicViscosityPaS()/thermalConductivityWPerMK; }
}

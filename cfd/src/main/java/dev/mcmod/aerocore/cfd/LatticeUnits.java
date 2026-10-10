package dev.mcmod.aerocore.cfd;

import dev.mcmod.aerocore.api.FluidState;

/** SI conversion for a 2D lattice with reference density rho_lattice=1. Force is per unit depth. */
public record LatticeUnits(double cellSizeM, double timeStepSeconds, double referenceDensityKgPerM3) {
    public LatticeUnits {
        positive(cellSizeM, "Cell size");
        positive(timeStepSeconds, "Time step");
        positive(referenceDensityKgPerM3, "Reference density");
    }

    public static LatticeUnits forFluid(FluidState fluid, double cellSizeM, double latticeViscosity) {
        positive(fluid.densityKgPerM3(), "Density");
        positive(cellSizeM, "Cell size");
        positive(latticeViscosity, "Lattice viscosity");
        double nu = fluid.dynamicViscosityPaS() / fluid.densityKgPerM3();
        return new LatticeUnits(cellSizeM, latticeViscosity * cellSizeM * cellSizeM / nu, fluid.densityKgPerM3());
    }

    public double velocityMps(double latticeVelocity) { return latticeVelocity * cellSizeM / timeStepSeconds; }
    public double accelerationMps2(double latticeAcceleration) {
        return latticeAcceleration * cellSizeM / (timeStepSeconds * timeStepSeconds);
    }
    public double pressureDifferencePa(double latticeDensity) {
        double speedScale = cellSizeM / timeStepSeconds;
        return (latticeDensity - 1) / 3 * referenceDensityKgPerM3 * speedScale * speedScale;
    }
    public double forcePerDepthNpm(double latticeForce) {
        return latticeForce * referenceDensityKgPerM3 * Math.pow(cellSizeM, 3) / (timeStepSeconds * timeStepSeconds);
    }

    private static void positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0) { throw new IllegalArgumentException(name + " must be finite and positive"); }
    }
}

package dev.mcmod.aerocore.api;

/** Sectional speed, Reynolds number based on chord/characteristic length, and physical Mach. */
public record FlowConditions(double speedMps, double reynoldsNumber, double machNumber) {
    public static final FlowConditions ZERO = new FlowConditions(0, 0, 0);
    public FlowConditions {
        Checks.nonnegative(speedMps, "Speed");
        Checks.nonnegative(reynoldsNumber, "Reynolds number");
        Checks.nonnegative(machNumber, "Mach number");
    }
    public static FlowConditions of(FluidState fluid, double speedMps, double lengthM) {
        Checks.nonnegative(speedMps, "Speed");
        Checks.positive(lengthM, "Characteristic length");
        return new FlowConditions(speedMps,
                fluid.densityKgPerM3() * speedMps * lengthM / fluid.dynamicViscosityPaS(),
                speedMps / fluid.speedOfSoundMps());
    }
}

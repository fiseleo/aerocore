package dev.mcmod.aerocore.api.frame;

/** A frame member's material (0.33): density (kg/m^3), Young's modulus E and shear modulus G (Pa), yield strength (Pa). */
public record FrameMaterial(double densityKgM3, double modulusPa, double shearModulusPa, double yieldPa) {
    public FrameMaterial {
        Require.positive(densityKgM3, "Density"); Require.positive(modulusPa, "Young's modulus");
        Require.positive(shearModulusPa, "Shear modulus"); Require.positive(yieldPa, "Yield strength");
    }
}

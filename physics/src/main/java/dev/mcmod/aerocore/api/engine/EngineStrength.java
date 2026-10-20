package dev.mcmod.aerocore.api.engine;

/** Strength limits of piston-engine parts (SI units). */
public final class EngineStrength {
    private EngineStrength() {}

    /**
     * Peak cylinder pressure a flat piston crown carries at a stress (Pa), as a uniformly loaded, simply supported
     * circular plate: sigma_max = 3 (3 + nu) p a^2 / (8 t^2) at its centre (Roark, Formulas for Stress and Strain,
     * flat circular plates), so p = 8 t^2 sigma / (3 (3 + nu) a^2). Pa.
     */
    public static double crownPressureLimit(double pistonDiameter, double crownThickness, double allowableStressPa, double poisson) {
        double a = pistonDiameter / 2;
        return 8 * crownThickness * crownThickness * allowableStressPa / (3 * (3 + poisson) * a * a);
    }

    /**
     * Highest crank speed (rad/s) at which a rod beam carries the inertia of a reciprocating mass (kg) at its allowable
     * stress: the peak inertia force at top dead centre m r omega^2 (1 + r / l) over the beam area.
     */
    public static double rodSpeedLimit(double allowableStressPa, double beamArea, double reciprocatingMass, double crankRadius, double rodLength) {
        double force = allowableStressPa * beamArea;
        return Math.sqrt(force / (reciprocatingMass * crankRadius * (1 + crankRadius / rodLength)));
    }
}

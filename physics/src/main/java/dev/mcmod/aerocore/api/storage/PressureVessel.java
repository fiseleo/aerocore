package dev.mcmod.aerocore.api.storage;

/**
 * Wall thickness of thin pressure vessels (0.28): rocket motor cases and compressed-gas tanks (Roark's Formulas for
 * Stress and Strain, thin-walled cylinders and spheres). A metal cylinder carries the hoop stress p r / t; a sphere
 * p r / 2t. A filament-wound composite cylinder by netting analysis carries hoop and axial loads in its fibres, a total
 * of 1.5 p r / sigma of laminate (Peters, Composite Filament Winding, ch. 6), domes about half that.
 */
public final class PressureVessel {
    private PressureVessel() {}

    /** Wall of a metal cylinder for a burst pressure (working pressure times safety factor), m. */
    public static double metalCylinderWallM(double burstPressurePa, double radiusM, double ultimateStrengthPa) {
        check(burstPressurePa, radiusM, ultimateStrengthPa);
        return burstPressurePa * radiusM / ultimateStrengthPa;
    }

    /** Wall of a metal sphere (or hemispherical end) for a burst pressure, m. */
    public static double metalSphereWallM(double burstPressurePa, double radiusM, double ultimateStrengthPa) {
        check(burstPressurePa, radiusM, ultimateStrengthPa);
        return burstPressurePa * radiusM / (2 * ultimateStrengthPa);
    }

    /** Laminate of a filament-wound cylinder (hoop and helical, netting analysis) for a burst pressure, m. */
    public static double compositeCylinderWallM(double burstPressurePa, double radiusM, double laminateStrengthPa) {
        check(burstPressurePa, radiusM, laminateStrengthPa);
        return 1.5 * burstPressurePa * radiusM / laminateStrengthPa;
    }

    /** Burst pressure of a metal cylinder of a wall, Pa. */
    public static double metalCylinderBurstPa(double wallM, double radiusM, double ultimateStrengthPa) {
        check(wallM, radiusM, ultimateStrengthPa);
        return ultimateStrengthPa * wallM / radiusM;
    }

    /** Burst pressure of a filament-wound cylinder of a laminate, Pa. */
    public static double compositeCylinderBurstPa(double wallM, double radiusM, double laminateStrengthPa) {
        check(wallM, radiusM, laminateStrengthPa);
        return laminateStrengthPa * wallM / (1.5 * radiusM);
    }

    private static void check(double a, double r, double s) {
        if (!(a > 0) || !(r > 0) || !(s > 0)) { throw new IllegalArgumentException("Pressure vessel inputs must be positive"); }
    }
}

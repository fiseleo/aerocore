package dev.mcmod.aerocore.api.power;

/**
 * Line shafting (0.26): a steel shaft in plain bearings. Its strength in torsion: the power it carries at a speed is
 * tau pi d^3 / 16 omega for an allowable shear tau (about 28 MPa for line shafts carrying belt pulls, which bend them
 * too: Machinery's Handbook, "Transmission shafting"). Each bearing loses mu F r omega to friction (plain bearing, oil
 * lubricated: mu about 0.02, Shigley ch. 12), F the shaft's weight over its span plus the belts' pull.
 */
public final class Shafting {
    private Shafting() {}

    public static final double STEEL_DENSITY = 7850, ALLOWABLE_SHEAR_PA = 28e6, BEARING_MU = 0.02;

    /** Power a solid shaft of a diameter (m) transmits at an angular speed (rad/s), W. */
    public static double capacityW(double diameterM, double omegaRadS) {
        return ALLOWABLE_SHEAR_PA * Math.PI * Math.pow(diameterM, 3) / 16 * Math.abs(omegaRadS);
    }

    /** Mass of a solid steel shaft per metre, kg/m. */
    public static double kgPerM(double diameterM) { return STEEL_DENSITY * Math.PI * diameterM * diameterM / 4; }

    /** Polar moment of inertia of a solid steel shaft per metre, kg m^2/m. */
    public static double inertiaPerM(double diameterM) { return kgPerM(diameterM) * diameterM * diameterM / 8; }

    /** Friction loss of a plain bearing carrying a load (N) on a journal of a diameter (m) at a speed (rad/s), W. */
    public static double bearingLossW(double loadN, double journalM, double omegaRadS) {
        return BEARING_MU * Math.abs(loadN) * journalM / 2 * Math.abs(omegaRadS);
    }
}

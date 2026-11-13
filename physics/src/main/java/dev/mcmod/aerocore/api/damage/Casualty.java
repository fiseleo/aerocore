package dev.mcmod.aerocore.api.damage;

/**
 * 0.42: what a crash and a breach do to what a vehicle carries: the risk to its occupants from the change of velocity,
 * and the water a hole lets in. Formulas with typical constants; what a game makes of a risk (health lost) is its rule.
 */
public final class Casualty {
    private Casualty() {}

    /**
     * Change of velocity (m/s) at which half the belted front occupants of a car are seriously hurt (MAIS 3+) in a
     * frontal crash, and at which a tenth are: of the order of 60 to 65 km/h and 35 to 40 km/h in risk curves fitted to
     * the US crash samples (for example Kononen, Flannagan and Wang, Accident Analysis and Prevention 43, 2011). Round
     * figures from such curves, not one study's fit.
     */
    public static final double BELTED_FRONTAL_HALF_MPS = 63 / 3.6, BELTED_FRONTAL_TENTH_MPS = 38 / 3.6;

    /** A crash's change of velocity for those aboard, m/s: the impulse (N s) over the mass that took it (kg). */
    public static double deltaV(double impulseNs, double massKg) {
        if (!(impulseNs >= 0) || !(massKg > 0)) { throw new IllegalArgumentException("Impulse not negative, mass positive"); }
        return impulseNs / massKg;
    }

    /**
     * The risk (0..1) of serious injury at a change of velocity (m/s): a logistic curve through one half at halfMps and
     * one tenth at tenthMps (tenthMps below halfMps).
     */
    public static double seriousRisk(double deltaVMps, double halfMps, double tenthMps) {
        if (!(halfMps > tenthMps) || !(tenthMps >= 0)) { throw new IllegalArgumentException("The tenth-risk speed lies below the half-risk speed"); }
        double slope = Math.log(9) / (halfMps - tenthMps);
        return 1 / (1 + Math.exp(-slope * (Math.abs(deltaVMps) - halfMps)));
    }

    /** seriousRisk for a belted occupant in a frontal crash. */
    public static double seriousRisk(double deltaVMps) { return seriousRisk(deltaVMps, BELTED_FRONTAL_HALF_MPS, BELTED_FRONTAL_TENTH_MPS); }

    /** Discharge coefficient of a ragged hole in plating (a sharp-edged orifice's 0.6). */
    public static final double HOLE_DISCHARGE = 0.6;

    /**
     * Water coming in through a hole below the waterline, kg/s: Torricelli's orifice flow C_d A sqrt(2 g h) rho, with the
     * hole's area (m^2), its depth below the outside water (m; none above it), gravity (m/s^2) and the water's density
     * (kg/m^3). The level inside is taken as below the hole (the flow slows once it covers it, which this leaves out).
     */
    public static double inflowKgPerS(double areaM2, double headM, double gravityMps2, double densityKgPerM3) {
        if (!(areaM2 >= 0) || !(gravityMps2 >= 0) || !(densityKgPerM3 > 0)) { throw new IllegalArgumentException("Area and gravity not negative, density positive"); }
        return headM > 0 ? HOLE_DISCHARGE * areaM2 * Math.sqrt(2 * gravityMps2 * headM) * densityKgPerM3 : 0;
    }
}

package dev.mcmod.aerocore.api.damage;

/**
 * 0.42: how use wears things out, as shares of a life (0 new .. 1 worn out) to add to a part's condition. Formulas only;
 * the lives, volumes and rates of a given part are the caller's (typical values are named with where they come from).
 */
public final class Wear {
    private Wear() {}

    /**
     * Specific wear rate of an organic brake lining at moderate temperature, m^3 per J of friction work: linings lose
     * of the order of 10 to 30 mm^3/MJ (Limpert, Brake Design and Safety, 3rd ed., ch. 2; the middle of that range).
     */
    public static final double BRAKE_LINING_M3_PER_J = 20e-9 / 1e6;

    /** Specific wear rate of an organic clutch facing, m^3/J: of the same order as a brake lining's (Shaver, Manual Transmission Clutch Systems, SAE 1997). */
    public static final double CLUTCH_FACING_M3_PER_J = 15e-9 / 1e6;

    /**
     * Tread rubber worn per J of sliding friction at mild severity, m^3/J: abrasion on real road surfaces is of the order
     * of 10 to 100 mm^3 per kJ of frictional energy (Grosch, Rubber Chemistry and Technology 81, 2008); the low end
     * gives a car tyre a life of tens of thousands of km at the few kJ of slip work a km of ordinary driving costs.
     */
    public static final double TREAD_M3_PER_J = 20e-9 / 1e3;

    /**
     * The share of a friction lining (a brake pad, a clutch facing) worn by friction work (J): Archard's law in its
     * energy form, volume = specific wear rate (m^3/J) x work, over the lining's usable volume (m^3).
     */
    public static double lining(double frictionJ, double specificWearM3PerJ, double usableVolumeM3) {
        if (!(frictionJ >= 0) || !(specificWearM3PerJ >= 0) || !(usableVolumeM3 > 0)) { throw new IllegalArgumentException("Work and rate not negative, volume positive"); }
        return frictionJ * specificWearM3PerJ / usableVolumeM3;
    }

    /**
     * How much faster a lining wears hot: 1 up to the knee temperature (K), then doubling with every doublingK above it.
     * Organic linings wear at a nearly constant rate up to about 200 to 250 C and exponentially faster beyond, as the
     * resin binder breaks down (Limpert, ch. 2); the knee and the doubling step are the caller's lining.
     */
    public static double liningHeat(double temperatureK, double kneeK, double doublingK) {
        if (!(doublingK > 0)) { throw new IllegalArgumentException("Doubling step must be positive"); }
        return temperatureK <= kneeK || Double.isNaN(temperatureK) ? 1 : Math.pow(2, Math.min((temperatureK - kneeK) / doublingK, 20));
    }

    /**
     * The share of a tyre's tread worn by sliding friction work (J): abradability (m^3/J) x work x severity, over the
     * usable tread volume (m^3). The work is the slip part only (tyre force x sliding speed x time), not the rolling
     * resistance, which goes into the carcass as heat.
     */
    public static double tread(double slipJ, double abradabilityM3PerJ, double severity, double treadVolumeM3) {
        if (!(slipJ >= 0) || !(abradabilityM3PerJ >= 0) || !(severity >= 0) || !(treadVolumeM3 > 0)) { throw new IllegalArgumentException("Work, rate and severity not negative, volume positive"); }
        return slipJ * abradabilityM3PerJ * severity / treadVolumeM3;
    }

    /**
     * How much more rubber a J of sliding takes at a sliding speed (m/s): abrasion per unit energy climbs steeply once
     * the contact gets hot (Grosch 2008: a power of the energy dissipation rate between 1.5 and 3 in total). Taken as
     * 1 + (v / v0)^2, v0 the caller's reference (about 3 m/s puts a locked wheel at 30 m/s a hundred times above gentle
     * slip, which is how a flat spot or a burnout goes through a tread in minutes).
     */
    public static double treadSeverity(double slideMps, double referenceMps) {
        if (!(referenceMps > 0)) { throw new IllegalArgumentException("Reference speed must be positive"); }
        double r = Math.abs(slideMps) / referenceMps;
        return 1 + r * r;
    }

    /**
     * The share of a machine's life used in dt (s) when it lasts lifeS at its rated speed and load: dt / life x
     * (speed ratio)^speedExponent x (load ratio)^loadExponent. For rolling bearings the load exponent is 3 (balls) or
     * 10/3 (rollers) and the speed exponent 1 (a life in revolutions; ISO 281); piston-engine wear between overhauls
     * goes about as speed squared (mean piston speed) and with load.
     */
    public static double running(double dtS, double lifeS, double speedRatio, double speedExponent, double loadRatio, double loadExponent) {
        if (!(dtS >= 0) || !(lifeS > 0) || !(speedRatio >= 0) || !(loadRatio >= 0)) { throw new IllegalArgumentException("Time and ratios not negative, life positive"); }
        return dtS / lifeS * Math.pow(speedRatio, speedExponent) * Math.pow(loadRatio, loadExponent);
    }

    /** The Larson-Miller constant of steels and nickel alloys (log10 of hours). */
    public static final double LARSON_MILLER_C = 20;

    /**
     * The creep life (same unit as ratedLife) of a hot part at a temperature (K) when it lasts ratedLife hours at
     * ratedK under the same stress: the Larson-Miller parameter T (C + log10 t) is the same for both, so
     * log10 t = (ratedK / T)(C + log10 ratedHours) - C. Near 1200 K with C = 20 the life halves about every 15 K.
     */
    public static double creepLifeHours(double ratedHours, double ratedK, double temperatureK, double c) {
        if (!(ratedHours > 0) || !(ratedK > 0) || !(temperatureK > 0)) { throw new IllegalArgumentException("Life and temperatures must be positive"); }
        return Math.pow(10, ratedK / temperatureK * (c + Math.log10(ratedHours)) - c);
    }

    /** The share of a hot part's creep life used in dt (s) at a temperature (creepLifeHours). */
    public static double creep(double dtS, double ratedHours, double ratedK, double temperatureK, double c) {
        if (!(dtS >= 0)) { throw new IllegalArgumentException("Time must not be negative"); }
        return dtS / 3600 / creepLifeHours(ratedHours, ratedK, temperatureK, c);
    }

    /**
     * Miner's rule with Basquin's line: the share of a fatigue life used by cycles at a stress amplitude, when the part
     * lasts referenceCycles at referenceStress and its S-N line has the exponent b (N = N_ref (S_ref / S)^b; 3 to 5 for
     * welded steel, 8 to 12 for smooth machined parts). Below the endurance limit (same unit as the stresses) nothing.
     */
    public static double fatigue(double cycles, double stress, double enduranceLimit, double referenceStress, double referenceCycles, double b) {
        if (!(cycles >= 0) || !(referenceStress > 0) || !(referenceCycles > 0) || !(b > 0)) { throw new IllegalArgumentException("Cycles not negative; reference stress, cycles and exponent positive"); }
        double s = Math.abs(stress);
        if (s <= enduranceLimit || s == 0) { return 0; }
        return cycles / (referenceCycles * Math.pow(referenceStress / s, b));
    }
}

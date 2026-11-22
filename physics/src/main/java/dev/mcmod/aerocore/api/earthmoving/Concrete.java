package dev.mcmod.aerocore.api.earthmoving;

/**
 * Fresh concrete: setting and strength gain by maturity.
 *
 * <ul>
 * <li>Nurse-Saul maturity (ASTM C1074): M = sum (T - T_0) dt over the time the concrete is above the datum
 *     T_0 = -10 C, in K s here (the standard's C h times 3600). The equivalent age at the reference temperature
 *     T_r = 20 C is M / (T_r - T_0): a day at 20 C is 30 K x 86400 s. Below the datum nothing happens (in reality fresh
 *     concrete that freezes is damaged; not modelled).</li>
 * <li>Setting (ASTM C403 penetration resistance): ordinary portland-cement concrete without admixtures sets initially
 *     about 3 to 5 h and finally about 5 to 8 h after mixing at 20 to 23 C (Kosmatka and Wilson, Design and Control of
 *     Concrete Mixtures, PCA); taken as 4 h and 6 h at 20 C, so the maturities {@link #INITIAL_SET_KS} and
 *     {@link #FINAL_SET_KS}. At 5 C the same maturity takes twice as long.</li>
 * <li>Strength (ACI 209R-92, moist-cured type I cement): f(t) = f_28 t / (a + b t), a = 4.0 days, b = 0.85, with t the
 *     equivalent age counted from the final set (so the strength starts at zero there; at 28 days it is within 1 % of
 *     f_28; it tends to f_28 / 0.85).</li>
 * </ul>
 */
public final class Concrete {
    private Concrete() {}

    /** Datum temperature of the Nurse-Saul function, K (-10 C, ASTM C1074). */
    public static final double DATUM_K = 263.15;

    /** Reference temperature of the equivalent age, K (20 C). */
    public static final double REFERENCE_K = 293.15;

    /** Maturity at the initial set, K s: 4 h at 20 C. */
    public static final double INITIAL_SET_KS = (REFERENCE_K - DATUM_K) * 4 * 3600;

    /** Maturity at the final set, K s: 6 h at 20 C. */
    public static final double FINAL_SET_KS = (REFERENCE_K - DATUM_K) * 6 * 3600;

    /** ACI 209's a for moist-cured type I cement, days. */
    public static final double ACI209_A_DAYS = 4.0;

    /** ACI 209's b for moist-cured type I cement. */
    public static final double ACI209_B = 0.85;

    /** Maturity after a time step: the previous maturity (K s) plus max(0, T - T_0) dt for a temperature (K) held over dt (s). */
    public static double maturityKs(double previousKs, double temperatureK, double dtS) {
        if (!(previousKs >= 0) || !Double.isFinite(previousKs) || !(temperatureK > 0) || !Double.isFinite(temperatureK) || !(dtS >= 0) || !Double.isFinite(dtS)) {
            throw new IllegalArgumentException("Maturity and time finite and not negative, temperature positive");
        }
        return previousKs + Math.max(0, temperatureK - DATUM_K) * dtS;
    }

    /** Equivalent age at 20 C (s) of a maturity (K s): M / (T_r - T_0). */
    public static double equivalentAgeS(double maturityKs) {
        if (!(maturityKs >= 0) || !Double.isFinite(maturityKs)) { throw new IllegalArgumentException("Maturity finite and not negative"); }
        return maturityKs / (REFERENCE_K - DATUM_K);
    }

    /** Time (s) to reach a maturity (K s) at a constant temperature (K); infinite at or below the datum. */
    public static double secondsTo(double maturityKs, double temperatureK) {
        if (!(maturityKs >= 0) || !Double.isFinite(maturityKs) || !(temperatureK > 0) || !Double.isFinite(temperatureK)) {
            throw new IllegalArgumentException("Maturity finite and not negative, temperature positive");
        }
        double rate = temperatureK - DATUM_K;
        if (maturityKs == 0) { return 0; }
        return rate > 0 ? maturityKs / rate : Double.POSITIVE_INFINITY;
    }

    /** Whether concrete of a maturity (K s) has set initially (can no longer be placed or worked). */
    public static boolean initiallySet(double maturityKs) { return maturityKs >= INITIAL_SET_KS; }

    /** Whether concrete of a maturity (K s) has set finally (holds its shape, starts gaining strength). */
    public static boolean finallySet(double maturityKs) { return maturityKs >= FINAL_SET_KS; }

    /** Compressive strength (Pa) at a maturity (K s) of a concrete of a 28-day strength (Pa): ACI 209 on the equivalent age after the final set. */
    public static double strengthPa(double maturityKs, double strength28Pa) {
        if (!(strength28Pa >= 0) || !Double.isFinite(strength28Pa)) { throw new IllegalArgumentException("Strength finite and not negative"); }
        double days = Math.max(0, equivalentAgeS(maturityKs) - equivalentAgeS(FINAL_SET_KS)) / 86400;
        return strength28Pa * days / (ACI209_A_DAYS + ACI209_B * days);
    }
}

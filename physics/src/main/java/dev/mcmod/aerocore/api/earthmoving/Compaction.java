package dev.mcmod.aerocore.api.earthmoving;

/**
 * Compaction of fill by rollers, as relative compaction (dry density over the standard Proctor maximum, ASTM D698).
 *
 * <ul>
 * <li>Passes: each pass removes the same share of what is left to gain, RC_n = RC_lim - (RC_lim - RC_0) exp(-n / N_c),
 *     the exponential approach of the density-passes curves of field trials (Forssblad 1981, Vibratory Soil and Rock
 *     Fill Compaction, Dynapac: most of the gain in the first four to six passes, little after eight). N_c is set by the
 *     passes that take a loose lift to 95 %. Typical such passes for a 10 to 12 t vibratory roller in lifts of 0.2 to
 *     0.3 m are 4 to 8 (Forssblad; Caterpillar's compaction guides): {@link #PASSES_TO_95_GRANULAR} is the low end for
 *     sand and gravel under a smooth drum, {@link #PASSES_TO_95_COHESIVE} mid-range for silt and clay under a padfoot.</li>
 * <li>Other rollers and lifts: the passes scale with the lift thickness over the roller's effective linear load, the
 *     static linear load plus the centrifugal force per drum width (Forssblad: the depth a roller compacts grows about
 *     in proportion to its linear load). The dynamic part counts in full at {@link #REFERENCE_IMPACTS_PER_M} impacts
 *     per metre or more and in proportion below (faster travel or lower frequency spreads the blows). Both scalings
 *     are this model's assumptions around the typical figures.</li>
 * <li>Limit: never above 100 % of Proctor, and never above the zero-air-voids density at the moisture,
 *     rho_zav = rho_s / (1 + w rho_s / rho_w) (a wet soil cannot be squeezed past saturation). Passes never lower the
 *     density.</li>
 * </ul>
 */
public final class Compaction {
    private Compaction() {}

    /** Passes of the reference roller (smooth drum, vibrating) that take a loose 0.3 m lift of sand or gravel to 95 %. */
    public static final double PASSES_TO_95_GRANULAR = 4;

    /** Passes of the reference roller (padfoot, vibrating) that take a loose 0.2 m lift of silt or clay to 95 %. */
    public static final double PASSES_TO_95_COHESIVE = 6;

    /** Lift of the granular figure, m. */
    public static final double LIFT_GRANULAR_M = 0.3;

    /** Lift of the cohesive figure, m. */
    public static final double LIFT_COHESIVE_M = 0.2;

    /**
     * Effective linear load of the reference roller, N/m: a 10 to 12 t single-drum vibratory roller, static linear load
     * about 28 kN/m plus a centrifugal force of about 266 kN on a 2.13 m drum (125 kN/m), at 31 Hz (makers' data for
     * the class, e.g. Caterpillar's CS56).
     */
    public static final double REFERENCE_LINEAR_LOAD_N_PER_M = 28e3 + 266e3 / 2.13;

    /** Impacts per metre at which the dynamic force counts in full: 30 (the reference roller at 31 Hz and about 1 m/s). */
    public static final double REFERENCE_IMPACTS_PER_M = 30;

    /**
     * Effective linear load (N/m) of a roller: static linear load (N/m) plus centrifugal force (N, 0 for a static
     * roller) over the drum width (m), the dynamic part scaled by min(1, impacts per metre / 30) with impacts per metre
     * = frequency (Hz) / speed (m/s, sign ignored; standing still counts in full).
     */
    public static double linearLoadNPerM(double staticNPerM, double centrifugalN, double widthM, double frequencyHz, double speedMps) {
        if (!(staticNPerM >= 0) || !(centrifugalN >= 0) || !(widthM > 0) || !(frequencyHz >= 0) || !Double.isFinite(staticNPerM)
                || !Double.isFinite(centrifugalN) || !Double.isFinite(widthM) || !Double.isFinite(frequencyHz) || !Double.isFinite(speedMps)) {
            throw new IllegalArgumentException("Loads and frequency finite and not negative, width positive, speed finite");
        }
        double v = Math.abs(speedMps);
        double share = frequencyHz == 0 ? 0 : v * REFERENCE_IMPACTS_PER_M <= frequencyHz ? 1 : frequencyHz / (v * REFERENCE_IMPACTS_PER_M);
        return staticNPerM + centrifugalN / widthM * share;
    }

    /**
     * Passes to 95 % for a roller of an effective linear load (N/m, positive) on a lift (m), from a typical figure
     * (passes for the reference roller on its reference lift): typical x (lift / reference lift) x (reference load / load).
     */
    public static double passesTo95(double typicalPasses, double referenceLiftM, double liftM, double linearLoadNPerM) {
        if (!(typicalPasses > 0) || !(referenceLiftM > 0) || !(liftM >= 0) || !(linearLoadNPerM > 0) || !Double.isFinite(typicalPasses)
                || !Double.isFinite(liftM) || !Double.isFinite(linearLoadNPerM)) {
            throw new IllegalArgumentException("Passes, reference lift and load positive, lift not negative");
        }
        return typicalPasses * liftM / referenceLiftM * REFERENCE_LINEAR_LOAD_N_PER_M / linearLoadNPerM;
    }

    /**
     * The pass constant N_c (passes per e-fold of what is left to gain) of a roller that takes a lift from a loose
     * relative compaction (0..0.95 exclusive) to 95 % in a number of passes: N_95 / ln((1 - RC_loose) / 0.05).
     */
    public static double passConstant(double looseRelative, double passesTo95) {
        if (!(looseRelative > 0 && looseRelative < 0.95) || !(passesTo95 >= 0) || Double.isNaN(passesTo95)) {
            throw new IllegalArgumentException("Loose relative compaction within (0, 0.95), passes not negative");
        }
        return passesTo95 / Math.log((1 - looseRelative) / 0.05);
    }

    /**
     * Relative compaction after a number of passes (may be fractional: a part of the area rolled, or a substep's share
     * of a pass) from a relative compaction, with a pass constant (0 compacts at once to the limit, infinite never) and
     * a limit (0..1, {@link #limit}): lim - (lim - rc) exp(-n / N_c), and never below rc.
     */
    public static double afterPasses(double relative, double passes, double passConstant, double limit) {
        if (!(relative >= 0) || !Double.isFinite(relative) || !(passes >= 0) || !Double.isFinite(passes) || !(passConstant >= 0) || !(limit >= 0 && limit <= 1)) {
            throw new IllegalArgumentException("Relative compaction and passes finite and not negative, pass constant not negative, limit within [0, 1]");
        }
        if (relative >= limit) { return relative; }
        if (passConstant == 0) { return passes > 0 ? limit : relative; }
        return limit - (limit - relative) * Math.exp(-passes / passConstant);
    }

    /** Zero-air-voids dry density (kg/m^3) at a moisture (kg water per kg solids) with solids of a density (kg/m^3): rho_s / (1 + w rho_s / rho_w). */
    public static double zeroAirVoidsKgM3(double moisture, double solidsKgM3) {
        if (!(moisture >= 0) || !(solidsKgM3 > 0) || !Double.isFinite(moisture) || !Double.isFinite(solidsKgM3)) {
            throw new IllegalArgumentException("Moisture not negative, solids density positive");
        }
        return solidsKgM3 / (1 + moisture * solidsKgM3 / Material.WATER_KG_M3);
    }

    /** The limit of relative compaction (0..1) of a soil of a Proctor density (kg/m^3) at a moisture: min(1, rho_zav / rho_Proctor). */
    public static double limit(double proctorKgM3, double moisture, double solidsKgM3) {
        if (!(proctorKgM3 > 0) || !Double.isFinite(proctorKgM3)) { throw new IllegalArgumentException("Proctor density positive"); }
        return Math.min(1, zeroAirVoidsKgM3(moisture, solidsKgM3) / proctorKgM3);
    }

    /** The limit of a material at its own moisture ({@link Material#SOLIDS_KG_M3} solids). */
    public static double limit(Material m) { return limit(m.proctorKgM3(), m.moisture(), Material.SOLIDS_KG_M3); }

    /** Relative compaction of a material dug and dumped loose: loose density / (1 + moisture) / Proctor density. */
    public static double looseRelative(Material m) {
        if (!(m.proctorKgM3() > 0)) { throw new IllegalArgumentException(m.name() + " has no Proctor density"); }
        return m.looseKgM3() / (1 + m.moisture()) / m.proctorKgM3();
    }
}

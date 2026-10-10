package dev.mcmod.aerocore.api;

/**
 * The mean wind near the ground over any surface, from one reference wind: the 10 m wind over open, grassy terrain
 * (roughness length 0.03 m, the meteorological standard exposure). Neutral stratification throughout.
 * <ul>
 * <li>Roughness: Davenport's classification as revised by Wieringa (Davenport et al. 2000; WMO Guide No. 8, 2008,
 * Annex 5.B): sea, snow plains and featureless desert 0.0002 m (water itself by Charnock, SeaWind), smooth (beaches,
 * marsh, bare open country) 0.005, open (grass, isolated obstacles) 0.03, roughly open (low crops) 0.10, rough (high
 * crops, scattered obstacles) 0.25, very rough (bushland, orchards) 0.5, closed (mature forest, villages) 1.0, chaotic
 * (town centres) 2.0.</li>
 * <li>Profile: the logarithmic law U(z) = (u* / kappa) ln((z - d) / z0) with zero-plane displacement d. A change of
 * surface keeps the wind at the blending height (60 m, Wieringa 1986; Mason 1988) and adapts the profile below it to
 * the local roughness; above 60 m the reference profile continues to 200 m and stays constant higher (the Ekman layer's
 * slower growth and turning are not modelled).</li>
 * <li>Forest canopy (Raupach 1994 rules of thumb): d = 0.7 h and z0 = 0.1 h for a canopy of height h; inside the canopy
 * the wind decays as U(h) exp(a (z / h - 1)), a = 2 (Cionco 1965, attenuation coefficients 1-4 for forests).</li>
 * <li>Gusts: turbulence intensity I_u = 1 / ln((z - d) / z0) (EN 1991-1-4 eq. 4.7, turbulence factor 1), i.e.
 * sigma_u = 2.5 u*.</li>
 * <li>Hills: Taylor and Lee (1984, Climatological Bulletin 18:3-22) fractional speed-up at a crest
 * dS = B H / L_h, decaying with height as exp(-A z / L_h), for hill height H and the upwind distance L_h from the crest
 * to half height: B = 1.6, A = 4 for three-dimensional hills (2.0 and 3 for long ridges, 0.8 and 2.5 for
 * escarpments); H / L_h is held at 0.5, where flow separation begins to cap it.</li>
 * </ul>
 */
public final class SurfaceWind {
    public static final double KARMAN = SeaWind.KARMAN;
    /** Roughness of the reference exposure, m. */
    public static final double REFERENCE_ROUGHNESS = 0.03;
    /** Davenport-Wieringa class roughness lengths, m. */
    public static final double SEA = 0.0002, SMOOTH = 0.005, OPEN = 0.03, ROUGHLY_OPEN = 0.10, ROUGH = 0.25, VERY_ROUGH = 0.5, CLOSED = 1.0, CHAOTIC = 2.0;
    /** Height where the wind no longer feels a local change of surface, m. */
    public static final double BLENDING_HEIGHT = 60;
    /** Height above which the mean wind is held constant, m. */
    public static final double TOP_HEIGHT = 200;
    /** Canopy attenuation coefficient (Cionco). */
    public static final double CANOPY_ATTENUATION = 2;
    /** Taylor and Lee's crest factor and decay for three-dimensional hills, and the slope cap. */
    public static final double HILL_B = 1.6, HILL_A = 4, MAX_HILL_SLOPE = 0.5;
    private static final double LOWEST = 0.1;

    private SurfaceWind() {}

    /** Wind of the reference profile (open terrain) at height z above the ground, for its 10 m speed. */
    public static double reference(double u10Open, double heightM) {
        double z = Math.clamp(heightM, LOWEST, TOP_HEIGHT);
        return u10Open * Math.log(z / REFERENCE_ROUGHNESS) / Math.log(10 / REFERENCE_ROUGHNESS);
    }

    /**
     * Mean wind speed (m/s) at height z (m) above the ground over a surface of roughness z0 (m) with zero-plane
     * displacement d (m; 0.7 h in a forest) and canopy height h (m; 0 without), for a 10 m reference wind over open
     * terrain. Below the canopy top the exponential canopy profile; at or above the blending height the reference profile.
     */
    public static double speed(double u10Open, double heightM, double z0, double displacementM, double canopyM) {
        if (!(u10Open > 0)) { return 0; }
        if (heightM >= BLENDING_HEIGHT) { return reference(u10Open, heightM); }
        double blend = reference(u10Open, BLENDING_HEIGHT), scale = blend / Math.log((BLENDING_HEIGHT - displacementM) / z0);
        if (canopyM > 0 && heightM < canopyM) {
            double top = scale * Math.log(Math.max(canopyM - displacementM, z0 * Math.E) / z0);
            return top * Math.exp(CANOPY_ATTENUATION * (Math.max(heightM, 0) / canopyM - 1));
        }
        double above = Math.max(heightM - displacementM, z0 * Math.E);
        return scale * Math.log(above / z0);
    }

    /** Friction velocity of the local profile under a 10 m reference wind, m/s. */
    public static double frictionVelocity(double u10Open, double z0, double displacementM) {
        return KARMAN * reference(u10Open, BLENDING_HEIGHT) / Math.log((BLENDING_HEIGHT - displacementM) / z0);
    }

    /**
     * The 10 m wind over open water (m/s) that has the same blending-height wind as the reference: Charnock's sea
     * roughness (SeaWind.roughness) found by fixed-point iteration.
     */
    public static double overWater(double u10Open) {
        if (!(u10Open > 0)) { return 0; }
        // Still air (SeaWind.LIGHT_AIR): the sea's smooth-flow roughness has no solution; scale linearly as SeaWind does.
        if (u10Open < SeaWind.LIGHT_AIR) { return u10Open / SeaWind.LIGHT_AIR * overWater(SeaWind.LIGHT_AIR); }
        double blend = reference(u10Open, BLENDING_HEIGHT), u10 = u10Open;
        for (int i = 0; i < 30; i++) {
            double z0 = SeaWind.roughness(SeaWind.frictionVelocity(Math.min(u10, SeaWind.MAX_WIND)), Math.min(u10, SeaWind.MAX_WIND));
            double next = blend * Math.log(10 / z0) / Math.log(BLENDING_HEIGHT / z0);
            if (Math.abs(next - u10) < 1e-9) { return next; }
            u10 = next;
        }
        return u10;
    }

    /** Longitudinal turbulence intensity at height z over roughness z0 with displacement d (EN 1991-1-4). */
    public static double intensity(double heightM, double z0, double displacementM) {
        double above = Math.max(heightM - displacementM, z0 * Math.E);
        return 1 / Math.log(Math.min(above, TOP_HEIGHT) / z0);
    }

    /**
     * Fractional speed-up over a hill (Taylor and Lee, three-dimensional hill): hill height H (m) above its upwind base,
     * L_h (m) from the crest upwind to half height, height above the local ground z (m), and the share of a crest the
     * point is (1 at the crest, falling to 0 down its upwind slope, a rule for points between base and crest).
     */
    public static double hillSpeedUp(double hillHeightM, double halfLengthM, double heightM, double crestShare) {
        if (!(hillHeightM > 0) || !(halfLengthM > 0)) { return 0; }
        double slope = Math.min(hillHeightM / halfLengthM, MAX_HILL_SLOPE);
        return Math.clamp(crestShare, 0, 1) * HILL_B * slope * Math.exp(-HILL_A * Math.max(heightM, 0) / halfLengthM);
    }

    /** Roughness that the logarithmic mean of a surface patchwork gives: exp(sum w ln z0 / sum w). */
    public static double effectiveRoughness(double[] z0, double[] weights) {
        double sum = 0, total = 0;
        for (int i = 0; i < z0.length; i++) { if (weights[i] > 0 && z0[i] > 0) { sum += weights[i] * Math.log(z0[i]); total += weights[i]; } }
        return total > 0 ? Math.exp(sum / total) : OPEN;
    }

    /**
     * A slowly varying synoptic wind: the weather's speed times a factor 1 + 0.25 X(t) and a direction wandering
     * 0.7 rad about a prevailing one, X and the direction's wander each a sum of three sinusoids with periods of about
     * 3, 1.1 and 0.37 Minecraft days (3600, 1320 and 444 s) and random phases from the seed. The periods, spreads and the
     * weather speeds are a rule (Minecraft has no meteorology); real synoptic winds change over days with fronts.
     */
    public static final class Synoptic {
        private static final double[] PERIODS = {3600, 1320, 444}, WEIGHTS = {0.6, 0.3, 0.1};
        private final double prevailing;
        private final double[] speedPhase = new double[3], directionPhase = new double[3];

        public Synoptic(long seed) {
            java.util.SplittableRandom random = new java.util.SplittableRandom(seed);
            prevailing = random.nextDouble() * 2 * Math.PI;
            for (int i = 0; i < 3; i++) { speedPhase[i] = random.nextDouble() * 2 * Math.PI; directionPhase[i] = random.nextDouble() * 2 * Math.PI; }
        }

        private static double wander(double t, double[] phase) {
            double sum = 0;
            for (int i = 0; i < 3; i++) { sum += WEIGHTS[i] * Math.sin(2 * Math.PI * t / PERIODS[i] + phase[i]); }
            return sum;
        }

        /** Speed factor at game time t (s), between 0.75 and 1.25. */
        public double speedFactor(double t) { return 1 + 0.25 * wander(t, speedPhase); }

        /** Direction the wind blows toward at game time t (s), rad. */
        public double direction(double t) { return prevailing + 0.7 * wander(t, directionPhase); }
    }
}

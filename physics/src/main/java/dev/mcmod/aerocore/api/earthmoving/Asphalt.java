package dev.mcmod.aerocore.api.earthmoving;

/**
 * Hot-mix asphalt cooling: a freshly laid mat until it is too cold to compact, and a load in a truck bed.
 *
 * <p>One-dimensional conduction through the layer (and, for a mat, the base below it), implicit in time, as in
 * PaveCool (Chadbourn, Luoma, Newcomb and Voller 1996, Consideration of hot-mix asphalt thermal properties during
 * compaction, ASTM STP 1287; MnDOT report 1998-18) and its successor MultiCool, which go back to Corlew and Dickson
 * (1968, Methods for calculating temperature profiles of hot-mix asphalt concrete as related to the construction of
 * asphalt pavements, AAPT 37). The surface loses heat by convection, h = 5.7 + 3.8 V W/(m^2 K) with the wind V in m/s
 * (McAdams 1954, the flat-plate wind coefficient), and by radiation to a clear sky at T_sky = 0.0552 T_air^1.5
 * (Swinbank 1963), emissivity 0.95; it gains the absorbed sunshine (absorptivity 0.90). Mix: conductivity 1.21
 * W/(m K) and specific heat 921 J/(kg K) (Corlew and Dickson's 0.70 BTU/(h ft F) and 0.22 BTU/(lb F)), 2240 kg/m^3
 * behind the screed; the base is given the mix's properties and sits on its own temperature 0.5 m down. Rolling has
 * to stop at the cessation temperature, 175 F (79.4 C, PaveCool's default) of the mat's mean.
 *
 * <p>Result: a 50 mm mat laid at 145 C on a 10 C base in 10 C air and a 4 m/s wind reaches 79.4 C in about 20 minutes;
 * 25 mm in about 7, 75 mm in about 37 (the cold-weather paving guides after Corlew and Dickson give about 15 minutes
 * for a 50 mm lift on a 10 to 15 C base, 4 to 8 for 25 mm). A metre-deep truck load at 160 C is still above 150 C after
 * an hour: it cools in its top and bottom crust.
 */
public final class Asphalt {
    private Asphalt() {}

    /** Thermal conductivity of hot mix, W/(m K). */
    public static final double CONDUCTIVITY_W_PER_M_K = 1.21;

    /** Specific heat of hot mix, J/(kg K). */
    public static final double SPECIFIC_HEAT_J_PER_KG_K = 921;

    /** Density of a mat behind the screed, kg/m^3. */
    public static final double MAT_DENSITY_KG_M3 = 2240;

    /** Long-wave emissivity of the mat. */
    public static final double EMISSIVITY = 0.95;

    /** Solar absorptivity of the black mat. */
    public static final double SOLAR_ABSORPTIVITY = 0.90;

    /** A typical laying temperature, K (150 C; mixes are laid at about 135 to 160 C). */
    public static final double LAYING_K = 423.15;

    /** Cessation temperature, K: 175 F (79.4 C), below which rolling no longer densifies the mat. */
    public static final double CESSATION_K = 273.15 + (175 - 32) / 1.8;

    /** Depth of the base below a mat that keeps its own temperature, m. */
    public static final double BASE_DEPTH_M = 0.5;

    /** Time step of the built-in runs, s. */
    public static final double STEP_S = 5;

    /** Longest run of {@link #secondsToK}, s (a day): what it returns when the target is never reached. */
    public static final double MAX_S = 86400;

    private static final double SIGMA = 5.670374419e-8;

    /** Convective heat transfer coefficient of a surface in a wind (m/s, sign ignored), W/(m^2 K): 5.7 + 3.8 V (McAdams). */
    public static double convectionWPerM2K(double windMps) {
        if (!Double.isFinite(windMps)) { throw new IllegalArgumentException("Wind must be finite"); }
        return 5.7 + 3.8 * Math.abs(windMps);
    }

    /** Clear-sky temperature (K) over air of a temperature (K): 0.0552 T^1.5 (Swinbank 1963). */
    public static double skyK(double airK) {
        if (!(airK > 0) || !Double.isFinite(airK)) { throw new IllegalArgumentException("Air temperature must be positive"); }
        return 0.0552 * Math.pow(airK, 1.5);
    }

    /**
     * A cooling layer of hot mix: a mat on its base, or a load in a truck bed. Mutable; one per layer. Cells of the mix
     * from the top down, then (for a mat) base cells growing by 15 % to {@link #BASE_DEPTH_M}.
     */
    public static final class Layer {
        private final double[] t, dz, k, rc, lo, mid, up, rhs, cp, dp;
        private final int mixCells;
        private final double bottomK, topR, bedR;
        private final boolean onBed;
        private double surfaceK, elapsedS;

        private Layer(double thicknessM, double startK, int mixCells, double baseK, boolean onBed, double topR, double bedR) {
            if (!(thicknessM > 0) || !Double.isFinite(thicknessM) || !(startK > 0) || !Double.isFinite(startK) || !(baseK > 0) || !Double.isFinite(baseK)
                    || !(topR >= 0) || !(bedR >= 0) || !Double.isFinite(topR) || !Double.isFinite(bedR)) {
                throw new IllegalArgumentException("Thickness and temperatures positive and finite, resistances not negative");
            }
            double cell = thicknessM / mixCells;
            int baseCells = 0;
            if (!onBed) {
                for (double depth = 0, size = cell; depth < BASE_DEPTH_M; size *= 1.15) { depth += size; baseCells++; }
            }
            int n = mixCells + baseCells;
            t = new double[n]; dz = new double[n]; k = new double[n]; rc = new double[n];
            lo = new double[n]; mid = new double[n]; up = new double[n]; rhs = new double[n]; cp = new double[n]; dp = new double[n];
            double size = cell;
            for (int i = 0; i < n; i++) {
                boolean mix = i < mixCells;
                dz[i] = mix ? cell : size;
                if (!mix) { size *= 1.15; }
                k[i] = CONDUCTIVITY_W_PER_M_K;
                rc[i] = MAT_DENSITY_KG_M3 * SPECIFIC_HEAT_J_PER_KG_K;
                t[i] = mix ? startK : baseK;
            }
            this.mixCells = mixCells;
            this.bottomK = baseK;
            this.onBed = onBed;
            this.topR = topR;
            this.bedR = bedR;
            surfaceK = startK;
        }

        /**
         * Advance by a time step (s) in air of a temperature (K), a wind (m/s) and sunshine on the surface (W/m^2):
         * implicit (backward Euler) conduction with the surface radiation linearised at the last surface temperature.
         * Any step is stable; steps of a few seconds keep it accurate (5 s is within 1 % of 1 s).
         */
        public void step(double dtS, double airK, double windMps, double solarWPerM2) {
            if (!(dtS >= 0) || !Double.isFinite(dtS) || !(solarWPerM2 >= 0) || !Double.isFinite(solarWPerM2)) {
                throw new IllegalArgumentException("Time step and sunshine finite and not negative");
            }
            if (dtS == 0) { return; }
            double hc = convectionWPerM2K(windMps), sky = skyK(airK);
            double hr = EMISSIVITY * SIGMA * (surfaceK * surfaceK + sky * sky) * (surfaceK + sky);
            double h = hc + hr, ambient = (hc * airK + hr * sky) / h;
            double r1 = dz[0] / (2 * k[0]) + topR, uTop = 1 / (r1 + 1 / h), sun = SOLAR_ABSORPTIVITY * solarWPerM2 / (1 + h * r1);
            int n = t.length;
            for (int i = 0; i < n; i++) {
                double c = rc[i] * dz[i] / dtS;
                lo[i] = 0; up[i] = 0; mid[i] = c; rhs[i] = c * t[i];
                if (i > 0) { double g = conductance(i - 1); lo[i] = -g; mid[i] += g; }
                if (i < n - 1) { double g = conductance(i); up[i] = -g; mid[i] += g; }
            }
            mid[0] += uTop;
            rhs[0] += uTop * ambient + sun;
            if (onBed) {
                double last = t[n - 1], hb = hc + EMISSIVITY * SIGMA * (last * last + airK * airK) * (last + airK);
                double ub = 1 / (dz[n - 1] / (2 * k[n - 1]) + bedR + 1 / hb);
                mid[n - 1] += ub; rhs[n - 1] += ub * airK;
            } else {
                double ub = 2 * k[n - 1] / dz[n - 1];
                mid[n - 1] += ub; rhs[n - 1] += ub * bottomK;
            }
            // Thomas algorithm.
            cp[0] = up[0] / mid[0]; dp[0] = rhs[0] / mid[0];
            for (int i = 1; i < n; i++) {
                double m = mid[i] - lo[i] * cp[i - 1];
                cp[i] = up[i] / m;
                dp[i] = (rhs[i] - lo[i] * dp[i - 1]) / m;
            }
            t[n - 1] = dp[n - 1];
            for (int i = n - 2; i >= 0; i--) { t[i] = dp[i] - cp[i] * t[i + 1]; }
            surfaceK = (t[0] / r1 + SOLAR_ABSORPTIVITY * solarWPerM2 + h * ambient) / (1 / r1 + h);
            elapsedS += dtS;
        }

        private double conductance(int i) { return 1 / (dz[i] / (2 * k[i]) + dz[i + 1] / (2 * k[i + 1])); }

        /** Mean temperature of the mix, K (what the cessation temperature applies to). */
        public double meanK() {
            double sum = 0;
            for (int i = 0; i < mixCells; i++) { sum += t[i]; }
            return sum / mixCells;
        }

        /** Temperature of the exposed surface (under a tarp: of the mix's top), K. */
        public double surfaceK() { return surfaceK; }

        /** Time stepped so far, s. */
        public double elapsedS() { return elapsedS; }
    }

    /** A mat of a thickness (m) laid at a temperature (K) on a base at a temperature (K): 10 mix cells. */
    public static Layer mat(double thicknessM, double layingK, double baseK) {
        return new Layer(thicknessM, layingK, 10, baseK, false, 0, 0);
    }

    /**
     * A load of a depth (m) at a temperature (K) in a truck bed, cells of about 20 mm (at least 10): the top open to the
     * air through a tarp's resistance (m^2 K/W, 0 for none; a canvas tarp with its trapped air is of the order of
     * 0.1), the bottom through the bed's (0 for bare steel, more for an insulated bed) to air below. Sides ignored.
     */
    public static Layer load(double depthM, double loadK, double tarpM2KPerW, double bedM2KPerW) {
        if (!(depthM > 0) || !Double.isFinite(depthM)) { throw new IllegalArgumentException("Depth must be positive"); }
        int cells = (int) Math.max(10, Math.min(200, Math.ceil(depthM / 0.02)));
        return new Layer(depthM, loadK, cells, loadK, true, tarpM2KPerW, bedM2KPerW);
    }

    /**
     * Time (s) for a mat (thickness m, laid at K, on a base at K) to cool to a mean temperature (K) in air (K), wind
     * (m/s) and sunshine (W/m^2) held constant: stepped by {@link #STEP_S} and interpolated within the last step, so it
     * is continuous in every input. 0 when laid at or below the target; {@link #MAX_S} when not reached in a day.
     */
    public static double secondsToK(double thicknessM, double layingK, double baseK, double airK, double windMps, double solarWPerM2, double targetK) {
        if (!(targetK > 0) || !Double.isFinite(targetK)) { throw new IllegalArgumentException("Target temperature must be positive"); }
        Layer mat = mat(thicknessM, layingK, baseK);
        double before = mat.meanK();
        if (before <= targetK) { return 0; }
        while (mat.elapsedS() < MAX_S) {
            mat.step(STEP_S, airK, windMps, solarWPerM2);
            double after = mat.meanK();
            if (after <= targetK) { return mat.elapsedS() - STEP_S * (targetK - after) / (before - after); }
            before = after;
        }
        return MAX_S;
    }

    /** Mean temperature (K) of a mat (as {@link #secondsToK}) after a time (s). */
    public static double matMeanK(double thicknessM, double layingK, double baseK, double airK, double windMps, double solarWPerM2, double seconds) {
        return run(mat(thicknessM, layingK, baseK), airK, windMps, solarWPerM2, seconds);
    }

    /** Mean temperature (K) of a truck load (as {@link #load}) after a time (s) in air (K), wind (m/s) and sunshine (W/m^2). */
    public static double loadMeanK(double depthM, double loadK, double tarpM2KPerW, double bedM2KPerW, double airK, double windMps, double solarWPerM2,
                                   double seconds) {
        return run(load(depthM, loadK, tarpM2KPerW, bedM2KPerW), airK, windMps, solarWPerM2, seconds);
    }

    private static double run(Layer layer, double airK, double windMps, double solarWPerM2, double seconds) {
        if (!(seconds >= 0) || !(seconds <= 30 * MAX_S)) { throw new IllegalArgumentException("Time within [0, 30 days]"); }
        int steps = (int) Math.ceil(seconds / STEP_S);
        for (int i = 0; i < steps; i++) { layer.step(seconds / steps, airK, windMps, solarWPerM2); }
        return layer.meanK();
    }
}

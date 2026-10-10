package dev.mcmod.aerocore.api;

/**
 * Where a resolved sea breaks and how long its foam stays.
 * <ul>
 * <li>Breaking: a surface particle accelerating downward faster than a threshold (Snyder and Kennedy 1983; the
 * smooth-crested limit is 0.388 g, Longuet-Higgins and Fox 1977), the linear vertical acceleration at the point
 * standing in for the particle's. Linear accelerations of a spectrum cut at 5 omega_p are no measure of that limit
 * (an open-ocean sea's rarely reach it; a short-fetch sea at the steepness limit has deviations near a quarter of g,
 * carried by centimetre waves that micro-break without whitecaps), so the threshold is set so the Gaussian fraction of
 * surface beyond it equals the actively breaking whitecap fraction W_A, whatever acceleration that takes. Until 0.14
 * it was capped at 0.388 g, which made such a pond foam over tens of times Monahan's coverage.</li>
 * <li>W_A is a tenth of the whitecap coverage W (stage A crests against stage B foam, Monahan and Lu 1990), and
 * W = 3.84e-6 U10^3.41 (Monahan and O'Muircheartaigh 1980, data to 16.6 m/s). The wind is the one that raises the
 * sea's Hs when fully developed (Pierson-Moskowitz: Hs = 0.2092 U19.5^2 / g, U19.5 = 1.05 U10, the middle of
 * 1.026-1.075); a sea younger or older than that gets that equivalent wind's whitecaps, unless the wind over it is
 * known (thresholdAtWind).</li>
 * <li>Foam decays exponentially with e-folding time 3.5 s (Monahan 1988; 3.53 s, Monahan et al. 1982; single
 * patches 0.2-10.4 s, Callaghan et al. 2012).</li>
 * </ul>
 */
public final class Whitecaps {
    /** Foam e-folding time, s. */
    public static final double FOAM_DECAY_S = 3.5;
    /** Downward acceleration of the almost-highest smooth wave's crest, as a fraction of g. */
    public static final double LIMIT = 0.388;
    /** Upper end of Monahan and O'Muircheartaigh's data, m/s. */
    public static final double MAX_WIND = 16.6;
    private Whitecaps() {}

    /** 10 m wind of a fully developed sea of this significant height, m/s. */
    public static double equivalentWind(double significantHeightM) {
        Checks.nonnegative(significantHeightM, "Significant height");
        return Math.sqrt(Atmosphere.GRAVITY_MPS2 * significantHeightM / 0.2092) / 1.05;
    }

    /** Whitecap coverage (fraction) at a 10 m wind, held at the data's upper end beyond it. */
    public static double coverage(double windMps) {
        Checks.nonnegative(windMps, "Wind speed");
        return Math.min(1, 3.84e-6 * Math.pow(Math.min(windMps, MAX_WIND), 3.41));
    }

    /**
     * Downward surface acceleration (m/s^2, positive) beyond which the sea breaks: the level exceeded by the active
     * fraction of a Gaussian acceleration field with this deviation (CompiledWaves.accelerationDeviation). Infinite
     * when nothing breaks.
     */
    public static double threshold(double accelerationDeviation, double significantHeightM) {
        return thresholdAtWind(accelerationDeviation, equivalentWind(significantHeightM));
    }

    /** threshold for a sea under a known 10 m wind (m/s), whose whitecap coverage it sets. */
    public static double thresholdAtWind(double accelerationDeviation, double windMps) {
        double active = 0.1 * coverage(windMps);
        // Below 1e-12 the breaking share is nil (and 1 - active would round to 1).
        if (!(active > 1e-12) || !(accelerationDeviation > 0)) { return Double.POSITIVE_INFINITY; }
        return accelerationDeviation * inverseNormal(1 - active);
    }

    /** Inverse standard normal CDF (Acklam 2003 rational approximation, relative error below 1.2e-9). */
    public static double inverseNormal(double p) {
        if (!(p > 0 && p < 1)) { throw new IllegalArgumentException("Probability must be in (0, 1)"); }
        double[] a = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
        double[] b = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01};
        double[] c = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
        double[] d = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00};
        double low = 0.02425;
        if (p < low) {
            double q = Math.sqrt(-2 * Math.log(p));
            return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
        }
        if (p > 1 - low) {
            double q = Math.sqrt(-2 * Math.log(1 - p));
            return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
        }
        double q = p - 0.5, r = q * q;
        return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
    }

    /** Foam after dt seconds: the old foam decayed, or full foam where the surface breaks now. */
    public static double foam(double previous, boolean breaking, double dt) {
        return breaking ? 1 : previous * Math.exp(-dt / FOAM_DECAY_S);
    }
}

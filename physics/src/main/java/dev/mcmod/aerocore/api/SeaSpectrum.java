package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Irregular short-crested seas as a superposition of linear components. Frequency spectrum: JONSWAP as in
 * DNV-RP-C205 (2010) 3.5.5, S(omega) = A_gamma S_PM(omega) gamma^exp(-((omega - omega_p)/(sigma omega_p))^2 / 2) with the
 * Pierson-Moskowitz S_PM = (5/16) Hs^2 omega_p^4 omega^-5 exp(-(5/4)(omega/omega_p)^-4), A_gamma = 1 - 0.287 ln gamma,
 * sigma = 0.07 below and 0.09 above the peak. Directional spreading: D(theta) = Gamma(1 + n/2) / (sqrt(pi) Gamma(1/2 + n/2))
 * cos^n(theta - theta_p) for |theta - theta_p| <= pi/2 (DNV-RP-C205 3.5.8). Synthesis is single-summation: each
 * frequency bin gets one component with a jittered frequency, a direction drawn from D and a random phase, so the
 * sea is short-crested without phase-locked directional pairs. Linear theory: no breaking, no nonlinear crests.
 */
public final class SeaSpectrum {
    /**
     * Synthesis range in multiples of the peak frequency: it holds about 99 % of the JONSWAP variance (the omega^-5
     * tail above 5 omega_p carries under 1 %), but omits short chop below 1/25 of the peak wavelength.
     */
    public static final double LOW = 0.5, HIGH = 5.0;
    private SeaSpectrum() {}

    /** DNV-RP-C205 recommended peak-enhancement factor from Tp / sqrt(Hs) (s / m^0.5). */
    public static double recommendedGamma(double significantHeightM, double peakPeriodS) {
        Checks.positive(significantHeightM, "Significant height"); Checks.positive(peakPeriodS, "Peak period");
        double ratio = peakPeriodS / Math.sqrt(significantHeightM);
        if (ratio <= 3.6) { return 5; }
        if (ratio >= 5) { return 1; }
        return Math.exp(5.75 - 1.15 * ratio);
    }

    /** JONSWAP spectral density S(omega), m^2 s, for omega > 0 (rad/s). */
    public static double jonswap(double omega, double significantHeightM, double peakPeriodS, double gamma) {
        if (!(omega > 0)) { return 0; }
        if (!(gamma >= 1) || gamma > 10) { throw new IllegalArgumentException("Peak enhancement must be in [1, 10]"); }
        double peak = 2 * Math.PI / peakPeriodS, ratio = omega / peak;
        double pm = 5.0 / 16 * significantHeightM * significantHeightM * Math.pow(peak, 4) * Math.pow(omega, -5) * Math.exp(-1.25 * Math.pow(ratio, -4));
        double sigma = omega <= peak ? 0.07 : 0.09, width = (omega - peak) / (sigma * peak);
        return (1 - 0.287 * Math.log(gamma)) * pm * Math.pow(gamma, Math.exp(-0.5 * width * width));
    }

    /** Normalised cos^n spreading density (per radian) at an angle from the mean direction; zero beyond +-pi/2. */
    public static double spreading(double angleFromMean, double exponent) {
        Checks.positive(exponent, "Spreading exponent");
        if (Math.abs(angleFromMean) > Math.PI / 2) { return 0; }
        return Math.exp(logGamma(1 + exponent / 2) - logGamma(0.5 + exponent / 2)) / Math.sqrt(Math.PI) * Math.pow(Math.cos(angleFromMean), exponent);
    }

    /**
     * Components of a short-crested sea: count frequency bins of equal width over [0.5, 4] omega_p, each with
     * amplitude sqrt(2 S(omega_i) d omega), a frequency jittered inside its bin (no repeat period), a direction drawn
     * from cos^n spreading about meanDirection (rad, 0 = +X, pi/2 = +Z) and a uniform random phase, all from the seed.
     * Rejects seas steeper than the DNV limit S_p = 2 pi Hs / (g Tp^2) <= 1/15 or with 1.86 Hs above 0.78 h.
     */
    public static WaveField synthesize(double significantHeightM, double peakPeriodS, double gamma, double meanDirectionRadians,
                                       double spreadingExponent, double depthM, int count, long seed) {
        Checks.positive(significantHeightM, "Significant height"); Checks.positive(peakPeriodS, "Peak period");
        if (count < 4 || count > 128) { throw new IllegalArgumentException("Use 4..128 components"); }
        if (2 * Math.PI * significantHeightM / (Atmosphere.GRAVITY_MPS2 * peakPeriodS * peakPeriodS) > 1.0 / 15) {
            throw new IllegalArgumentException("Sea too steep: 2 pi Hs / (g Tp^2) must be <= 1/15");
        }
        if (Double.isFinite(depthM) && 1.86 * significantHeightM > LinearWave.MAX_HEIGHT_TO_DEPTH * depthM) {
            throw new IllegalArgumentException("Largest expected wave (1.86 Hs) exceeds the breaking limit 0.78 h");
        }
        Random random = new Random(seed);
        double peak = 2 * Math.PI / peakPeriodS, low = LOW * peak, width = (HIGH - LOW) * peak / count;
        double[] cdf = spreadingCdf(spreadingExponent);
        List<LinearWave> waves = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double omega = low + (i + random.nextDouble()) * width;
            double amplitude = Math.sqrt(2 * jonswap(omega, significantHeightM, peakPeriodS, gamma) * width);
            double direction = meanDirectionRadians + sampleAngle(cdf, random.nextDouble());
            double wavelength = wavelength(omega, depthM);
            // Each component must stay inside linear theory; spectral tails are tiny, so this only clips extremes.
            amplitude = Math.min(amplitude, 0.5 * LinearWave.MAX_STEEPNESS * wavelength);
            if (Double.isFinite(depthM)) { amplitude = Math.min(amplitude, 0.5 * LinearWave.MAX_HEIGHT_TO_DEPTH * depthM); }
            waves.add(new LinearWave(amplitude, wavelength, direction, depthM, 2 * Math.PI * random.nextDouble()));
        }
        return new WaveField(waves);
    }

    /** Wavelength (m) for angular frequency omega in water of depth h (+infinity for deep water), by Newton iteration. */
    public static double wavelength(double omega, double depthM) {
        Checks.positive(omega, "Angular frequency");
        double g = Atmosphere.GRAVITY_MPS2, k = omega * omega / g;
        if (Double.isFinite(depthM)) {
            for (int i = 0; i < 60; i++) {
                double t = Math.tanh(k * depthM), f = g * k * t - omega * omega, df = g * t + g * k * depthM * (1 - t * t);
                double next = k - f / df;
                if (Math.abs(next - k) < 1e-14 * k) { k = next; break; }
                k = next;
            }
        }
        return 2 * Math.PI / k;
    }

    /** Spectral moment m_j = int omega^j S d omega over (0.2, 20) omega_p, effectively the whole spectrum (numerical). */
    public static double moment(int order, double significantHeightM, double peakPeriodS, double gamma) {
        double peak = 2 * Math.PI / peakPeriodS, low = 0.2 * peak, high = 20 * peak, sum = 0;
        int n = 200000;
        for (int i = 0; i < n; i++) {
            double omega = low + (i + 0.5) * (high - low) / n;
            sum += Math.pow(omega, order) * jonswap(omega, significantHeightM, peakPeriodS, gamma);
        }
        return sum * (high - low) / n;
    }

    private static double[] spreadingCdf(double exponent) {
        int n = 512;
        double[] cdf = new double[n + 1];
        for (int i = 1; i <= n; i++) {
            double a = -Math.PI / 2 + Math.PI * (i - 0.5) / n;
            cdf[i] = cdf[i - 1] + spreading(a, exponent) * Math.PI / n;
        }
        for (int i = 1; i <= n; i++) { cdf[i] /= cdf[n]; }
        return cdf;
    }
    private static double sampleAngle(double[] cdf, double u) {
        int n = cdf.length - 1, i = 1;
        while (i < n && cdf[i] < u) { i++; }
        double t = (u - cdf[i - 1]) / Math.max(1e-12, cdf[i] - cdf[i - 1]);
        return -Math.PI / 2 + Math.PI * (i - 1 + t) / n;
    }
    /** Lanczos approximation of ln Gamma(x), x > 0 (g = 7, n = 9; relative error below 1e-13). */
    static double logGamma(double x) {
        double[] c = {0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059,
                12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7};
        if (x < 0.5) { return Math.log(Math.PI / Math.sin(Math.PI * x)) - logGamma(1 - x); }
        x -= 1;
        double a = c[0], t = x + 7.5;
        for (int i = 1; i < 9; i++) { a += c[i] / (x + i); }
        return 0.5 * Math.log(2 * Math.PI) + (x + 0.5) * Math.log(t) - t + Math.log(a);
    }
}

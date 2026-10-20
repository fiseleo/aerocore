package dev.mcmod.aerocore.api.acoustics;

import java.util.ArrayList;
import java.util.List;

/**
 * Looking at sound without hearing it (0.37): a radix-2 FFT, Welch's averaged power spectrum (Hann windows, half
 * overlap), its peaks, 1/3-octave band levels (IEC 61260 nominal centres) and the A-weighting of IEC 61672. For tests
 * and the development exports; nothing here runs in the game's sound path.
 */
public final class Spectrum {
    private Spectrum() {}

    /** In-place complex FFT of length a power of two (forward: e^-i). */
    public static void fft(double[] re, double[] im) {
        int n = re.length;
        if (Integer.bitCount(n) != 1 || im.length != n) { throw new IllegalArgumentException("FFT length must be a power of two"); }
        for (int i = 1, j = 0; i < n; i++) {
            int bit = n >> 1;
            for (; (j & bit) != 0; bit >>= 1) { j ^= bit; }
            j ^= bit;
            if (i < j) {
                double t = re[i]; re[i] = re[j]; re[j] = t;
                t = im[i]; im[i] = im[j]; im[j] = t;
            }
        }
        for (int len = 2; len <= n; len <<= 1) {
            double ang = -2 * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
            for (int i = 0; i < n; i += len) {
                double cr = 1, ci = 0;
                for (int k = 0; k < len / 2; k++) {
                    int a = i + k, b = a + len / 2;
                    double xr = re[b] * cr - im[b] * ci, xi = re[b] * ci + im[b] * cr;
                    re[b] = re[a] - xr; im[b] = im[a] - xi;
                    re[a] += xr; im[a] += xi;
                    double t = cr * wr - ci * wi;
                    ci = cr * wi + ci * wr;
                    cr = t;
                }
            }
        }
    }

    /** A one-sided power spectrum: power per bin (the mean square of the signal is their sum) and the bin width. */
    public record Power(double[] power, double binHz) {
        public double frequency(int bin) { return bin * binHz; }
        public int bin(double hz) { return (int) Math.max(0, Math.min(power.length - 1, Math.round(hz / binHz))); }
        /** Power summed over a band (Hz). */
        public double band(double lowHz, double highHz) {
            double s = 0;
            for (int k = Math.max(0, (int) Math.ceil(lowHz / binHz)); k <= Math.min(power.length - 1, (int) Math.floor(highHz / binHz)); k++) { s += power[k]; }
            return s;
        }
        public double total() { double s = 0; for (double p : power) { s += p; } return s; }
        /** Level of a band relative to full scale (a full-scale sine is -3 dB). */
        public double bandDb(double lowHz, double highHz) { return 10 * Math.log10(Math.max(1e-30, band(lowHz, highHz))); }
    }

    /** Welch's estimate with segments of n samples (a power of two). */
    public static Power welch(float[] x, int rate, int n) {
        if (Integer.bitCount(n) != 1) { throw new IllegalArgumentException("segment must be a power of two"); }
        double[] w = new double[n];
        double wss = 0;
        for (int i = 0; i < n; i++) { w[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / n); wss += w[i] * w[i]; }
        double[] p = new double[n / 2 + 1];
        int segments = 0;
        for (int start = 0; start + n <= x.length; start += n / 2) {
            double[] re = new double[n], im = new double[n];
            for (int i = 0; i < n; i++) { re[i] = x[start + i] * w[i]; }
            fft(re, im);
            for (int k = 0; k <= n / 2; k++) {
                double m = (re[k] * re[k] + im[k] * im[k]) / (wss * n);
                p[k] += (k == 0 || k == n / 2) ? m : 2 * m;
            }
            segments++;
        }
        if (segments == 0) { throw new IllegalArgumentException("fewer samples than one segment"); }
        for (int k = 0; k < p.length; k++) { p[k] /= segments; }
        return new Power(p, rate / (double) n);
    }

    /** A spectral peak: its frequency (Hz, refined by a parabola through the bins around it) and level (dB re full scale). */
    public record Peak(double hz, double db) {}

    /** The strongest local maxima between two frequencies, strongest first, at least minSpacingHz apart. */
    public static List<Peak> peaks(Power s, double lowHz, double highHz, int count, double minSpacingHz) {
        List<Peak> all = new ArrayList<>();
        double[] p = s.power();
        for (int k = Math.max(1, s.bin(lowHz)); k < Math.min(p.length - 1, s.bin(highHz)); k++) {
            if (p[k] > p[k - 1] && p[k] >= p[k + 1] && p[k] > 0) {
                double a = Math.log(Math.max(p[k - 1], 1e-30)), b = Math.log(p[k]), c = Math.log(Math.max(p[k + 1], 1e-30));
                double d = 0.5 * (a - c) / (a - 2 * b + c);
                if (!Double.isFinite(d)) { d = 0; }
                all.add(new Peak((k + d) * s.binHz(), 10 * Math.log10(p[k - 1] + p[k] + p[k + 1])));
            }
        }
        all.sort((x, y) -> Double.compare(y.db(), x.db()));
        List<Peak> out = new ArrayList<>();
        for (Peak q : all) {
            boolean near = false;
            for (Peak o : out) { if (Math.abs(o.hz() - q.hz()) < minSpacingHz) { near = true; break; } }
            if (!near) { out.add(q); }
            if (out.size() == count) { break; }
        }
        return out;
    }

    /** IEC 61260 nominal 1/3-octave centres from 20 Hz to 20 kHz. */
    public static final double[] THIRD_OCTAVES = {20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800,
            1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000, 20000};

    /** Levels (dB re full scale) of the 1/3-octave bands (exact base-two band edges around the nominal centres). */
    public static double[] thirdOctaves(Power s) {
        double[] out = new double[THIRD_OCTAVES.length];
        for (int i = 0; i < out.length; i++) {
            double f = 1000 * Math.pow(2, Math.round(3 * Math.log(THIRD_OCTAVES[i] / 1000) / Math.log(2)) / 3.0);
            out[i] = s.bandDb(f / Math.pow(2, 1 / 6.0), f * Math.pow(2, 1 / 6.0));
        }
        return out;
    }

    /** A-weighting at a frequency, dB (IEC 61672-1; 0 at 1 kHz). */
    public static double aWeightDb(double f) {
        double f2 = f * f;
        double ra = 12194.0 * 12194.0 * f2 * f2 / ((f2 + 20.6 * 20.6) * Math.sqrt((f2 + 107.7 * 107.7) * (f2 + 737.9 * 737.9)) * (f2 + 12194.0 * 12194.0));
        return 20 * Math.log10(Math.max(ra, 1e-30)) + 2.0;
    }

    /** The A-weighted total level (dB re full scale) of a spectrum. */
    public static double aWeightedDb(Power s) {
        double sum = 0;
        for (int k = 1; k < s.power().length; k++) { sum += s.power()[k] * Math.pow(10, aWeightDb(s.frequency(k)) / 10); }
        return 10 * Math.log10(Math.max(sum, 1e-30));
    }
}

package dev.mcmod.aerocore.api.acoustics;

/**
 * Filters for synthesis (0.37), all at {@link Voice#RATE}. Each keeps its own state and can be retuned while it runs
 * (a cutoff following an engine's speed) without clicking: the biquads are transposed direct form II, the state
 * variable filter is the trapezoidal (zero-delay feedback) form of Zavalishin / Simper, which stays stable under fast
 * modulation. Frequencies are clamped inside (1 Hz, 0.45 RATE).
 */
public final class Filters {
    private Filters() {}

    static double clampHz(double f) { return Math.max(1, Math.min(0.45 * Voice.RATE, Double.isFinite(f) ? f : 1)); }

    /** A one-pole low-pass (6 dB per octave): y += a (x - y) with a = 1 - exp(-2 pi f / rate); also its high-pass. */
    public static final class OnePole {
        private double a, y;
        public OnePole(double cutoffHz) { tune(cutoffHz); }
        public void tune(double cutoffHz) { a = 1 - Math.exp(-2 * Math.PI * clampHz(cutoffHz) / Voice.RATE); }
        public double lowpass(double x) { y += a * (x - y); return y; }
        public double highpass(double x) { y += a * (x - y); return x - y; }
        public double value() { return y; }
        public void reset() { y = 0; }
    }

    /** A biquad (RBJ Audio EQ Cookbook coefficients). */
    public static final class Biquad {
        private double b0 = 1, b1, b2, a1, a2, z1, z2;

        public static Biquad lowpass(double f, double q) { Biquad b = new Biquad(); b.tuneLowpass(f, q); return b; }
        public static Biquad highpass(double f, double q) { Biquad b = new Biquad(); b.tuneHighpass(f, q); return b; }
        public static Biquad bandpass(double f, double q) { Biquad b = new Biquad(); b.tuneBandpass(f, q); return b; }
        public static Biquad peaking(double f, double q, double gainDb) { Biquad b = new Biquad(); b.tunePeaking(f, q, gainDb); return b; }

        private void set(double nb0, double nb1, double nb2, double na0, double na1, double na2) {
            b0 = nb0 / na0; b1 = nb1 / na0; b2 = nb2 / na0; a1 = na1 / na0; a2 = na2 / na0;
        }

        public void tuneLowpass(double f, double q) {
            double w = 2 * Math.PI * clampHz(f) / Voice.RATE, c = Math.cos(w), alpha = Math.sin(w) / (2 * Math.max(q, 1e-3));
            set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + alpha, -2 * c, 1 - alpha);
        }

        public void tuneHighpass(double f, double q) {
            double w = 2 * Math.PI * clampHz(f) / Voice.RATE, c = Math.cos(w), alpha = Math.sin(w) / (2 * Math.max(q, 1e-3));
            set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + alpha, -2 * c, 1 - alpha);
        }

        /** Band-pass with 0 dB at the centre. */
        public void tuneBandpass(double f, double q) {
            double w = 2 * Math.PI * clampHz(f) / Voice.RATE, c = Math.cos(w), alpha = Math.sin(w) / (2 * Math.max(q, 1e-3));
            set(alpha, 0, -alpha, 1 + alpha, -2 * c, 1 - alpha);
        }

        public void tunePeaking(double f, double q, double gainDb) {
            double a = Math.pow(10, gainDb / 40), w = 2 * Math.PI * clampHz(f) / Voice.RATE, c = Math.cos(w), alpha = Math.sin(w) / (2 * Math.max(q, 1e-3));
            set(1 + alpha * a, -2 * c, 1 - alpha * a, 1 + alpha / a, -2 * c, 1 - alpha / a);
        }

        public double process(double x) {
            double y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }

        public void reset() { z1 = z2 = 0; }
    }

    /** The trapezoidal state variable filter: low, band and high-pass outputs of one input. */
    public static final class Svf {
        private double g, k, a1, a2, a3, ic1, ic2;
        public double low, band, high;

        public Svf(double f, double q) { tune(f, q); }

        public void tune(double f, double q) {
            g = Math.tan(Math.PI * clampHz(f) / Voice.RATE);
            k = 1 / Math.max(q, 1e-3);
            a1 = 1 / (1 + g * (g + k));
            a2 = g * a1;
            a3 = g * a2;
        }

        public void process(double x) {
            double v3 = x - ic2, v1 = a1 * ic1 + a2 * v3, v2 = ic2 + a2 * ic1 + a3 * v3;
            ic1 = 2 * v1 - ic1;
            ic2 = 2 * v2 - ic2;
            low = v2; band = v1; high = x - k * v1 - v2;
        }

        public void reset() { ic1 = ic2 = low = band = high = 0; }
    }
}

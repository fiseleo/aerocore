package dev.mcmod.aerocore.api.acoustics;

/**
 * Seeded noise for synthesis (0.37): the same seed gives the same samples (a test, or two players hearing one
 * vehicle, gets the same sound). White is xorshift64* scaled to [-1, 1); pink falls 3 dB an octave (Paul Kellet's
 * refined filter of white noise, within 0.05 dB from 9 Hz to the Nyquist frequency at 44.1 kHz); brown falls 6 dB an
 * octave (a leaky integrator). Pink and brown are scaled to about the RMS of white (0.58).
 */
public final class Noise {
    private long state;
    private double b0, b1, b2, b3, b4, b5, b6, brown;

    public Noise(long seed) {
        long s = seed * 0x9E3779B97F4A7C15L + 0x632BE59BD9B4E019L;
        state = s == 0 ? 0x2545F4914F6CDD1DL : s;
        for (int i = 0; i < 4; i++) { white(); }
    }

    /** Uniform in [-1, 1). */
    public double white() {
        long x = state;
        x ^= x >>> 12; x ^= x << 25; x ^= x >>> 27;
        state = x;
        long r = x * 0x2545F4914F6CDD1DL;
        return ((r >>> 11) * 0x1.0p-53) * 2 - 1;
    }

    /** Uniform in [0, 1). */
    public double uniform() { return (white() + 1) * 0.5; }

    /** True with probability p. */
    public boolean chance(double p) { return uniform() < p; }

    /** About normal with zero mean and unit variance (sum of four uniforms, scaled). */
    public double gauss() { return (white() + white() + white() + white()) * 0.8660254037844386; }

    /** Pink noise (-3 dB per octave). */
    public double pink() {
        double w = white();
        b0 = 0.99886 * b0 + w * 0.0555179;
        b1 = 0.99332 * b1 + w * 0.0750759;
        b2 = 0.96900 * b2 + w * 0.1538520;
        b3 = 0.86650 * b3 + w * 0.3104856;
        b4 = 0.55000 * b4 + w * 0.5329522;
        b5 = -0.7616 * b5 - w * 0.0168980;
        double p = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362;
        b6 = w * 0.115926;
        return p * 0.333;
    }

    /** Brown noise (-6 dB per octave above about 35 Hz). */
    public double brown() {
        brown = 0.995 * brown + white() * 0.1;
        return brown;
    }
}

package dev.mcmod.aerocore.api.acoustics;

/**
 * A phase accumulator with band-limited waveforms (0.37): sine, and saw and pulse with PolyBLEP corrections at their
 * steps (Valimaki and Huovilainen 2007), so a sweep to high frequencies does not fold back as audible aliasing.
 * The frequency may change every sample.
 */
public final class Oscillator {
    private double phase;

    public Oscillator() {}
    public Oscillator(double startPhase) { phase = startPhase - Math.floor(startPhase); }

    /** The phase in [0, 1). */
    public double phase() { return phase; }

    /** Advances by one sample at hz and returns the phase before the step. */
    public double advance(double hz) {
        double p = phase;
        phase += Math.max(0, hz) / Voice.RATE;
        if (phase >= 1) { phase -= Math.floor(phase); }
        return p;
    }

    public double sine(double hz) { return Math.sin(2 * Math.PI * advance(hz)); }

    /** A falling-edge-free saw (-1 to 1) with PolyBLEP. */
    public double saw(double hz) {
        double dt = Math.min(0.5, Math.max(0, hz) / Voice.RATE), p = advance(hz);
        return 2 * p - 1 - blep(p, dt);
    }

    /** A pulse of duty (0-1) between -1 and 1, PolyBLEP at both edges. */
    public double pulse(double hz, double duty) {
        double dt = Math.min(0.5, Math.max(0, hz) / Voice.RATE), p = advance(hz), d = Math.max(0.01, Math.min(0.99, duty));
        double v = p < d ? 1 : -1;
        v += blep(p, dt);
        double q = p - d;
        if (q < 0) { q += 1; }
        v -= blep(q, dt);
        return v;
    }

    static double blep(double t, double dt) {
        if (dt <= 0) { return 0; }
        if (t < dt) { double x = t / dt; return x + x - x * x - 1; }
        if (t > 1 - dt) { double x = (t - 1) / dt; return x * x + x + x + 1; }
        return 0;
    }
}

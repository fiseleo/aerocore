package dev.mcmod.aerocore.api.acoustics;

/**
 * Modal synthesis (0.37): a struck object rings as a sum of damped sinusoids, each mode a two-pole resonator
 * y[n] = 2 r cos(w) y[n-1] - r^2 y[n-2] + g x[n] (Adrien 1991; van den Doel and Pai 1998 for contact sounds). The modes'
 * frequencies, their 60 dB decay times and gains come from the struck object (ImpactNoise works them out from size
 * and material); {@link #strike} excites all of them at once.
 */
public final class Modal {
    private final double[] c1, c2, gain, y1, y2;
    private final double[] decayS;

    /** Modes by frequency (Hz), 60 dB decay time (s) and relative gain. */
    public Modal(double[] frequencyHz, double[] decayS, double[] gain) {
        int n = frequencyHz.length;
        if (decayS.length != n || gain.length != n) { throw new IllegalArgumentException("one decay and gain per mode"); }
        this.c1 = new double[n]; this.c2 = new double[n]; this.gain = new double[n];
        this.y1 = new double[n]; this.y2 = new double[n];
        this.decayS = decayS.clone();
        for (int i = 0; i < n; i++) {
            double f = Filters.clampHz(frequencyHz[i]), w = 2 * Math.PI * f / Voice.RATE;
            double r = Math.exp(-6.907755 / (Math.max(1e-3, decayS[i]) * Voice.RATE));
            c1[i] = 2 * r * Math.cos(w);
            c2[i] = -r * r;
            // normalised so a unit strike gives a mode of peak about gain (the resonator's impulse response peaks at 1/sin w)
            this.gain[i] = gain[i] * Math.sin(w);
        }
    }

    public int modes() { return c1.length; }

    /** Adds a strike of an amplitude (the modes' peaks scale with it). */
    public void strike(double amplitude) {
        for (int i = 0; i < c1.length; i++) { y1[i] += amplitude * gain[i]; }
    }

    /** Adds an excitation sample (a contact force through time: scraping, rolling). */
    public double next(double excitation) {
        double sum = 0;
        for (int i = 0; i < c1.length; i++) {
            double y = c1[i] * y1[i] + c2[i] * y2[i] + gain[i] * excitation;
            y2[i] = y1[i];
            y1[i] = y;
            sum += y;
        }
        return sum;
    }

    /** Whether any mode still rings above about a millionth. */
    public boolean ringing() {
        for (int i = 0; i < c1.length; i++) { if (Math.abs(y1[i]) + Math.abs(y2[i]) > 1e-6) { return true; } }
        return false;
    }

    /** The longest 60 dB decay of the modes, s. */
    public double longestDecayS() {
        double m = 0;
        for (double d : decayS) { m = Math.max(m, d); }
        return m;
    }
}

package dev.mcmod.aerocore.api.acoustics;

/** Small helpers for synthesis (0.37): decibels, smoothing coefficients, soft clipping. */
public final class Dsp {
    private Dsp() {}

    /** Reference sound pressure, Pa (0 dB SPL). */
    public static final double P_REF = 20e-6;
    /** Reference sound power, W (0 dB sound power level). */
    public static final double W_REF = 1e-12;

    public static double gainToDb(double gain) { return 20 * Math.log10(Math.max(1e-12, Math.abs(gain))); }
    public static double dbToGain(double db) { return Math.pow(10, db / 20); }
    /** Sound pressure level of an RMS pressure (Pa). */
    public static double splDb(double rmsPa) { return 20 * Math.log10(Math.max(1e-12, rmsPa) / P_REF); }
    public static double pressurePa(double splDb) { return P_REF * Math.pow(10, splDb / 20); }
    /** Sound power level of an acoustic power (W). */
    public static double powerLevelDb(double watts) { return 10 * Math.log10(Math.max(1e-30, watts) / W_REF); }
    public static double powerW(double levelDb) { return W_REF * Math.pow(10, levelDb / 10); }
    /** Sums levels in dB as powers. */
    public static double sumDb(double... levels) {
        double s = 0;
        for (double l : levels) { s += Math.pow(10, l / 10); }
        return 10 * Math.log10(Math.max(1e-30, s));
    }

    /** The per-sample coefficient of a one-pole smoother reaching 63 % in seconds. */
    public static double smoothing(double seconds) { return seconds <= 0 ? 1 : 1 - Math.exp(-1 / (seconds * Voice.RATE)); }

    /** A soft clip that is about linear below 0.5 and never passes 1. */
    public static double softClip(double x) {
        double a = Math.abs(x);
        if (a < 0.5) { return x; }
        double y = 0.5 + 0.5 * Math.tanh((a - 0.5) * 2);
        return Math.copySign(y, x);
    }

    /** RMS of samples. */
    public static double rms(float[] samples, int from, int to) {
        double s = 0;
        int n = Math.max(1, to - from);
        for (int i = from; i < to; i++) { s += samples[i] * (double) samples[i]; }
        return Math.sqrt(s / n);
    }
}

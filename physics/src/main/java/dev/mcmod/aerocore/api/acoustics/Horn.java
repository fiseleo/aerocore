package dev.mcmod.aerocore.api.acoustics;

/**
 * Horns and whistles (0.37):
 * <ul>
 * <li>An electric horn: a steel diaphragm driven by an interrupter at its resonance, a buzz rich in harmonics through a
 *     short trumpet; cars carry a pair a major third apart (about 420 and 510 Hz). Level: 105-118 dB(A) at 2 m
 *     (UN ECE R28 asks 93-112 dB(A) at 7 m for a car), here a sound power of 125 dB.</li>
 * <li>An air horn (trucks, trains): a diaphragm valve in a long flared trumpet, a brassy tone with strong odd and even
 *     harmonics, lower (about 280 Hz for a single trumpet, chords on several). Sound power about 130 dB.</li>
 * <li>A ship's whistle: COLREGs Annex III, the fundamental by the vessel's length: 70-200 Hz at 200 m and longer,
 *     130-350 Hz from 75 to 200 m, 250-700 Hz below 75 m (here the middle of each range, falling with length), heard
 *     0.5-2 nautical miles (a sound pressure of 120-143 dB at 1 m).</li>
 * </ul>
 */
public final class Horn implements Voice {
    public enum Kind { ELECTRIC, AIR, WHISTLE }

    public record Spec(Kind kind, double[] fundamentalsHz, long seed) {}

    private final Spec spec;
    private final Oscillator[] tones;
    private final Noise noise;
    private final Filters.Biquad trumpet;
    private volatile boolean sounding;
    private double env;

    public Horn(Spec spec) {
        this.spec = spec;
        this.tones = new Oscillator[spec.fundamentalsHz().length];
        for (int i = 0; i < tones.length; i++) { tones[i] = new Oscillator(0.13 * i); }
        this.noise = new Noise(spec.seed());
        double formant = switch (spec.kind()) { case ELECTRIC -> 2800; case AIR -> 1400; case WHISTLE -> 900; };
        this.trumpet = Filters.Biquad.peaking(formant, 1.2, 8);
    }

    /** A car's pair a major third apart (electric). */
    public static Spec car(long seed) { return new Spec(Kind.ELECTRIC, new double[]{420, 510}, seed); }
    /** A truck's air horn chord (three trumpets). */
    public static Spec truck(long seed) { return new Spec(Kind.AIR, new double[]{280, 350, 420}, seed); }

    /** A ship's whistle for a vessel's length, COLREGs Annex III. */
    public static Spec ship(double lengthM, long seed) {
        double f;
        if (lengthM >= 200) { f = 130; }
        else if (lengthM >= 75) { f = 240 - 0.55 * (lengthM - 75); }
        else { f = 600 - 4.5 * Math.max(0, lengthM - 10); }
        return new Spec(Kind.WHISTLE, new double[]{Math.max(70, Math.min(700, f))}, seed);
    }

    public static double soundPowerDb(Spec spec, boolean sounding) {
        if (!sounding) { return -100; }
        return switch (spec.kind()) { case ELECTRIC -> 125; case AIR -> 130; case WHISTLE -> 140; };
    }

    public void sound(boolean on) { sounding = on; }

    @Override public void render(float[] out, int offset, int n) {
        boolean on = sounding;
        double attack = spec.kind() == Kind.ELECTRIC ? Dsp.smoothing(0.008) : Dsp.smoothing(0.06);
        double release = spec.kind() == Kind.WHISTLE ? Dsp.smoothing(0.25) : Dsp.smoothing(0.04);
        for (int i = 0; i < n; i++) {
            env += ((on ? 1 : 0) - env) * (on ? attack : release);
            if (env < 1e-5 && !on) { out[offset + i] = 0; continue; }
            double s = 0;
            for (int k = 0; k < tones.length; k++) {
                double f = spec.fundamentalsHz()[k] * (1 + 0.002 * noise.white());
                s += switch (spec.kind()) {
                    case ELECTRIC -> tones[k].pulse(f, 0.3);
                    case AIR -> 0.7 * tones[k].saw(f) + 0.3 * Math.sin(2 * Math.PI * tones[k].phase() * 2);
                    case WHISTLE -> Math.sin(2 * Math.PI * tones[k].advance(f)) + 0.15 * Math.sin(4 * Math.PI * tones[k].phase());
                };
            }
            s = trumpet.process(s / tones.length) + (spec.kind() == Kind.WHISTLE ? 0.03 * noise.white() : 0);
            out[offset + i] = (float) Dsp.softClip(0.35 * env * s);
        }
    }
}

package dev.mcmod.aerocore.api.acoustics;

/**
 * Air rushing past a body (0.37): turbulent boundary layers and separated edges radiate as dipoles, so their sound
 * power grows as U^6 (Curle 1955); the spectrum is broad, peaking at a Strouhal frequency 0.2 U / d of the edges'
 * scale d (mirrors, pillars, struts: a few centimetres), and swells and fades with the gusts. An opening (a window,
 * an open cockpit, a door ajar) adds its cavity's buffeting, a low rumble near the Helmholtz frequency.
 * Level: 85 dB at 30 m/s for a 2.2 m^2 frontal area, 60 log10(U / 30) + 10 log10(A / 2.2); the same law serves the
 * listener's own ears in the wind (an open seat, flying), with A that of a head.
 */
public final class FlowNoise implements Voice {
    public record Spec(double frontalAreaM2, double edgeScaleM, long seed) {}

    /** The airspeed (m/s), how open the body is to the stream (0-1) and the gustiness (0-1). */
    public record State(double airspeedMps, double opening, double gust) {
        public static final State CALM = new State(0, 0, 0);
    }

    private final Spec spec;
    private volatile State state = State.CALM;
    private final Noise noise;
    private final Filters.Svf band = new Filters.Svf(300, 0.5), buffet = new Filters.Svf(18, 2);
    private double speed, swell, retune;

    public FlowNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public void set(State s) { if (s != null) { state = s; } }

    public static double soundPowerDb(Spec spec, State st) {
        double u = Math.abs(st.airspeedMps());
        if (u < 1) { return -100; }
        return 85 + 60 * Math.log10(u / 30) + 10 * Math.log10(Math.max(1e-3, spec.frontalAreaM2()) / 2.2) + 6 * Math.clamp(st.opening(), 0, 1);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.1);
        for (int i = 0; i < n; i++) {
            speed += (Math.abs(st.airspeedMps()) - speed) * follow;
            if (--retune <= 0) {
                retune = 64;
                double peak = 0.2 * Math.max(speed, 1) / Math.max(0.005, spec.edgeScaleM());
                band.tune(Math.min(peak, 6000), 0.45);
                buffet.tune(Math.max(8, 12 + speed * 0.4), 2.5);
            }
            swell += (noise.white() * 0.5 - swell) * 0.00004 * (1 + 4 * st.gust());
            double level = Math.min(1, speed / 5) * (1 + (0.3 + 0.5 * st.gust()) * swell);
            band.process(noise.pink());
            buffet.process(noise.white());
            double sample = level * (band.band * 1.1 + 0.4 * st.opening() * buffet.band * 6);
            out[offset + i] = (float) Dsp.softClip(sample);
        }
    }
}

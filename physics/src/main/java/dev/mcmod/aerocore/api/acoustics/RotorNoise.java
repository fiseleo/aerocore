package dev.mcmod.aerocore.api.acoustics;

/**
 * Propellers and rotors (0.37). Each blade's loading and thickness, passing a point on the disc's rim, sound at the
 * blade passing frequency B n and its harmonics (Gutin 1936): the tones grow steeply with the tip's Mach number (here
 * their sound power rises as the tip Mach to the sixth power over a level set by the shaft power,
 * Lw = 10 log10(P_kW) + 92 + 60 log10(M_tip / 0.6), which gives a light aircraft's 2-blade propeller near 115 dB at
 * full power, a helicopter's main rotor 120-125 dB), plus the broadband swish of turbulent flow over the blades. A
 * helicopter rotor descending into its own wake slaps: blade-vortex interaction, sharp impulses at the blade passing
 * rate, here with the descent's share. A marine propeller is a dull thrum at its blade rate and, once its tips
 * cavitate, a crackling hiss.
 */
public final class RotorNoise implements Voice {
    public enum Kind { PROPELLER, ROTOR, MARINE }

    public record Spec(Kind kind, int blades, double diameterM, long seed) {}

    /** Spinning at rev/s with shaft power (W) into air or water; slap (0-1) the share of blade-vortex interaction, cavitation (0-1). */
    public record State(double revPerS, double powerW, double slap, double cavitation) {
        public static final State STILL = new State(0, 0, 0, 0);
    }

    private final Spec spec;
    private volatile State state = State.STILL;
    private final Noise noise;
    private final Filters.Svf swish = new Filters.Svf(800, 0.6);
    private final Filters.Biquad body = Filters.Biquad.lowpass(1200, 0.7), cavitate = Filters.Biquad.highpass(2500, 0.7);
    private double rev, phase, slapEnv, retune;

    public RotorNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public Spec spec() { return spec; }
    public void set(State s) { if (s != null) { state = s; } }

    public static double tipMach(Spec s, double revPerS) { return Math.PI * s.diameterM() * Math.abs(revPerS) / 343; }

    public static double soundPowerDb(Spec s, State st) {
        if (Math.abs(st.revPerS()) < 0.2) { return -100; }
        if (s.kind() == Kind.MARINE) {
            return 120 + 10 * Math.log10(Math.max(1e-3, st.powerW() / 1e5)) + 20 * Math.max(0, st.cavitation());
        }
        double kw = Math.max(0.05, st.powerW() / 1000 + 0.02 * s.diameterM() * Math.abs(st.revPerS()));
        return 10 * Math.log10(kw) + 92 + 60 * Math.log10(Math.max(0.05, tipMach(s, st.revPerS())) / 0.6) + 6 * Math.clamp(st.slap(), 0, 1);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.1);
        int blades = Math.max(1, spec.blades());
        for (int i = 0; i < n; i++) {
            rev += (Math.abs(st.revPerS()) - rev) * follow;
            double bpf = blades * rev;
            double before = phase;
            phase += bpf / RATE;
            phase -= Math.floor(phase);
            double mach = tipMach(spec, rev);
            double s;
            if (spec.kind() == Kind.MARINE) {
                s = body.process(Math.sin(2 * Math.PI * phase) * 0.6 + noise.brown() * 0.8);
                s += cavitate.process(noise.white()) * Math.clamp(st.cavitation(), 0, 1) * 0.8;
            } else {
                // a loading pulse per blade passing: sharper (richer in harmonics) at higher tip Mach
                double sharp = 2 + 10 * Math.min(1, mach);
                double pulse = Math.exp(-sharp * (1 - Math.cos(2 * Math.PI * phase)));
                if (--retune <= 0) { retune = 64; swish.tune(Math.min(6000, 200 + 3000 * mach), 0.5); }
                swish.process(noise.white());
                s = 1.2 * (pulse - 0.15) + 0.4 * swish.band * (0.5 + 0.5 * pulse);
                if (phase < before && st.slap() > 0) { slapEnv = Math.clamp(st.slap(), 0, 1); }
                slapEnv *= 0.993;
                s += slapEnv * noise.white() * 1.5;
            }
            out[offset + i] = (float) Dsp.softClip(0.5 * s * Math.min(1, rev / 2));
        }
    }
}

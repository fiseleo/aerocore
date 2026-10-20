package dev.mcmod.aerocore.api.acoustics;

/**
 * Electric machines (0.37): magnetic forces pull the stator at twice the electrical frequency (f_e = poles/2 x rev/s),
 * the rotor's slots add tones at slots x rev/s, a cooling fan hisses with speed; a drive's inverter switching at its
 * carrier (a few kHz) sings with sidebands at the carrier plus and minus twice f_e, and older traction drives step the
 * carrier with speed (the "singing" scale of trains). A machine on the mains (a generator, a transformer) hums at
 * twice the grid frequency and its harmonics. Small motors (wipers, seat and window motors, a landing gear's pump)
 * are the same with a gearbox whine at the worm's tooth frequency.
 * Level: 10 log10(P_kW) + 72 dB at full load (IEC 60034-9 limits for 1-100 kW machines lie 70-95 dB(A)), less at part load.
 */
public final class MotorNoise implements Voice {
    /** A machine: poles, rotor slots, rated power (W), its drive's carrier (Hz, 0: mains or DC), and a gear's teeth (0 none). */
    public record Spec(int poles, int slots, double ratedW, double carrierHz, int gearTeeth, long seed) {
        public static Spec traction(double ratedW, long seed) { return new Spec(4, 36, ratedW, 4000, 0, seed); }
        public static Spec small(double ratedW, long seed) { return new Spec(2, 12, ratedW, 0, 30, seed); }
    }

    /** Running at a speed (rev/s) and load (0-1); grid locks its electrical frequency to the mains (Hz, 0 when not). */
    public record State(double revPerS, double load, double gridHz) {
        public static final State STILL = new State(0, 0, 0);
    }

    private final Spec spec;
    private volatile State state = State.STILL;
    private final Oscillator magnetic = new Oscillator(), slot = new Oscillator(0.3), carrier = new Oscillator(0.6), gear = new Oscillator(0.1);
    private final Noise noise;
    private final Filters.Biquad fan = Filters.Biquad.bandpass(1500, 0.7);
    private double rev, load;

    public MotorNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public Spec spec() { return spec; }
    public void set(State s) { if (s != null) { state = s; } }

    public static double soundPowerDb(Spec spec, State st) {
        if (st.revPerS() < 0.5 && st.gridHz() <= 0) { return -100; }
        double kw = Math.max(1e-3, spec.ratedW() / 1000);
        return 72 + 10 * Math.log10(kw) - 6 * (1 - Math.clamp(st.load(), 0, 1)) + 10 * Math.log10(Math.max(0.1, Math.min(1.5, st.revPerS() / 25 + 0.1)));
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.05);
        for (int i = 0; i < n; i++) {
            rev += (st.revPerS() - rev) * follow;
            load += (Math.clamp(st.load(), 0, 1) - load) * follow;
            double fe = st.gridHz() > 0 ? st.gridHz() : spec.poles() / 2.0 * rev;
            double s = 0.4 * (0.3 + 0.7 * load) * magnetic.sine(2 * fe) + 0.15 * slot.sine(spec.slots() * rev);
            if (spec.carrierHz() > 0 && rev > 0.2) {
                double c = spec.carrierHz() * (rev < 15 ? 0.5 + 0.5 * Math.floor(rev / 3) / 5 : 1);
                s += 0.12 * (0.4 + 0.6 * load) * carrier.sine(c + 2 * fe);
            }
            if (spec.gearTeeth() > 0) { s += 0.25 * gear.saw(spec.gearTeeth() * rev); }
            s += fan.process(noise.white()) * 0.3 * Math.min(1, rev / 40);
            out[offset + i] = (float) Dsp.softClip(0.6 * s * Math.min(1, (rev + (st.gridHz() > 0 ? 10 : 0)) / 2));
        }
    }
}

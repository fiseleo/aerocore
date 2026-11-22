package dev.mcmod.aerocore.api.acoustics;

/**
 * A crawler's track, one side of it (0.43):
 * <ul>
 * <li>Links: the track is a chain of shoes of pitch p, so as the belt runs at v every place along it is struck at the
 *     link passing frequency f = v / p: the sprocket's teeth engaging a link, the idler where the links drop onto the
 *     ground, each road wheel rolling over a joint. The sprocket's and the idler's are steel on steel, a clank ringing
 *     the links and the sprocket (here modes near 0.6, 1.3, 2.4 and 3.9 kHz), the road wheels' a duller knock through
 *     the hull; the polygon the links make round the sprocket (chordal action) pulls the belt in pulses at f. Each
 *     impact comes a little early or late (worn pins, up to 3 % of a link's period, from the seed), so the clank's
 *     high harmonics spread while the low ones stay lines. Tracked vehicles' spectra are lines at f and its harmonics
 *     over everything else: an M113 at 10 mph on its 6 inch shoes at 30, 60, 90 and 120 Hz, the track laying frequency
 *     17.6 x mph / shoe inches (US Army measurements, DTIC ADA458620).</li>
 * <li>Rubber band tracks have no metal joints to clank: the lugs' soft thump and the wheels' rumble.</li>
 * <li>Squeal: slipping sideways (skid steering, a pivot turn) the shoes scrape the ground and the road wheels grind on
 *     the guide horns, a screech around 2 kHz with a wandering tone.</li>
 * </ul>
 * Level ({@link #soundPowerDb}): each impact's energy grows as v^2 (the links' change of speed at the sprocket is in
 * proportion to v, their mass to p) and v / p of them come a second, so the power goes as v^3 with the pitch cancelling:
 * 30 log10 v. 112 dB for a steel track at 10 m/s under 60 kN (a 12 t carrier's two tracks 115 dB, about 84 dB at 15 m:
 * the order of tracked vehicle pass-bys), heavier 5 log10 of the load (a rule), a rubber track 10 dB below (a rule;
 * band track trials on the M113 found it markedly quieter, DTIC ADA458620). The voice's output stays near -12 dBFS RMS
 * while it runs; its balance between rolling and squeal follows their sound powers.
 */
public final class TrackNoise implements Voice {
    /** A track: its shoes' pitch (m), whether they are a rubber band (no clank), its road wheels, and the seed. */
    public record Spec(double pitchM, boolean rubber, int roadWheels, long seed) {
        /** A steel track of a pitch (an excavator's 0.17-0.22 m, a carrier's 0.15 m) on five road wheels. */
        public static Spec steel(double pitchM, long seed) { return new Spec(pitchM, false, 5, seed); }
        /** A rubber track of a lug pitch (a compact excavator's 0.07-0.1 m) on five road wheels. */
        public static Spec rubber(double pitchM, long seed) { return new Spec(pitchM, true, 5, seed); }
    }

    /** How it runs: the belt's speed past the hull (m/s), the shoes' sideways slip on the ground (m/s) and the load on the track (N). */
    public record State(double beltSpeedMps, double slipMps, double loadN) {
        /** Standing. */
        public static final State STILL = new State(0, 0, 0);
    }

    /** The reference point of the level: 112 dB at 10 m/s under 60 kN on steel. */
    private static final double REF_DB = 112, REF_MPS = 10, REF_N = 6e4, RUBBER_DB = -10;
    /** The squeal: the friction coefficient of the scrape and the share of its power radiated (rules). */
    private static final double SCRAPE_FRICTION = 0.5, SCRAPE_EFFICIENCY = 3e-6;
    /** Each impact comes up to this share of a link's period late (from the seed). */
    private static final double JITTER = 0.03;
    /** Limits that keep wild inputs finite: speed and slip (m/s), load (N), link frequency (Hz). */
    private static final double MOST_MPS = 40, MOST_N = 1e8, MOST_HZ = 2000;

    private final Spec spec;
    private final double pitch;
    private final boolean rubber;
    private volatile State state = State.STILL;
    private final Noise noise;
    private final double[] offsets, strengths;
    private final int[] countdown;
    private final Modal clank, thud;
    private final Filters.Svf rumble = new Filters.Svf(220, 0.6), screech = new Filters.Svf(2200, 1.5);
    private final Oscillator squealTone = new Oscillator(0.3);
    private double phase, speed, rollMix, squealMix, wobble;

    /** A track's voice; where along a link each road wheel strikes and the modes' spread come from the spec's seed. */
    public TrackNoise(Spec spec) {
        this.spec = spec;
        this.pitch = Double.isFinite(spec.pitchM()) && spec.pitchM() > 0 ? Math.max(0.02, spec.pitchM()) : 0;
        this.rubber = spec.rubber();
        this.noise = new Noise(spec.seed());
        int wheels = Math.clamp(spec.roadWheels(), 0, 12);
        offsets = new double[2 + wheels];
        strengths = new double[2 + wheels];
        countdown = new int[2 + wheels];
        offsets[0] = 0;
        strengths[0] = 1;
        offsets[1] = 0.2 + 0.6 * noise.uniform();
        strengths[1] = 0.6;
        for (int i = 2; i < offsets.length; i++) { offsets[i] = noise.uniform(); strengths[i] = 0.3; }
        double[] cf = {620, 1340, 2350, 3900};
        for (int i = 0; i < cf.length; i++) { cf[i] *= 1 + 0.03 * noise.white(); }
        clank = new Modal(cf, new double[]{0.07, 0.05, 0.035, 0.025}, new double[]{1, 0.7, 0.5, 0.35});
        thud = rubber
                ? new Modal(new double[]{70 * (1 + 0.03 * noise.white()), 140, 260}, new double[]{0.06, 0.04, 0.03}, new double[]{1, 0.4, 0.15})
                : new Modal(new double[]{95 * (1 + 0.03 * noise.white()), 170, 320}, new double[]{0.12, 0.08, 0.05}, new double[]{1, 0.6, 0.3});
    }

    /** The track this voice is. */
    public Spec spec() { return spec; }

    /** Sets how it runs (from any thread; the voice reads the latest at its next block). */
    public void set(State s) { if (s != null) { state = s; } }

    private static double finite(double x) { return Double.isFinite(x) ? x : 0; }

    /** The link passing frequency f = v / p (Hz) of a pitch (m) at a belt speed (m/s); 0 without a pitch. */
    public static double linkHz(double pitchM, double beltSpeedMps) {
        return Double.isFinite(pitchM) && pitchM > 0 ? Math.min(MOST_MPS, Math.abs(finite(beltSpeedMps))) / Math.max(0.02, pitchM) : 0;
    }

    /** How much the track squeals (0-1): sideways slip past 0.15 m/s, fully at 1.15 m/s. */
    public static double squeal(State st) {
        return Math.clamp((Math.min(MOST_MPS, Math.abs(finite(st.slipMps()))) - 0.15) / 1.0, 0, 1);
    }

    private static double loadN(State st) { return Math.clamp(Math.abs(finite(st.loadN())), 1000, MOST_N); }

    private static double rollingW(Spec spec, State st) {
        double v = Math.min(MOST_MPS, Math.abs(finite(st.beltSpeedMps())));
        if (v < 0.05) { return 0; }
        return Dsp.powerW(REF_DB + 30 * Math.log10(v / REF_MPS) + 5 * Math.log10(loadN(st) / REF_N) + (spec.rubber() ? RUBBER_DB : 0));
    }

    private static double squealW(Spec spec, State st) {
        double sq = squeal(st);
        if (sq <= 0) { return 0; }
        double slip = Math.min(MOST_MPS, Math.abs(finite(st.slipMps())));
        return sq * SCRAPE_EFFICIENCY * SCRAPE_FRICTION * loadN(st) * slip * (spec.rubber() ? 0.25 : 1);
    }

    /**
     * The track's sound power level (dB): rolling 112 + 30 log10(v / 10 m/s) + 5 log10(N / 60 kN), 10 dB less on
     * rubber, and the squeal 10 log10(s x 3e-6 x 0.5 N v_slip / 1 pW) (s the squeal 0-1: 3e-6 of the scraping friction's
     * power, a rule; a quarter of it on rubber), summed as powers; -100 standing. A 60 kN track pivoting at 1 m/s of slip
     * squeals at about 109 dB, louder than it rolls at 3 m/s (96 dB), as skid-steered crawlers do.
     */
    public static double soundPowerDb(Spec spec, State st) {
        double w = rollingW(spec, st) + squealW(spec, st);
        return w > 0 ? Dsp.powerLevelDb(w) : -100;
    }

    private void strike(int source, double amp) {
        boolean end = source < 2;   // the sprocket or the idler
        if (rubber) {
            thud.strike((end ? 0.6 : 0.45) * amp);
        } else if (end) {
            clank.strike(0.5 * amp);
            thud.strike(0.18 * amp);
        } else {
            thud.strike(0.35 * amp);
            clank.strike(0.1 * amp);
        }
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.03), slow = Dsp.smoothing(0.08);
        double speedTarget = Math.min(MOST_MPS, Math.abs(finite(st.beltSpeedMps())));
        double rw = rollingW(spec, st), sw = squealW(spec, st), total = rw + sw;
        double rollTarget = total > 0 ? Math.sqrt(rw / total) : 0, squealTarget = total > 0 ? Math.sqrt(sw / total) : 0;
        double squealHz = (rubber ? 1300 : 1700) + 700 * Math.min(1, Math.abs(finite(st.slipMps())) / 2);
        for (int i = 0; i < n; i++) {
            speed += (speedTarget - speed) * follow;
            rollMix += (rollTarget - rollMix) * slow;
            squealMix += (squealTarget - squealMix) * slow;
            // the links passing: each place along a link is struck once a period, a little late by the seed
            double f = pitch > 0 ? Math.min(MOST_HZ, speed / pitch) : 0;
            double next = phase + f / RATE;
            if (f > 0) {
                for (int k = 0; k < offsets.length; k++) {
                    double o = offsets[k];
                    if ((phase < o && o <= next) || o + 1 <= next) {
                        countdown[k] = 1 + (int) Math.min(RATE, noise.uniform() * JITTER * RATE / f);
                    }
                }
            }
            phase = next >= 1 ? next - 1 : next;
            double scale = rollMix / Math.sqrt(1 + f / 20);
            for (int k = 0; k < countdown.length; k++) {
                if (countdown[k] > 0 && --countdown[k] == 0) { strike(k, scale * strengths[k] * (0.85 + 0.3 * noise.uniform())); }
            }
            double s = clank.next(0) + thud.next(0);
            // the chordal action's pull at f and the wheels' rumble
            double moving = Math.min(1, speed / 0.3), th = 2 * Math.PI * phase;
            s += (rubber ? 0.03 : 0.05) * rollMix * moving * (Math.sin(th) + 0.4 * Math.sin(2 * th));
            rumble.process(noise.pink());
            s += (rubber ? 0.12 : 0.06) * rollMix * moving * rumble.band;
            // the squeal: a scraping screech and a wandering tone
            if (squealMix > 1e-6) {
                screech.process(noise.white());
                wobble += noise.white() * 0.02 - wobble * 0.0005;
                double q = 2 * Math.PI * squealTone.advance(squealHz * (1 + 0.05 * Math.sin(wobble)));
                double tone = Math.sin(q) + 0.5 * Math.sin(2 * q) + 0.25 * Math.sin(3 * q);
                s += squealMix * (0.5 * screech.band + 0.12 * tone * (0.8 + 0.2 * noise.white()));
            }
            out[offset + i] = (float) Dsp.softClip(s);
        }
    }
}

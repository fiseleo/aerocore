package dev.mcmod.aerocore.api.acoustics;

/**
 * A rolling tyre's sound (0.37):
 * <ul>
 * <li>Tread impacts: blocks meet the road at f = v / pitch. Real tyres vary the pitch (here three lengths, 0.88, 1 and
 *     1.12 times, in a seeded random order: Varterasian 1969's pitch sequencing) so the tone spreads into a hum; the
 *     impacts ring the belt and the air pumped out of the grooves (a band near 1 kHz, Sandberg and Ejsmont, Tyre/Road
 *     Noise Reference Book, ch. 6).</li>
 * <li>The road's texture: a low roar, louder on coarse surfaces (gravel crunches as a stream of grains, sand and snow
 *     hush, ice is nearly silent), a hiss of water on a wet road.</li>
 * <li>Slip: near the peak of grip the tread sticks and slips, a squeal with harmonics around 0.7-1.3 kHz (rising with
 *     the slip speed and the load); a locked wheel drags (a broadband scrub); on loose ground slip only digs louder.</li>
 * </ul>
 * Level ({@link #soundPowerDb}): one tyre's 94 dB at 80 km/h (four of them give the 70-72 dB(A) of a car's pass-by at
 * 7.5 m that tyre noise dominates at that speed), 33 log10(v / 22.2 m/s) (the CPX slope of 30-40), plus the tread's
 * and surface's offsets and a squeal of up to 104 dB. The output stays near -12 dBFS RMS while rolling.
 */
public final class TyreNoise implements Voice {
    /** Tread kinds by their block pitch (m) and level offset (dB). */
    public enum Tread {
        ROAD(0.028, 0), PERFORMANCE(0.032, 1), WINTER(0.026, 2), ALL_TERRAIN(0.045, 3), MUD_TERRAIN(0.07, 6), SLICK(0, -3), AIRCRAFT_RIB(0, -1);
        final double pitchM, offsetDb;
        Tread(double pitchM, double offsetDb) { this.pitchM = pitchM; this.offsetDb = offsetDb; }
        public double pitchM() { return pitchM; }
    }

    public record Spec(Tread tread, double radiusM, double widthM, long seed) {}

    /**
     * How it rolls: speed over the ground (m/s), slip speed of the contact (m/s), the load (N) and its rated load
     * (N), the surface, wet, and whether the wheel is locked (not turning while the car moves).
     */
    public record State(double speedMps, double slipMps, double loadN, double ratedLoadN, Surface surface, boolean wet, boolean locked) {
        public static final State STILL = new State(0, 0, 0, 1, Surface.ROCK, false, false);
    }

    private final Spec spec;
    private volatile State state = State.STILL;
    private final Noise noise;
    private final double[] pitches;
    private int pitchIndex;
    private double blockPhase, speed, slip, loadShare, squealPhase, squealWobble, grainTimer;
    private final Filters.Biquad belt = Filters.Biquad.bandpass(950, 1.2), roar = Filters.Biquad.lowpass(500, 0.7);
    private final Filters.Biquad wetHiss = Filters.Biquad.highpass(3000, 0.7), scrub = Filters.Biquad.bandpass(1600, 0.6);
    private final Filters.Biquad grainBand = Filters.Biquad.bandpass(2800, 1.5), lowCut = Filters.Biquad.highpass(90, 0.7);
    private final Modal pebble = new Modal(new double[]{2100, 3400}, new double[]{0.01, 0.006}, new double[]{1, 0.5});

    public TyreNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
        pitches = new double[61];
        double[] lengths = {0.88, 1, 1.12};
        for (int i = 0; i < pitches.length; i++) { pitches[i] = lengths[(int) (noise.uniform() * 3)]; }
    }

    public void set(State s) { if (s != null) { state = s; } }

    /** The tread block passing frequency at a speed, Hz (0 for a slick). */
    public static double blockHz(Tread tread, double speedMps) { return tread.pitchM() > 0 ? Math.abs(speedMps) / tread.pitchM() : 0; }

    /** How much the contact squeals (0-1): slip speeds past 1 m/s with load on it, not on loose ground. */
    public static double squeal(State st) {
        if (!st.surface().rings() || st.wet()) { return 0; }
        double s = Math.clamp((Math.abs(st.slipMps()) - 0.8) / 3, 0, 1);
        return s * Math.min(1, st.loadN() / Math.max(1, 0.3 * st.ratedLoadN()));
    }

    public static double soundPowerDb(Spec spec, State st) {
        double v = Math.abs(st.speedMps());
        if (v < 0.3 && Math.abs(st.slipMps()) < 0.3) { return -100; }
        double surface = switch (st.surface()) {
            case GRAVEL -> 6; case SAND -> -2; case SNOW -> -3; case ICE -> -6; case SOIL, MUD -> 0; case WOOD -> 2; case STEEL -> 3; default -> 0;
        };
        double rolling = 94 + 33 * Math.log10(Math.max(v, 0.5) / 22.2) + spec.tread().offsetDb + surface + 10 * Math.log10(Math.max(0.2, spec.widthM() / 0.2)) + (st.wet() ? 3 : 0);
        double sq = squeal(st), squealDb = sq > 0 ? 96 + 8 * sq : -100;
        double drag = st.locked() && v > 1 ? 92 + 20 * Math.log10(v / 10) : -100;
        double dig = !st.surface().rings() && Math.abs(st.slipMps()) > 1 ? 88 + 15 * Math.log10(Math.abs(st.slipMps())) : -100;
        return Dsp.sumDb(rolling, squealDb, drag, dig);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.03);
        double sq = squeal(st);
        double rough = switch (st.surface()) { case GRAVEL -> 1.6; case SOIL, MUD -> 0.9; case SAND -> 0.6; case SNOW -> 0.4; case ICE -> 0.15; case WOOD, STEEL -> 1.1; default -> 1; };
        double squealHz = 700 + 400 * Math.min(1, Math.abs(st.slipMps()) / 6) + 150 * Math.min(1, st.loadN() / Math.max(1, st.ratedLoadN()));
        boolean loose = !st.surface().rings();
        for (int i = 0; i < n; i++) {
            speed += (Math.abs(st.speedMps()) - speed) * follow;
            slip += (Math.abs(st.slipMps()) - slip) * follow;
            loadShare += (sq - loadShare) * follow;
            double moving = Math.min(1, speed / 2);
            // tread blocks
            double impact = 0;
            if (spec.tread().pitchM() > 0) {
                blockPhase += speed / (spec.tread().pitchM() * pitches[pitchIndex]) / RATE;
                if (blockPhase >= 1) {
                    blockPhase -= Math.floor(blockPhase);
                    pitchIndex = (pitchIndex + 1) % pitches.length;
                    impact = 1 + 0.2 * noise.white();
                }
            }
            double tone = belt.process(impact * 6 + noise.white() * 0.3) * moving;
            double road = lowCut.process(roar.process(noise.pink())) * rough * moving * 0.6;
            double water = st.wet() ? wetHiss.process(noise.white()) * 0.5 * moving : 0;
            // grains on loose or coarse ground
            double grains = 0;
            if (loose || st.surface() == Surface.GRAVEL) {
                grainTimer -= speed * (st.surface() == Surface.GRAVEL ? 60 : 25) / RATE;
                if (grainTimer <= 0) { grainTimer += noise.uniform() * 2; pebble.strike(0.05 * (0.5 + noise.uniform())); }
                grains = pebble.next(0) + grainBand.process(noise.white()) * 0.05 * moving * (1 + slip);
            }
            // squeal: a stick-slip tone with a wandering pitch and its harmonics
            double squeal = 0;
            if (loadShare > 1e-3) {
                squealWobble += (noise.white() * 0.02 - squealWobble * 0.0005);
                squealPhase += squealHz * (1 + 0.03 * Math.sin(squealWobble)) / RATE;
                squealPhase -= Math.floor(squealPhase);
                double p = 2 * Math.PI * squealPhase;
                squeal = loadShare * 0.35 * (Math.sin(p) + 0.45 * Math.sin(2 * p) + 0.2 * Math.sin(3 * p)) * (0.85 + 0.15 * noise.white());
            }
            double drag = st.locked() && speed > 1 ? scrub.process(noise.white()) * 0.6 : 0;
            double sample = tone + road + water + grains + squeal + drag;
            out[offset + i] = (float) Dsp.softClip(sample * 0.5);
        }
    }
}

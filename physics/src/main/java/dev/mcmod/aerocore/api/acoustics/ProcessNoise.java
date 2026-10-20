package dev.mcmod.aerocore.api.acoustics;

/**
 * Machines at work (0.37): each process sounds by what makes the noise in it (machining and shop acoustics, e.g.
 * Bies and Hansen ch. 11; tooth passing and chatter in Altintas, Manufacturing Automation), over the hum of the drive:
 * <ul>
 * <li>Cutting with teeth (milling, sawing, hobbing, broaching): impacts at the tooth passing frequency
 *     (teeth x spindle rev/s) ringing the tool and work (here modes near 1.6, 3.3 and 5.1 kHz); turning: the chip's
 *     continuous tearing (2-5 kHz) with chips breaking; drilling and tapping: a hiss and squeal; grinding and finishing:
 *     the grits' high hiss (4-9 kHz) and sparks.</li>
 * <li>Strokes (planers, shapers, slotters, presses, riveters): a stroke's swish and a clunk at each reversal or blow.</li>
 * <li>Discharges and jets: spark erosion's crackle at its discharge rate, a plasma arc's harsh hiss and buzz, a laser's
 *     assist gas, a waterjet's roar (a 4000 bar jet), MIG welding's short-circuit crackle ("frying"), TIG's buzz.</li>
 * <li>Heat and flow: a burner's low roar (combustion noise, 50-300 Hz), steam's hiss and an engine's exhaust beat,
 *     a pump's hum and flow, a fan's whoosh and blade tone, a spray gun's hiss.</li>
 * <li>Drives: an electric motor's hum (twice the line frequency and the slots), a belt's slap at its pass frequency, a
 *     crane's hoist whine and chain, a transformer or generator on the grid humming at 100 Hz and its harmonics.</li>
 * </ul>
 * Level: 70 + 10 log10(P_kW) dB plus the process's offset (a waterjet +22, plasma +16, grinding and sawing +8, a
 * burner +6, a laser -6, spark erosion -10), so a 10 kW lathe turning is about 80 dB of sound power, a plasma table
 * near 105: in line with machine shop measurements of 75-95 dB(A) at the operator.
 */
public final class ProcessNoise implements Voice {
    public enum Process {
        TURNING(0), MILLING(2), DRILLING(0), BORING(-2), GRINDING(8), FINISHING(2), GEAR_CUTTING(4), STROKE(4), SAWING(8),
        SPARK_EROSION(-10), ELECTROCHEMICAL(-12), LASER(-6), PLASMA(16), WATERJET(22), THREADING(0), RIVET_SQUEEZE(2),
        RIVET_IMPACT(12), RIVET_DRILL(6), WELD_MIG(6), WELD_TIG(-2), BURNER(6), PRESS(10), STEAM(8), PUMP(-2), FAN(0),
        SPRAY(4), CRANE(0), HUM(-8), BELT(-4), COMPRESSOR(6), CURING(-10);

        final double offsetDb;
        Process(double offsetDb) { this.offsetDb = offsetDb; }
    }

    /** A machine: its process, power (W), cutting teeth (or blades), the spindle's or drive's speed (rev/s), its stroke rate (Hz). */
    public record Spec(Process process, double powerW, int teeth, double revPerS, double strokeHz, long seed) {}

    /** Working (0-1 of its power), or only running idle (its drive turning). */
    public record State(double load, boolean running) {
        public static final State OFF = new State(0, false);
    }

    private final Spec spec;
    private volatile State state = State.OFF;
    private final Noise noise;
    private final Modal ring = new Modal(new double[]{1600, 3300, 5100}, new double[]{0.02, 0.012, 0.008}, new double[]{1, 0.6, 0.4});
    private final Modal clunk = new Modal(new double[]{180, 420, 950}, new double[]{0.15, 0.08, 0.05}, new double[]{1, 0.5, 0.3});
    private final Filters.Svf band = new Filters.Svf(3000, 0.5), low = new Filters.Svf(150, 0.6);
    private final Filters.Biquad high = Filters.Biquad.highpass(4000, 0.7);
    private final Oscillator hum = new Oscillator(), tooth = new Oscillator(0.3), stroke = new Oscillator(0.5), blade = new Oscillator(0.7);
    private double load, run, burst, retuned = -1, wander;

    public ProcessNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public Spec spec() { return spec; }
    public void set(State s) { if (s != null) { state = s; } }

    public static double soundPowerDb(Spec spec, State st) {
        if (!st.running() && st.load() <= 0) { return -100; }
        double kw = Math.max(0.05, spec.powerW() / 1000);
        double working = Math.clamp(st.load(), 0, 1);
        return 70 + 10 * Math.log10(kw) + spec.process().offsetDb * (0.3 + 0.7 * working) - 8 * (1 - working);
    }

    private void tune() {
        double f = switch (spec.process()) {
            case TURNING, BORING -> 3200; case DRILLING, THREADING -> 2600; case GRINDING, FINISHING -> 6500; case SAWING -> 2200;
            case PLASMA -> 5000; case WATERJET -> 3000; case LASER, SPRAY -> 6000; case STEAM -> 2500; case PUMP -> 900; case FAN -> 700;
            case WELD_MIG -> 3500; case WELD_TIG -> 2800; case COMPRESSOR -> 1200; default -> 2500;
        };
        band.tune(f, 0.55);
        low.tune(spec.process() == Process.BURNER ? 120 : 200, 0.6);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.15);
        if (retuned < 0) { tune(); retuned = 1; }
        double rev = Math.max(0, spec.revPerS());
        for (int i = 0; i < n; i++) {
            load += (Math.clamp(st.load(), 0, 1) - load) * follow;
            run += ((st.running() || st.load() > 0 ? 1 : 0) - run) * follow;
            band.process(noise.white());
            low.process(noise.pink());
            wander += (noise.white() * 0.5 - wander) * 0.0002;
            // the drive: a motor's hum while running
            double s = run * 0.12 * (Math.sin(2 * Math.PI * hum.advance(100)) + 0.3 * Math.sin(4 * Math.PI * hum.phase()));
            double w = load;
            switch (spec.process()) {
                case TURNING, BORING -> {
                    s += w * band.band * (0.8 + 0.4 * wander);
                    if (noise.chance(w * 6.0 / RATE)) { ring.strike(0.3); }
                    s += ring.next(0);
                }
                case MILLING, GEAR_CUTTING, SAWING -> {
                    double tp = Math.max(1, spec.teeth()) * rev;
                    double before = tooth.phase();
                    tooth.advance(tp);
                    if (tooth.phase() < before) { ring.strike(0.25 * w * (0.8 + 0.4 * noise.uniform())); }
                    s += ring.next(0) + w * 0.4 * band.band + (spec.process() == Process.SAWING ? 0.2 * w * tooth.saw(0) : 0);
                }
                case DRILLING, THREADING -> s += w * (band.band * 0.7 + 0.15 * Math.sin(2 * Math.PI * tooth.advance(2400 + 300 * wander)));
                case GRINDING, FINISHING -> {
                    s += w * high.process(noise.white()) * 0.9 + 0.1 * run * Math.sin(2 * Math.PI * tooth.advance(rev * 2));
                    if (noise.chance(w * 300.0 / RATE)) { burst = 0.4; }
                    burst *= 0.97;
                    s += burst * high.process(noise.white());
                }
                case STROKE, PRESS, RIVET_SQUEEZE -> {
                    double before = stroke.phase();
                    stroke.advance(Math.max(0.2, spec.strokeHz()));
                    if (stroke.phase() < before) { clunk.strike(0.8 * Math.max(0.2, w)); }
                    s += clunk.next(0) + w * 0.3 * band.band * Math.max(0, Math.sin(2 * Math.PI * stroke.phase()));
                }
                case RIVET_IMPACT -> {
                    double before = stroke.phase();
                    stroke.advance(25);
                    if (stroke.phase() < before && w > 0.05) { ring.strike(0.6 * w); clunk.strike(0.2 * w); }
                    s += ring.next(0) + clunk.next(0);
                }
                case RIVET_DRILL -> s += w * (band.band * 0.6 + 0.2 * Math.sin(2 * Math.PI * tooth.advance(1800)));
                case SPARK_EROSION, ELECTROCHEMICAL -> {
                    if (noise.chance(w * 2000.0 / RATE)) { burst = 0.5 + 0.5 * noise.uniform(); }
                    burst *= 0.8;
                    s += burst * noise.white() * 0.6 + 0.2 * run * low.band;
                }
                case LASER -> s += w * band.band * 0.4 + 0.1 * run * Math.sin(2 * Math.PI * tooth.advance(50));
                case PLASMA -> s += w * (band.band * 1.0 + 0.3 * tooth.saw(150) + high.process(noise.white()) * 0.5);
                case WATERJET -> s += w * (band.band * 1.0 + low.band * 0.8);
                case WELD_MIG -> {
                    if (noise.chance(w * 120.0 / RATE)) { burst = 1; }
                    burst *= 0.92;
                    s += w * (band.band * 0.4 + burst * high.process(noise.white()) * 1.2);
                }
                case WELD_TIG -> s += w * (0.25 * tooth.saw(100) + band.band * 0.3);
                case BURNER -> s += w * low.band * 2.0 * (1 + 0.3 * wander);
                case STEAM -> {
                    double before = stroke.phase();
                    stroke.advance(Math.max(0.5, 4 * rev));
                    if (stroke.phase() < before) { burst = 1; }
                    burst *= 0.9993;
                    s += w * (band.band * (0.2 + burst) + 0.1 * low.band);
                }
                case PUMP, COMPRESSOR -> s += run * (0.3 * low.band + 0.2 * band.band * w + 0.15 * Math.sin(2 * Math.PI * tooth.advance(Math.max(1, rev) * Math.max(1, spec.teeth()))));
                case FAN -> s += run * (0.5 * band.band + 0.2 * Math.sin(2 * Math.PI * blade.advance(Math.max(1, rev) * Math.max(2, spec.teeth()))));
                case SPRAY -> s += w * high.process(noise.white()) * 0.8;
                case CRANE -> {
                    s += w * (0.2 * Math.sin(2 * Math.PI * tooth.advance(Math.max(50, rev * 40))) + 0.15 * band.band);
                    if (noise.chance(w * 20.0 / RATE)) { clunk.strike(0.05); }
                    s += clunk.next(0);
                }
                case HUM -> s += run * 0.3 * (Math.sin(2 * Math.PI * blade.advance(100)) + 0.5 * Math.sin(4 * Math.PI * blade.phase()) + 0.25 * Math.sin(6 * Math.PI * blade.phase()));
                case BELT -> {
                    double before = stroke.phase();
                    stroke.advance(Math.max(0.2, rev * 1.5));
                    if (stroke.phase() < before) { clunk.strike(0.08 * run); }
                    s += clunk.next(0) + run * 0.15 * low.band;
                }
                case CURING -> s += run * 0.2 * low.band;
            }
            out[offset + i] = (float) Dsp.softClip(0.6 * s);
        }
    }
}

package dev.mcmod.aerocore.api.acoustics;

/**
 * A gas turbine's sound (0.37), the sum of its sources (Smith, Aircraft Noise, 1989):
 * <ul>
 * <li>Jet mixing: Lighthill's eighth power, W = K rho_a A U^8 / c^5 with K about 3e-5 for cold jets and an extra
 *     factor (T_j / T_a)^1 toward 2 for hot ones at moderate speeds (Tanna 1977), broad, peaking at a Strouhal number
 *     of 0.2 on the nozzle diameter, loudest 30-40 degrees off the jet's axis aft (here a directivity of +6 dB there,
 *     -10 dB ahead). An afterburner's jet (faster, hotter) and a rocket's or ramjet's roar are the same law.</li>
 * <li>Fan and compressor: tones at the blade passing frequency (blades x shaft rev/s) and its harmonics, loudest
 *     ahead; with the fan's tip relative speed supersonic, "buzz-saw" tones at every shaft order (the shocks ahead of
 *     the blades are not all alike). Level rising with tip Mach to the sixth power.</li>
 * <li>Turbine: a high whine at its blade passing frequency, aft. Combustion: a low rumble (a few hundred Hz).</li>
 * <li>A pulsejet: AeroCore's Pulsejet cycle's frequency and pressure swing give its buzz directly.</li>
 * </ul>
 */
public final class TurbineNoise implements Voice {
    public enum Kind { TURBOJET, TURBOFAN, TURBOSHAFT, RAMJET, ROCKET, PULSEJET }

    /**
     * An engine: kind, nozzle diameter (m), fan or first-stage blades and their tip diameter (m), turbine blades,
     * the shaft's design speed (rev/s).
     */
    public record Spec(Kind kind, double nozzleM, int fanBlades, double fanTipM, int turbineBlades, double designRevPerS, long seed) {}

    /**
     * Running: spool speed (rev/s; the fan's on a turbofan), jet speed (m/s) and temperature (K) at the nozzle, the
     * share of the reheat or flame (0-1), the angle (rad) between the jet's axis aft and the listener, the pulsejet's
     * firing frequency (Hz) and whether it fires.
     */
    public record State(double revPerS, double jetMps, double jetK, double flame, double angleRad, double pulseHz, boolean lit) {
        public static final State OFF = new State(0, 0, 288, 0, Math.PI / 2, 0, false);
    }

    private final Spec spec;
    private volatile State state = State.OFF;
    private final Noise noise;
    private final Filters.Svf jet = new Filters.Svf(300, 0.5), roarLow = new Filters.Svf(60, 0.7);
    private final Oscillator fan = new Oscillator(), shaft = new Oscillator(0.2), turbine = new Oscillator(0.4), pulse = new Oscillator();
    private final Filters.Biquad combustor = Filters.Biquad.bandpass(250, 0.8), crackle = Filters.Biquad.highpass(1500, 0.7);
    private double rev, u, flame, swell, crack, retune, pulsePhaseEnv;

    public TurbineNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public Spec spec() { return spec; }
    public void set(State s) { if (s != null) { state = s; } }

    /** Jet mixing sound power (W), Lighthill with a hot-jet factor. */
    public static double jetPowerW(double jetMps, double jetK, double nozzleM) {
        if (jetMps < 1) { return 0; }
        double area = Math.PI * nozzleM * nozzleM / 4, rho = 1.2, c = 343;
        double hot = Math.min(2, Math.max(1, jetK / 288));
        return 3e-5 * hot * rho * area * Math.pow(jetMps, 8) / Math.pow(c, 5);
    }

    /** Directivity (dB) of jet noise at an angle from the jet's axis aft: +6 at 35 degrees, falling ahead. */
    public static double jetDirectivityDb(double angleRad) {
        double deg = Math.toDegrees(Math.abs(angleRad));
        return deg < 35 ? 6 * deg / 35 - 2 : 6 - 16 * (deg - 35) / 145;
    }

    public static double soundPowerDb(Spec spec, State st) {
        if (!st.lit() && st.revPerS() < 1) { return -100; }
        double jetDb = Dsp.powerLevelDb(jetPowerW(st.jetMps(), st.jetK(), spec.nozzleM())) + jetDirectivityDb(st.angleRad());
        double tip = Math.PI * spec.fanTipM() * st.revPerS() / 343;
        double fanDb = spec.fanBlades() > 0 ? 110 + 60 * Math.log10(Math.max(0.05, tip)) + 10 * Math.log10(Math.max(0.01, spec.fanTipM() / 1.5)) * 2 : -100;
        if (st.angleRad() < Math.toRadians(90)) { fanDb -= 6; }   // fan tones go forward
        double pulseDb = spec.kind() == Kind.PULSEJET && st.lit() ? 130 + 10 * Math.log10(Math.max(0.05, spec.nozzleM() / 0.4)) : -100;
        return Dsp.sumDb(jetDb, fanDb, pulseDb);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.15), fast = Dsp.smoothing(0.03);
        for (int i = 0; i < n; i++) {
            rev += (st.revPerS() - rev) * follow;
            u += (st.jetMps() - u) * follow;
            flame += (Math.clamp(st.flame(), 0, 1) - flame) * fast;
            if (--retune <= 0) {
                retune = 64;
                double peak = 0.2 * Math.max(u, 5) / Math.max(0.02, spec.nozzleM());
                jet.tune(Math.min(peak, 8000), 0.45);
            }
            swell += (noise.white() * 0.5 - swell) * 0.00005;
            double s = 0;
            // jet mixing: broadband about its Strouhal peak, a slow swell; a reheat or rocket's crackle
            jet.process(noise.pink());
            roarLow.process(noise.brown());
            double jetLevel = Math.min(1, u / 300);
            s += jetLevel * (1 + 0.3 * swell) * (jet.band * 1.2 + roarLow.low * 0.8);
            if ((spec.kind() == Kind.ROCKET || flame > 0.2) && noise.chance(60.0 / RATE * (0.5 + flame))) { crack = 0.6 + 0.4 * noise.uniform(); }
            crack *= 0.94;
            s += crack * crackle.process(noise.white()) * (spec.kind() == Kind.ROCKET ? 1 : flame);
            // fan and compressor tones, buzz-saw when the tips are supersonic
            if (spec.fanBlades() > 0 && rev > 0.5) {
                double tipMach = Math.PI * spec.fanTipM() * rev / 343;
                double bpf = spec.fanBlades() * rev, f = 0.25 * Math.min(1, tipMach);
                double t = fan.sine(Math.min(bpf, 0.45 * RATE)) + 0.4 * Math.sin(4 * Math.PI * fan.phase());
                s += f * t;
                if (tipMach > 1) {
                    double p = shaft.saw(rev);
                    s += 0.15 * Math.min(1, (tipMach - 1) * 3) * p;
                }
            }
            if (spec.turbineBlades() > 0 && rev > 0.5) { s += 0.05 * turbine.sine(Math.min(spec.turbineBlades() * rev * 1.3, 0.45 * RATE)); }
            if (st.lit() && spec.kind() != Kind.ROCKET) { s += 0.25 * combustor.process(noise.white()) * (0.4 + 0.6 * Math.min(1, u / 400)); }
            // a pulsejet's buzz: a sharp pressure pulse each cycle, decaying
            if (spec.kind() == Kind.PULSEJET && st.lit()) {
                double before = pulse.phase();
                pulse.advance(Math.max(5, st.pulseHz()) * (1 + 0.02 * noise.white()));
                if (pulse.phase() < before) { pulsePhaseEnv = 1; }
                pulsePhaseEnv *= Math.exp(-7 * Math.max(5, st.pulseHz()) / RATE);
                s += 0.9 * pulsePhaseEnv * (1 + 0.35 * noise.white());
            }
            out[offset + i] = (float) Dsp.softClip(0.6 * s);
        }
    }
}

package dev.mcmod.aerocore.api.acoustics;

/**
 * A hydraulic pump and its circuit as heard on a construction machine (0.43):
 * <ul>
 * <li>The pump's ripple: each piston (or a gear pump's tooth, a vane pump's vane) delivers its slug of oil in turn, and
 *     the oil in each chamber is compressed from inlet to delivery pressure as the chamber opens to the delivery port,
 *     so the delivered flow, and with it the pressure in the lines, pulsates at the pumping frequency f = n z (n the
 *     shaft's rev/s, z the pistons: 9 in a typical axial piston pump; a gear pump's teeth) and its harmonics. The
 *     kinematic ripple of an odd piston count repeats at 2 n z, but at working pressures the compression of each
 *     chamber's oil, at n z, dominates. This fluid-borne noise is what the lines, the valve blocks and the machine's
 *     frame radiate (Edge and Johnston 1990, "The 'secondary source' method for the measurement of pump pressure
 *     ripple characteristics", Proc. IMechE 204 A; ISO 10767-1; Skaistis, Noise Control of Hydraulic Machinery, 1988).
 *     The harmonics fall with their order, here 1/k in amplitude at rated pressure (6 dB an octave), steeper unloaded
 *     where the compression is gentler; small differences between the pistons add the shaft's orders as sidebands at
 *     f +- n.</li>
 * <li>A relief valve passing oil: the jet through its seat at the full pressure drop is turbulent and cavitates, a
 *     broadband hiss above 2 kHz with the squeal of a chattering poppet.</li>
 * <li>A cylinder bottoming ({@link #bottomOut}): the piston meets its end cap, a clunk of heavy steel's modes and the
 *     pressure spike's bang in the lines.</li>
 * </ul>
 * Level ({@link #soundPowerDb}): the pump's {@link #pumpPowerDb} and the relief's {@link #reliefPowerDb} summed as
 * powers. The voice's output stays near -12 dBFS RMS while the pump turns; its balance between the pump and the relief
 * follows their sound powers.
 */
public final class HydraulicNoise implements Voice {
    /**
     * The pump's kind: a level offset (dB) and how fast its harmonics fall at rated pressure (the exponent of 1/k).
     * Rules: vane pumps are the quietest kind, external gear pumps whine with strong harmonics (Skaistis 1988 discusses
     * each kind's sources; the numbers here are not from a table).
     */
    public enum Kind {
        AXIAL_PISTON(0, 1.0), GEAR(2, 0.9), VANE(-4, 1.2);

        final double offsetDb, fall;
        Kind(double offsetDb, double fall) { this.offsetDb = offsetDb; this.fall = fall; }
    }

    /**
     * A pump: its kind, pistons (a gear pump's teeth, a vane pump's vanes; the z of f = n z), the rated pressure (Pa,
     * the relief valve's setting) and the seed of its noise.
     */
    public record Spec(Kind kind, int pistons, double ratedPressurePa, long seed) {
        /** An axial piston pump: 9 pistons, the common count of swash-plate and bent-axis pumps. */
        public static Spec axialPiston(double ratedPressurePa, long seed) { return new Spec(Kind.AXIAL_PISTON, 9, ratedPressurePa, seed); }
        /** An external gear pump of a tooth count (10-14 are usual). */
        public static Spec gear(int teeth, double ratedPressurePa, long seed) { return new Spec(Kind.GEAR, teeth, ratedPressurePa, seed); }
    }

    /**
     * How it runs: the shaft's speed (rad/s), the delivery pressure (Pa), the delivered flow (m^3/s) and the share of
     * that flow going over the relief valve (0: none, 1: all of it, the circuit stalled at relief).
     */
    public record State(double shaftRadPerS, double pressurePa, double flowM3PerS, double relieving) {
        /** Not turning, no pressure. */
        public static final State OFF = new State(0, 0, 0, 0);
    }

    /** Most harmonics of the pumping frequency drawn. */
    private static final int HARMONICS = 32;
    /** The harmonics fade out over TAPER_HZ below TOP_HZ (above, the pump's broadband noise covers them). */
    private static final double TOP_HZ = 9000, TAPER_HZ = 2000;
    /** The level's reference point: 82.5 dB at 28 MPa, 1 litre a second and 25 rev/s (see pumpPowerDb). */
    private static final double REF_DB = 82.5, REF_PA = 28e6, REF_FLOW = 1e-3, REF_REV = 25;
    /** An unloaded pump still turns against a standby pressure (a load-sensing circuit's 2-3 MPa) and some flow. */
    private static final double STANDBY_PA = 2.5e6, LEAST_FLOW = 2e-5;
    /** Mineral hydraulic oil (ISO VG 46 near 40 C): density (kg/m^3) and speed of sound (m/s). */
    private static final double OIL_DENSITY = 870, OIL_SOUND_MPS = 1400;
    /** IEC 60534-8-4's turbulent efficiency exponent, and the loss through the valve block and hoses (dB, a rule). */
    private static final double EFFICIENCY_EXPONENT = -4.6, RELIEF_LOSS_DB = 25;
    /** Limits that keep wild inputs finite: shaft rev/s, pressure (Pa), flow (m^3/s). */
    private static final double MOST_REV = 1000, MOST_PA = 1e9, MOST_FLOW = 10;

    private final Spec spec;
    private final Kind kind;
    private final int z;
    private final double rated;
    private volatile State state = State.OFF;
    private volatile double bottomPending;
    private final Noise noise;
    private final Oscillator pump = new Oscillator(), shaft = new Oscillator(0.25), squeal = new Oscillator(0.5);
    private final double[] weight = new double[HARMONICS];
    private final Filters.Svf flowBand = new Filters.Svf(350, 0.6), spikeBand = new Filters.Svf(700, 0.8);
    private final Filters.Biquad hiss = Filters.Biquad.highpass(2000, 0.7), seat = Filters.Biquad.bandpass(4500, 0.9);
    private final Modal clunk = new Modal(new double[]{140, 380, 820, 1650}, new double[]{0.3, 0.18, 0.1, 0.06}, new double[]{1, 0.6, 0.35, 0.2});
    private final double beatPhase;
    private double rev, load, pumpMix, reliefMix, spike, wobble;
    private int count, retune;

    /** A pump's voice; its noise (and the pistons' beat) comes from the spec's seed. */
    public HydraulicNoise(Spec spec) {
        this.spec = spec;
        this.kind = spec.kind() == null ? Kind.AXIAL_PISTON : spec.kind();
        this.z = Math.clamp(spec.pistons(), 1, 40);
        this.rated = Double.isFinite(spec.ratedPressurePa()) && spec.ratedPressurePa() > 1e5 ? spec.ratedPressurePa() : 35e6;
        this.noise = new Noise(spec.seed());
        this.beatPhase = 2 * Math.PI * noise.uniform();
    }

    /** The pump this voice is. */
    public Spec spec() { return spec; }

    /** Sets how it runs (from any thread; the voice reads the latest at its next block). */
    public void set(State s) { if (s != null) { state = s; } }

    /**
     * A cylinder bottoming now, of a strength 0-1 (its speed against the end cap over a full stroke's speed, say); its
     * level is {@link #bottomOutPowerDb}.
     */
    public void bottomOut(double strength) {
        if (Double.isFinite(strength)) { bottomPending = Math.max(bottomPending, Math.clamp(strength, 0, 1)); }
    }

    private static double finite(double x) { return Double.isFinite(x) ? x : 0; }

    private static double revPerS(State st) { return Math.min(MOST_REV, Math.abs(finite(st.shaftRadPerS())) / (2 * Math.PI)); }

    /** The pumping frequency f = n z (Hz) of a pump of pistons (or teeth, vanes) turning at shaftRadPerS. */
    public static double pumpingHz(int pistons, double shaftRadPerS) {
        return Math.max(1, pistons) * Math.min(MOST_REV, Math.abs(finite(shaftRadPerS)) / (2 * Math.PI));
    }

    private static double pumpW(Spec spec, State st) {
        double n = revPerS(st);
        if (n < 0.5) { return 0; }
        double p = Math.clamp(Math.abs(finite(st.pressurePa())), STANDBY_PA, MOST_PA);
        double q = Math.clamp(Math.abs(finite(st.flowM3PerS())), LEAST_FLOW, MOST_FLOW);
        double offset = spec.kind() == null ? 0 : spec.kind().offsetDb;
        return Dsp.powerW(REF_DB + offset + 20 * Math.log10(p / REF_PA) + 10 * Math.log10(q / REF_FLOW) + 10 * Math.log10(n / REF_REV));
    }

    private static double reliefW(State st) {
        double dp = Math.clamp(finite(st.pressurePa()), 0, MOST_PA);
        double q = Math.clamp(finite(st.relieving()), 0, 1) * Math.min(MOST_FLOW, Math.abs(finite(st.flowM3PerS())));
        if (!(dp > 0) || !(q > 0)) { return 0; }
        double u = Math.sqrt(2 * dp / OIL_DENSITY);
        double efficiency = Math.min(1e-2, Math.pow(10, EFFICIENCY_EXPONENT) * u / OIL_SOUND_MPS);
        return efficiency * q * dp * Math.pow(10, -RELIEF_LOSS_DB / 10);
    }

    /**
     * The pump's sound power level (dB): 82.5 + 20 log10(p / 28 MPa) + 10 log10(Q / 1 l/s) + 10 log10(n / 25 rev/s)
     * plus the kind's offset. The compression ripple is the chambers' dead volume compressed by p / B (B the oil's bulk
     * modulus), so the ripple's flow and pressure grow in proportion to p: 6 dB a doubling of pressure. The flow and
     * speed terms are a rule (more slugs a second, each radiating better at a higher frequency) that makes speed count
     * most, as pump noise studies find: doubling the speed adds 6 dB, the displacement 3 dB. A 71 cm^3 piston pump at
     * 1500 rpm and 280 bar gives 85 dB, about 74 dB at 1 m (the order of piston pumps' data sheet levels). Below
     * 2.5 MPa it stays at its standby level; -100 when the shaft stands.
     */
    public static double pumpPowerDb(Spec spec, State st) {
        double w = pumpW(spec, st);
        return w > 0 ? Dsp.powerLevelDb(w) : -100;
    }

    /**
     * The relief valve's sound power level (dB) after IEC 60534-8-4's prediction for a liquid through a valve: the
     * stream's power Q_r dp (Q_r the relieved flow, dp the pressure dropped to tank) times the turbulent acoustic
     * efficiency 10^-4.6 U / c_L (U = sqrt(2 dp / rho) the jet's speed, c_L oil's speed of sound; the standard is
     * validated on water at far lower pressures, so this is an extrapolation), less 25 dB through the valve block and
     * hoses (a rule). All of 4 l/s over a 35 MPa relief gives about 93 dB, a little over the pump's own 91 dB at that
     * point. -100 when nothing is relieved.
     */
    public static double reliefPowerDb(Spec spec, State st) {
        double w = reliefW(st);
        return w > 0 ? Dsp.powerLevelDb(w) : -100;
    }

    /** The circuit's sound power level (dB): the pump and the relief summed as powers; -100 when silent. */
    public static double soundPowerDb(Spec spec, State st) {
        double w = pumpW(spec, st) + reliefW(st);
        return w > 0 ? Dsp.powerLevelDb(w) : -100;
    }

    /**
     * The sound power level (dB) of a cylinder bottoming: the moving mass (kg, the boom, arm or load as seen at the rod)
     * stopping from a speed (m/s) against the end cap; 1e-4 of its kinetic energy radiates over the clunk's 0.15 s (a
     * rule, as WaterNoise's slam).
     */
    public static double bottomOutPowerDb(double movingMassKg, double speedMps) {
        double v = Math.min(1e3, Math.abs(finite(speedMps))), energy = 0.5 * Math.min(1e9, Math.abs(finite(movingMassKg))) * v * v;
        return energy > 0 ? Dsp.powerLevelDb(1e-4 * energy / 0.15) : -100;
    }

    /** The harmonics' amplitudes (normalised to an RMS of 1) for the present speed and load. */
    private void weigh() {
        double fp = z * rev;
        double fall = kind.fall + 0.6 * (1 - Math.min(1, load));
        double sum = 0;
        int top = 0;
        for (int k = 1; k <= HARMONICS; k++) {
            double taper = Math.clamp((TOP_HZ - k * fp) / TAPER_HZ, 0, 1);
            double a = taper > 0 ? taper * Math.pow(k, -fall) : 0;
            weight[k - 1] = a;
            if (a > 0) { top = k; }
            sum += a * a;
        }
        double g = sum > 0 ? 1 / Math.sqrt(sum / 2) : 0;
        for (int k = 0; k < top; k++) { weight[k] *= g; }
        count = top;
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.05), slow = Dsp.smoothing(0.08);
        double revTarget = revPerS(st);
        double pw = pumpW(spec, st), rw = reliefW(st), total = pw + rw;
        double pumpTarget = total > 0 ? Math.sqrt(pw / total) : 0, reliefTarget = total > 0 ? Math.sqrt(rw / total) : 0;
        double loadTarget = Math.clamp(Math.abs(finite(st.pressurePa())) / rated, 0, 1.5);
        double squealHz = 2300 + 1200 * Math.min(1, loadTarget);
        if (bottomPending > 0) {
            double b = bottomPending;
            bottomPending = 0;
            clunk.strike(0.35 * b);
            spike = Math.max(spike, b);
        }
        for (int i = 0; i < n; i++) {
            rev += (revTarget - rev) * follow;
            load += (loadTarget - load) * follow;
            pumpMix += (pumpTarget - pumpMix) * slow;
            reliefMix += (reliefTarget - reliefMix) * slow;
            if (--retune <= 0) { retune = 64; weigh(); }
            // the ripple: harmonics of n z by the sine recurrence sin((k+1) t) = 2 cos t sin(k t) - sin((k-1) t)
            pump.advance(z * rev);
            shaft.advance(rev);
            double t = 2 * Math.PI * pump.phase(), sk = Math.sin(t), twoCos = 2 * Math.cos(t), before = 0, ripple = 0;
            for (int k = 0; k < count; k++) {
                ripple += weight[k] * sk;
                double next = twoCos * sk - before;
                before = sk;
                sk = next;
            }
            double turning = Math.min(1, rev / 2);
            double sp = 2 * Math.PI * shaft.phase();
            double beat = 1 + 0.06 * Math.sin(sp + beatPhase) + 0.03 * Math.sin(2 * sp + 1.3);
            double s = 0.25 * ripple * beat * turning * pumpMix * (0.7 + 0.3 * Math.min(1, load));
            // the oil's flow through the lines: a low broadband rush
            flowBand.process(noise.pink());
            s += 0.06 * flowBand.band * turning * pumpMix;
            // the relief: the seat's hiss and a chattering poppet's wandering squeal
            if (reliefMix > 1e-6) {
                double w = noise.white();
                wobble += noise.white() * 0.02 - wobble * 0.0005;
                double q = 2 * Math.PI * squeal.advance(squealHz * (1 + 0.04 * Math.sin(wobble)));
                double tone = Math.sin(q) + 0.35 * Math.sin(2 * q);
                s += reliefMix * (0.4 * hiss.process(w) + 0.3 * seat.process(w) + 0.12 * tone * (0.8 + 0.2 * noise.white()));
            }
            // a cylinder bottoming: the clunk and the lines' bang
            s += clunk.next(0);
            if (spike > 1e-5) {
                spikeBand.process(noise.white());
                s += 0.8 * spike * spikeBand.band;
                spike *= 0.9985;
            }
            out[offset + i] = (float) Dsp.softClip(s);
        }
    }
}

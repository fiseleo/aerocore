package dev.mcmod.aerocore.api.acoustics;

/**
 * A four-stroke piston engine's sound (0.37), made from its firing events (physically informed synthesis in the
 * manner of Baldan, Lachambre, Delle Monache and Boussard 2015: pulses through pipe models, not recordings):
 * <ul>
 * <li>Each cylinder's exhaust valve opens at its own crank angle in the 720-degree cycle ({@link #firingAngles} works
 *     those out from where each cylinder's throw puts its top dead centre), and the cylinder's blowdown sends a pressure
 *     pulse into its bank's collector: an alpha function rising over about 25 crank degrees, carrying the turbulent
 *     rush of the gas (noise shaped by the same envelope, more of it at high load: the rasp), with a cycle-to-cycle
 *     spread of 6 % in strength and about a degree in timing (no two firings alike, so the firing harmonics are not
 *     pure tones). Its strength follows the cylinder pressure at the valve opening, so the load; with the fuel cut on
 *     the overrun, the cylinders only pump, and now and then unburnt fuel pops in the pipe.</li>
 * <li>Each bank's pipe is a waveguide: the pulse returns from the open end inverted after the round trip 2 L / c in
 *     gas at about 800 K (c about 560 m/s), with losses, then a muffler (a low-pass) and the radiation from the tail.</li>
 * <li>Intake: the airbox and runners ring as a Helmholtz resonator struck by every intake stroke, more so with the
 *     throttle open, plus the rush of air past the throttle.</li>
 * <li>Mechanics: the valves closing on their seats, once per cam revolution per valve and not quite evenly, ringing
 *     a few kHz, over the block's broadband rumble; a diesel's
 *     combustion knock (a 1.5-3 kHz burst at each firing, strongest at light load); a petrol engine's detonation rings
 *     the chamber's first circumferential mode, Draper's f = 1.84 c / (pi B) with c about 950 m/s in the burnt gas.</li>
 * <li>A turbocharger whistles: its wheel's speed comes from the boost through Euler's work (slip 0.9, efficiency 0.7),
 *     a hundred thousand rpm and more, so its blade passing frequency is mostly beyond hearing (20-30 kHz); what is
 *     heard is the wheel's second shaft order (its small asymmetries; {@link #turboWhistleHz}) and the blade passing
 *     frequency only when it is below 16 kHz. A blow-off valve hisses when the throttle shuts on boost (without one the
 *     compressor surges, a flutter); a starter whines while it cranks.</li>
 * </ul>
 * The voice's output stays near -12 dBFS RMS at any speed and load (its shape changes, not its level); how loud the
 * engine is comes from {@link #soundPowerDb}: an exhaust term 80 + 10 log10(V_d) + 30 log10(rpm / 1000)
 * + 20 log10(0.25 + load) + the muffler's offset (0 for a road silencer, about +8 sporty, +20 open pipes), which puts a
 * 2-litre road car at full load and 3000 rpm at 99 dB sound power (about 70 dB at 7.5 m, the ISO 362 pass-by range), an
 * intake 8 dB below the exhaust at full throttle, and the mechanical noise. These levels are a calibration to typical
 * vehicles, not a prediction from the cycle.
 */
public final class PistonEngineNoise implements Voice {
    /** Speed of sound in exhaust gas at about 800 K, m/s. */
    public static final double EXHAUST_SOUND_MPS = 560;
    /** Speed of sound in the burnt gas of a detonating cylinder, m/s (Draper's mode). */
    public static final double CHAMBER_SOUND_MPS = 950;

    /**
     * What the engine is: cylinders, each cylinder's exhaust-valve opening angle in the cycle (deg, 0-720) and bank,
     * displacement (l), bore (m), each bank's pipe length (m), the muffler's cutoff (Hz) and its offset to the
     * level (dB), the intake resonance (Hz), diesel, valves per cylinder, the turbo's compressor blades (0: none)
     * and tip diameter (m), and whether a blow-off valve vents on a closing throttle (else the compressor surges).
     */
    public record Spec(int cylinders, double[] openingDeg, int[] bank, int banks, double displacementL, double boreM, double pipeM,
                       double mufflerHz, double mufflerDb, double intakeHz, boolean diesel, int valvesPerCylinder,
                       int turboBlades, double turboTipM, boolean blowOffValve, long seed) {
        public Spec {
            if (cylinders < 1 || openingDeg.length != cylinders || bank.length != cylinders) { throw new IllegalArgumentException("one angle and bank per cylinder"); }
            openingDeg = openingDeg.clone();
            bank = bank.clone();
            banks = Math.max(1, banks);
            for (int b : bank) { if (b < 0 || b >= banks) { throw new IllegalArgumentException("bank out of range"); } }
        }
        /** Firings per second at an engine speed. */
        public double firingHz(double rpm) { return rpm / 120 * cylinders; }
    }

    /**
     * How it runs: crank speed (rpm), load (0-1: the cylinder pressure at the valve opening relative to full load),
     * throttle (0-1), whether it fires (running) or is turned by the starter (cranking), boost (absolute over
     * ambient, 1 without), and detonation (0-1: the share of firings that knock).
     */
    public record State(double rpm, double load, double throttle, boolean running, boolean cranking, double boostRatio, double knock) {
        public static final State OFF = new State(0, 0, 0, false, false, 1, 0);
    }

    private final Spec spec;
    private volatile State state = State.OFF;
    private final Noise noise;
    private final double[] pulseAge, pulseAmp, pulseDelay;
    private final DelayLine[] pipes;
    private final Filters.OnePole[] pipeLoss;
    private final Filters.Biquad[] muffler;
    private final Filters.Biquad radiation = Filters.Biquad.highpass(28, 0.7);
    private final Filters.Biquad intake, hiss = Filters.Biquad.bandpass(2500, 0.8), clatter = Filters.Biquad.bandpass(2200, 1.2);
    private final Filters.Biquad bovBand = Filters.Biquad.bandpass(3500, 0.6), starterBody = Filters.Biquad.bandpass(900, 2);
    private final Modal valves, detonation;
    private final Filters.Biquad block = Filters.Biquad.bandpass(700, 0.5);
    private final Oscillator turbo = new Oscillator(), starter = new Oscillator();
    private double crank, rpm, load, throttle, boost, run, crankingLevel, clatterEnv, popEnv, bovEnv, lastThrottle = 0, valvePhase;
    private final double pipeDelay;

    public PistonEngineNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
        int n = spec.cylinders();
        pulseAge = new double[n];
        pulseAmp = new double[n];
        pulseDelay = new double[n];
        java.util.Arrays.fill(pulseAge, Double.POSITIVE_INFINITY);
        pipes = new DelayLine[spec.banks()];
        pipeLoss = new Filters.OnePole[spec.banks()];
        muffler = new Filters.Biquad[spec.banks()];
        pipeDelay = Math.max(4, 2 * Math.max(0.3, spec.pipeM()) / EXHAUST_SOUND_MPS * RATE);
        for (int b = 0; b < spec.banks(); b++) {
            pipes[b] = new DelayLine((int) pipeDelay + 8);
            pipeLoss[b] = new Filters.OnePole(2500);
            muffler[b] = Filters.Biquad.lowpass(Math.max(80, spec.mufflerHz()), 0.8);
        }
        intake = Filters.Biquad.bandpass(Math.max(40, spec.intakeHz()), 3);
        valves = new Modal(new double[]{3900, 5300, 7100}, new double[]{0.012, 0.008, 0.005}, new double[]{1, 0.6, 0.35});
        double draper = 1.84 * CHAMBER_SOUND_MPS / (Math.PI * Math.max(0.02, spec.boreM()));
        detonation = new Modal(new double[]{draper, draper * 1.66}, new double[]{0.006, 0.004}, new double[]{1, 0.4});
    }

    public Spec spec() { return spec; }
    public void set(State s) { if (s != null) { state = s; } }
    public State state() { return state; }

    /**
     * Each cylinder's top dead centre on the crank (deg, 0-360: its throw plus its bank's angle) to its firing angle
     * in the 720-degree cycle: each fires once in two turns, so a cylinder fires at its angle or a turn later; of the
     * choices (cylinders sharing a crank angle split between the two turns) this takes the one with the most even
     * intervals, the earliest such in cylinder order: an inline four on a flat crank fires at 0, 540, 180, 360 (the order
     * 1-3-4-2), cylinders on a shared throw of a V fire a bank angle apart.
     */
    public static double[] firingAngles(double[] tdcDeg) {
        int n = tdcDeg.length;
        if (n > 20) { throw new IllegalArgumentException("at most 20 cylinders"); }
        double[] base = new double[n];
        for (int i = 0; i < n; i++) { base[i] = ((tdcDeg[i] % 360) + 360) % 360; }
        double[] best = null;
        double bestScore = Double.MAX_VALUE;
        double[] trial = new double[n], sorted = new double[n];
        for (int mask = 0; mask < (1 << n); mask++) {
            if ((mask & 1) != 0) { continue; }   // cylinder 0 fires in the first turn
            for (int i = 0; i < n; i++) { trial[i] = base[i] + (((mask >> i) & 1) == 1 ? 360 : 0); }
            System.arraycopy(trial, 0, sorted, 0, n);
            java.util.Arrays.sort(sorted);
            double ideal = 720.0 / n, score = 0;
            for (int i = 0; i < n; i++) {
                double gap = (i + 1 < n ? sorted[i + 1] : sorted[0] + 720) - sorted[i];
                score += (gap - ideal) * (gap - ideal);
            }
            if (score < bestScore - 1e-9) { bestScore = score; best = trial.clone(); }
        }
        return best;
    }

    /** Sound power level (dB re 1 pW) of the engine running so (exhaust, intake and mechanics), 0-free-field. */
    public static double soundPowerDb(Spec s, State st) {
        if (!st.running() && !st.cranking()) { return -100; }
        double rpm = Math.max(st.cranking() && !st.running() ? 150 : 300, st.rpm());
        double vd = Math.max(0.01, s.displacementL()), load = st.running() ? Math.clamp(st.load(), 0, 2) : 0.1;
        double exhaust = 80 + 10 * Math.log10(vd) + 30 * Math.log10(rpm / 1000) + 20 * Math.log10(0.25 + load) + s.mufflerDb() + (s.diesel() ? 3 : 0);
        double intake = exhaust - 8 + 10 * Math.log10(0.1 + 0.9 * Math.clamp(st.throttle(), 0, 1)) - s.mufflerDb() * 0.5;
        double mechanics = 72 + 10 * Math.log10(vd) + 40 * Math.log10(rpm / 1000) + (s.diesel() ? 6 : 0);
        double turbo = s.turboBlades() > 0 ? 70 + 10 * Math.log10(vd) + 30 * Math.log10(Math.max(1e-3, st.boostRatio() - 1) + 0.05) : -100;
        double cranking = st.cranking() ? 78 + 5 * Math.log10(vd) : -100;
        return Dsp.sumDb(exhaust, intake, mechanics, turbo, cranking);
    }

    /** The compressor wheel's speed (rev/s) giving a boost: Euler work cp T ((PR)^0.286 - 1) / eta = sigma U^2. */
    public static double turboRevPerS(double boostRatio, double tipDiameterM) {
        if (!(boostRatio > 1) || !(tipDiameterM > 0)) { return 0; }
        double work = 1005 * 293 * (Math.pow(boostRatio, 0.2857) - 1) / 0.7;
        double tip = Math.sqrt(work / 0.9);
        return tip / (Math.PI * tipDiameterM);
    }

    /** The heard turbo whistle at a boost: the wheel's second shaft order, Hz. */
    public static double turboWhistleHz(double boostRatio, double tipDiameterM) { return 2 * turboRevPerS(boostRatio, tipDiameterM); }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double rpmTarget = st.running() ? st.rpm() : st.cranking() ? Math.max(st.rpm(), 160) : st.rpm();
        double follow = Dsp.smoothing(0.02), slow = Dsp.smoothing(0.08);
        double runTarget = st.running() ? 1 : 0, crankTarget = st.cranking() ? 1 : 0;
        int cylinders = spec.cylinders();
        double norm = 0.9 / Math.sqrt(cylinders * 0.25 + 1);
        double[] bankIn = new double[spec.banks()];
        for (int i = 0; i < n; i++) {
            rpm += (rpmTarget - rpm) * follow;
            load += (Math.clamp(st.load(), 0, 2) - load) * slow;
            throttle += (Math.clamp(st.throttle(), 0, 1) - throttle) * follow;
            boost += (Math.max(1, st.boostRatio()) - 1 - boost) * slow;
            run += (runTarget - run) * slow;
            crankingLevel += (crankTarget - crankingLevel) * follow;
            double prev = crank;
            double degPerSample = rpm / 60 * 360 / RATE;
            crank += degPerSample;
            boolean wrapped = crank >= 720;
            if (wrapped) { crank -= 720; }
            // firings
            for (int c = 0; c < cylinders; c++) {
                double a = spec.openingDeg()[c];
                boolean crossed = wrapped ? (a > prev || a <= crank) : (a > prev && a <= crank);
                if (crossed && degPerSample > 0) { fire(c, st); }
            }
            // pulses into the banks
            java.util.Arrays.fill(bankIn, 0);
            double rise = Math.max(1e-4, 25 / Math.max(degPerSample * RATE, 1e-3)), intakeKick = 0;
            for (int c = 0; c < cylinders; c++) {
                double age = pulseAge[c] - pulseDelay[c];
                pulseAge[c] += 1.0 / RATE;
                if (age < 0 || age > 40 * rise) { continue; }
                double x = age / rise, envelope = x * Math.exp(1 - x);
                double p = pulseAmp[c] * envelope * (1 + (0.25 + 0.5 * Math.min(1, load)) * noise.white());
                bankIn[spec.bank()[c]] += p;
                intakeKick += pulseAmp[c] * envelope * (age < rise ? 1 : 0);
            }
            double exhaust = 0;
            for (int b = 0; b < bankIn.length; b++) {
                double back = pipeLoss[b].lowpass(pipes[b].read(pipeDelay));
                double y = bankIn[b] - 0.55 * back;
                pipes[b].write(y);
                exhaust += muffler[b].process(y);
            }
            exhaust = radiation.process(exhaust) * norm;
            // overrun pops in the pipe
            if (popEnv > 1e-4) { exhaust += popEnv * noise.white(); popEnv *= 0.9985; }
            // intake: the resonator, and the air past the throttle
            double air = intake.process(intakeKick * (0.15 + 0.85 * throttle)) * 2.5 * norm
                    + hiss.process(noise.white()) * 0.08 * throttle * Math.min(1, rpm / 6000);
            // valves closing
            valvePhase += rpm / 120 * spec.valvesPerCylinder() * cylinders / RATE;
            double tick = 0;
            if (valvePhase >= 1) { valvePhase -= 1 + 0.35 * noise.white(); tick = 0.02 * (0.7 + 0.6 * noise.uniform()) * Math.min(2, rpm / 3000); }
            double mech = valves.next(tick) + detonation.next(0) + block.process(noise.pink()) * 0.05 * Math.min(1.5, rpm / 4000);
            // diesel combustion knock
            double knock = clatterEnv > 1e-4 ? clatter.process(noise.white()) * clatterEnv : 0;
            clatterEnv *= 0.9930;
            // turbo whistle, blow-off or surge
            double turboOut = 0;
            if (spec.turboBlades() > 0 && boost > 0.02) {
                double rev = turboRevPerS(1 + boost, spec.turboTipM()), bpf = rev * spec.turboBlades();
                double whistle = turbo.sine(Math.min(2 * rev, 0.45 * RATE)) + 0.3 * Math.sin(4 * Math.PI * turbo.phase());
                if (bpf < 16_000) { whistle += 0.4 * Math.sin(2 * Math.PI * turbo.phase() * spec.turboBlades() / 2); }
                turboOut = 0.04 * Math.min(1, boost) * whistle;
            }
            if (bovEnv > 1e-4) {
                turboOut += spec.blowOffValve() ? bovBand.process(noise.white()) * bovEnv * 0.9
                        : bovEnv * 0.6 * Math.sin(2 * Math.PI * 14 * bovEnv) * bovBand.process(noise.white());
                bovEnv *= spec.blowOffValve() ? 0.99988 : 0.99975;
            }
            // starter
            double crankWhine = crankingLevel > 1e-3 ? crankingLevel * 0.12 * starterBody.process(starter.saw(1100 + 400 * Math.sin(2 * Math.PI * crank / 180))) : 0;
            double sample = run * exhaust + run * air + (0.3 + 0.7 * run) * mech + run * knock + turboOut + crankWhine;
            if (!st.running() && crankingLevel > 0.01) { sample += crankingLevel * exhaust * 0.4; }
            out[offset + i] = (float) Dsp.softClip(sample);
        }
        // throttle lift on boost: the valve opens (or the compressor surges)
        if (spec.turboBlades() > 0 && lastThrottle - st.throttle() > 0.35 && st.boostRatio() > 1.15) { bovEnv = Math.min(1, (st.boostRatio() - 1) * 1.2 + 0.3); }
        lastThrottle = st.throttle();
    }

    private void fire(int c, State st) {
        boolean burning = st.running();
        double strength;
        if (burning) {
            double l = Math.clamp(load, 0, 2);
            strength = spec.diesel() ? 0.55 + 0.45 * Math.min(1, l) : 0.25 + 0.75 * Math.min(1.5, l);
            if (spec.diesel()) { clatterEnv = Math.max(clatterEnv, 0.35 + 0.4 * (1 - Math.min(1, l))); }
            if (!spec.diesel() && st.knock() > 0 && noise.chance(st.knock())) { detonation.strike(0.15); }
            // the overrun (fuel cut): only pumping, and sometimes unburnt fuel pops in the pipe
            if (!spec.diesel() && st.throttle() < 0.03 && rpm > 1800 && l < 0.12) {
                strength = 0.12;
                if (noise.chance(0.04)) { popEnv = 0.25 + 0.25 * noise.uniform(); }
            }
        } else {
            strength = 0.1;   // cranking: compression only
        }
        pulseAmp[c] = strength * (1 + 0.06 * noise.gauss());
        pulseAge[c] = 0;
        // about a crank degree of spread in when the gas arrives
        pulseDelay[c] = Math.max(0, (1 + 0.8 * noise.gauss()) / Math.max(1, rpm * 6));
    }
}

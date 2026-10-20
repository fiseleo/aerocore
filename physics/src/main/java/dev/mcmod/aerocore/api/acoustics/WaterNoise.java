package dev.mcmod.aerocore.api.acoustics;

/**
 * A hull in water (0.37): the bow's breaking wave and its spray hiss (broadband, louder and brighter with speed: here
 * the bow spray's speed and crest height from AeroCore's BowSpray, sound power 80 + 40 log10(v / 10) dB for a small
 * boat's 10 m/s), the slap of small waves on the hull at low speed, and the slam of a planing hull falling onto the
 * water: von Karman / Wagner water entry, the pressure peak rho v^2 / (2 tan(deadrise)) acting over a moment, heard as
 * a low thud with a splash, its energy (1/2 m v^2 of the hull's vertical motion) radiating at about 1e-4.
 */
public final class WaterNoise implements Voice {
    public record Spec(double beamM, double deadriseDeg, long seed) {}

    /** Moving at a speed (m/s) through the water, with spray of a speed (m/s) and crest height (m), wetted (0-1). */
    public record State(double speedMps, double sprayMps, double crestM, double wetted) {
        public static final State DRY = new State(0, 0, 0, 0);
    }

    private final Spec spec;
    private volatile State state = State.DRY;
    private final Noise noise;
    private final Filters.Svf hiss = new Filters.Svf(2000, 0.5), wash = new Filters.Svf(400, 0.6);
    private final Modal thud = new Modal(new double[]{75, 140, 230}, new double[]{0.25, 0.15, 0.08}, new double[]{1, 0.5, 0.25});
    private volatile double slamPending;
    private double speed, splashEnv, lap, retune;

    public WaterNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    public void set(State s) { if (s != null) { state = s; } }

    /** A slam now, of a strength 0-1 (from the entry speed and deadrise: wagnerPressurePa against a reference). */
    public void slam(double strength) { slamPending = Math.max(slamPending, Math.clamp(strength, 0, 1)); }

    /**
     * Wagner's peak pressure (Pa) of a wedge of deadrise entering water at v (m/s): 1/2 rho v^2 (pi^2 / (4 tan^2 beta) + 1)
     * (Wagner 1932; Faltinsen, Hydrodynamics of High-Speed Marine Vehicles, eq. 8.16).
     */
    public static double wagnerPressurePa(double v, double deadriseDeg) {
        double t = Math.tan(Math.toRadians(Math.max(3, deadriseDeg)));
        return 0.5 * 1000 * v * v * (Math.PI * Math.PI / (4 * t * t) + 1);
    }

    public static double soundPowerDb(Spec spec, State st) {
        if (st.wetted() <= 0 || st.speedMps() < 0.3) { return -100; }
        double v = Math.max(0.3, st.speedMps());
        return 80 + 40 * Math.log10(v / 10) + 10 * Math.log10(Math.max(0.2, spec.beamM() / 2)) + 4 * Math.min(1, st.crestM());
    }

    /** The sound power (dB) of a slam: a hull of mass kg falling onto the water at v (m/s). */
    public static double slamPowerDb(double massKg, double v) {
        return Dsp.powerLevelDb(1e-4 * 0.5 * massKg * v * v / 0.15);
    }

    @Override public void render(float[] out, int offset, int n) {
        State st = state;
        double follow = Dsp.smoothing(0.2);
        if (slamPending > 0) { thud.strike(1.5 * slamPending); splashEnv = Math.max(splashEnv, slamPending); slamPending = 0; }
        for (int i = 0; i < n; i++) {
            speed += (st.speedMps() - speed) * follow;
            if (--retune <= 0) {
                retune = 128;
                hiss.tune(Math.min(8000, 1200 + 250 * speed), 0.45);
                wash.tune(250 + 15 * speed, 0.5);
            }
            hiss.process(noise.white());
            wash.process(noise.pink());
            double wet = Math.clamp(st.wetted(), 0, 1);
            double moving = Math.min(1, speed / 6);
            // small waves lapping at low speed, a bow wave's wash, the spray's hiss with speed
            lap += (noise.white() * 0.5 - lap) * 0.0003;
            double s = wet * ((1 - moving) * 0.5 * (0.5 + lap) * wash.band + moving * (wash.band * 0.6 + hiss.band * 0.9 * Math.min(1, st.sprayMps() / 8 + 0.2)));
            // a slam: a low thud and a splash
            s += thud.next(0);
            if (splashEnv > 1e-4) { s += hiss.band * splashEnv * 1.2; splashEnv *= 0.99985; }
            out[offset + i] = (float) Dsp.softClip(0.6 * s);
        }
    }
}

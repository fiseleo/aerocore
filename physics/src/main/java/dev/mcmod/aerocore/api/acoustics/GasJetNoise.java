package dev.mcmod.aerocore.api.acoustics;

/**
 * Gas let out of a vessel (0.37): an air brake's exhaust, a blow-off valve, a steam engine's exhaust beat, a pneumatic
 * tool. Above a pressure ratio of 1.89 the orifice chokes; the jet's fully expanded speed is
 * U = sqrt(2 cp T0 (1 - (pa/p0)^((g-1)/g))) (isentropic), and its noise is Lighthill's: acoustic power
 * W = K rho_a U^8 D^2 / c^5 with K about 3e-5 (subsonic round jets), plus shock noise once supersonic (here the same
 * law carried on, as for under-expanded jets measured by Tanna 1977 to within a few dB). Its spectrum peaks at a
 * Strouhal number of 0.2 (f = 0.2 U / D) and is broad (a band-pass of Q about 0.6); small orifices hiss high.
 * A vessel emptying through it falls in pressure exponentially, with the time V / (C_d A c*) for an ideal gas at
 * constant temperature (here C_d 0.8, c* = 0.58 sqrt(R T)), so the hiss falls and dulls as the pressure goes.
 */
public final class GasJetNoise implements Voice {
    public static final double AIR_R = 287.05, AIR_CP = 1005, GAMMA = 1.4, LIGHTHILL_K = 3e-5;

    /** A release: vessel volume (m^3, 0 for a steady supply), starting absolute pressure (Pa), orifice diameter (m), gas temperature (K). */
    public record Spec(double volumeM3, double pressurePa, double orificeM, double temperatureK, long seed) {}

    private final Spec spec;
    private final Noise noise;
    private final Filters.Svf band = new Filters.Svf(1000, 0.6);
    private volatile double startAt = -1;
    private double t, retune;
    private volatile boolean steady;

    public GasJetNoise(Spec spec) {
        this.spec = spec;
        this.noise = new Noise(spec.seed());
    }

    /** The fully expanded jet speed (m/s) from a vessel at p0 into pa. */
    public static double jetSpeed(double p0, double pa, double t0) {
        if (p0 <= pa) { return 0; }
        return Math.sqrt(2 * AIR_CP * t0 * (1 - Math.pow(pa / p0, (GAMMA - 1) / GAMMA)));
    }

    /** Lighthill's acoustic power (W) of a jet of speed u and diameter d into still air. */
    public static double powerW(double u, double d) {
        double rho = 1.2, c = 343;
        return LIGHTHILL_K * rho * Math.pow(u, 8) * d * d / Math.pow(c, 5);
    }

    /** The pressure (Pa) after t seconds of emptying (constant temperature, choked flow; never below ambient). */
    public static double pressureAt(Spec s, double t, double ambientPa) {
        if (s.volumeM3() <= 0) { return s.pressurePa(); }
        double area = Math.PI * s.orificeM() * s.orificeM() / 4;
        double tau = s.volumeM3() / (0.8 * area * 0.58 * Math.sqrt(AIR_R * s.temperatureK()));
        return Math.max(ambientPa, ambientPa + (s.pressurePa() - ambientPa) * Math.exp(-t / tau));
    }

    public static double soundPowerDb(Spec s, double pressurePa, double ambientPa) {
        double u = jetSpeed(pressurePa, ambientPa, s.temperatureK());
        return u < 1 ? -100 : Dsp.powerLevelDb(powerW(u, s.orificeM()));
    }

    /** Starts a release (a vessel emptying) now. */
    public void release() { startAt = 0; steady = false; }
    /** Blows steadily (a supply that does not run down) or stops. */
    public void blow(boolean on) { steady = on; if (on) { startAt = 0; } else if (spec.volumeM3() <= 0) { startAt = -1; } }

    @Override public void render(float[] out, int offset, int n) {
        double ambient = 101_325;
        if (startAt == 0) { t = 0; startAt = 1; }
        for (int i = 0; i < n; i++) {
            if (startAt < 0) { out[offset + i] = 0; continue; }
            double p = steady ? spec.pressurePa() : pressureAt(spec, t, ambient);
            double u = jetSpeed(p, ambient, spec.temperatureK());
            if (--retune <= 0) {
                retune = 64;
                band.tune(Math.min(0.2 * Math.max(u, 1) / Math.max(1e-3, spec.orificeM()), 14_000), 0.6);
            }
            band.process(noise.white());
            double level = Math.min(1, Math.pow(u / 520, 3));   // the voice's own shape: louder and brighter as the jet is faster
            out[offset + i] = (float) Dsp.softClip(0.8 * level * band.band);
            t += 1.0 / RATE;
            if (!steady && u < 5 && spec.volumeM3() > 0) { startAt = -1; }
        }
    }
}

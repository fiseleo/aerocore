package dev.mcmod.aerocore.api.acoustics;

/**
 * Knocks and clunks by modal synthesis (0.37): a struck panel rings at its plate modes and a blow on soft ground only
 * thuds. A plate of thickness h, sides a by b, of a material with longitudinal wave speed c_L rings at
 * f_mn = 0.453 c_L h ((m/a)^2 + (n/b)^2) (Kirchhoff, simply supported; Cremer and Heckl), each mode decaying 60 dB in
 * 2.2 / (f eta), eta the material's loss factor (steel 0.001, damped body panels 0.01-0.03). The contact itself is a
 * short noise burst as long as the Hertzian contact time (soft surfaces longer and duller).
 * <p>
 * Level: a fraction of the impact's energy radiates (about 1e-3 for a ringing panel, less from soft ground):
 * sound energy = eta_rad * 1/2 J v (J the impulse, v the closing speed), heard as a burst of that energy over the
 * ring's decay, so its sound power level is 10 log10(E / T60 / 1 pW).
 */
public final class ImpactNoise {
    private ImpactNoise() {}

    /** What is struck: a panel of the vehicle (thickness, sides, its wave speed and loss) against a surface. */
    public record Spec(double thicknessM, double sideAM, double sideBM, double waveSpeedMps, double lossFactor, Surface against) {
        /** A car's steel body panel: 0.8 mm between its stiffeners (0.3 by 0.25 m), damped. */
        public static Spec carPanel(Surface against) { return new Spec(0.0008, 0.3, 0.25, 5200, 0.015, against); }
        /** A heavy steel part (a hitch, a coupling, a gear): thick and lightly damped. */
        public static Spec steelPart(double sizeM) { return new Spec(Math.max(0.005, sizeM / 10), sizeM, sizeM * 0.7, 5200, 0.002, Surface.STEEL); }
    }

    /** The modes of the struck panel (frequencies, decays, gains), the lowest few with their spread. */
    public static Modal modes(Spec s, long seed) {
        Noise random = new Noise(seed);
        int count = 8;
        double[] f = new double[count], decay = new double[count], gain = new double[count];
        int k = 0;
        for (int m = 1; m <= 4 && k < count; m++) {
            for (int n = 1; n <= 3 && k < count; n++) {
                double fm = 0.453 * s.waveSpeedMps() * s.thicknessM() * (Math.pow(m / s.sideAM(), 2) + Math.pow(n / s.sideBM(), 2));
                f[k] = Math.max(40, Math.min(9000, fm * (1 + 0.03 * random.white())));
                decay[k] = Math.min(4, 2.2 / (f[k] * Math.max(1e-4, s.lossFactor())));
                gain[k] = 1.0 / (m * n);
                k++;
            }
        }
        return new Modal(f, decay, gain);
    }

    /** The radiated sound power level (dB) of an impact: impulse (N s) at a closing speed (m/s). */
    public static double soundPowerDb(Spec s, double impulseNs, double closingMps) {
        double energy = 0.5 * Math.abs(impulseNs) * Math.abs(closingMps);
        double efficiency = s.against().rings() ? 1e-3 : 2e-4;
        double ring = Math.max(0.03, Math.min(1, 2.2 / (300 * Math.max(1e-4, s.lossFactor()))));
        return Dsp.powerLevelDb(efficiency * energy / ring);
    }

    /** The sound of one impact as samples (peak about 0.5): the contact burst and the panel's ring, a strength 0-1. */
    public static float[] render(Spec s, double strength, long seed) {
        Modal modes = modes(s, seed);
        Noise noise = new Noise(seed ^ 0x5bd1e995L);
        double contactS = s.against().rings() ? 0.002 : 0.012;
        double seconds = Math.min(2.5, Math.max(0.15, modes.longestDecayS() * 0.6) + contactS);
        float[] out = new float[(int) (seconds * Voice.RATE)];
        Filters.OnePole dull = new Filters.OnePole(s.against().rings() ? 6000 : 700);
        int burst = (int) (contactS * Voice.RATE);
        double amp = Math.max(0, Math.min(1, strength));
        modes.strike(amp * 0.4);
        double peak = 1e-9;
        for (int i = 0; i < out.length; i++) {
            double contact = i < burst ? dull.lowpass(noise.white()) * Math.sin(Math.PI * i / burst) : dull.lowpass(0);
            double ring = s.against().rings() ? modes.next(0) : modes.next(0) * 0.2;
            double fade = i > out.length - 2205 ? (out.length - i) / 2205.0 : 1;
            out[i] = (float) ((contact * amp + ring) * fade);
            peak = Math.max(peak, Math.abs(out[i]));
        }
        double scale = 0.5 * amp / peak;
        for (int i = 0; i < out.length; i++) { out[i] *= (float) scale; }
        return out;
    }
}

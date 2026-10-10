package dev.mcmod.aerocore.api;

import java.util.Random;

/**
 * Gusts of the wind over the sea: the turbulent part of the wind as a deterministic sum of sinusoids, so that every
 * reader (server physics and client) gets the same gust from the same seed and time.
 * <ul>
 * <li>Spectra: Kaimal et al. (1972, Q. J. R. Meteorol. Soc. 98:563) for the neutral surface layer at 10 m,
 * f S_u / u*^2 = 105 n / (1 + 33 n)^(5/3) along the wind and f S_v / u*^2 = 17 n / (1 + 9.5 n)^(5/3) across it,
 * n = f z / U10, with u* from SeaWind (COARE 3.0); their integrals give sigma_u = 2.2 u* and sigma_v = 1.6 u*.</li>
 * <li>Synthesis: COMPONENTS frequencies, log-spaced between LOW_HZ (slower changes belong to the 10-minute mean that
 * defines the wind) and HIGH_HZ, jittered within their bands, with random phases; amplitudes sqrt(2 S Delta f) at the
 * present mean wind, so a changing mean wind changes the gusts' strength without phase jumps.</li>
 * <li>Space: Taylor's frozen turbulence: the pattern drifts downwind at U10, so the gust depends on t - s / U10 with s
 * the along-wind distance from a reference point. Assumed fully coherent across the wind, and the same at every
 * height near the surface (sigma_u is about constant through the surface layer, Panofsky and Dutton 1984).</li>
 * </ul>
 */
public final class WindGusts {
    public static final int COMPONENTS = 48;
    public static final double LOW_HZ = 1.0 / 600, HIGH_HZ = 1.0, HEIGHT = 10;
    private final double[] frequency, band, phaseU, phaseV;
    private record Amplitudes(double u10, double[] u, double[] v) {}
    private volatile Amplitudes amplitudes = new Amplitudes(-1, null, null);

    public WindGusts(long seed) {
        Random random = new Random(seed);
        frequency = new double[COMPONENTS]; band = new double[COMPONENTS]; phaseU = new double[COMPONENTS]; phaseV = new double[COMPONENTS];
        double ratio = Math.log(HIGH_HZ / LOW_HZ) / COMPONENTS;
        for (int i = 0; i < COMPONENTS; i++) {
            double low = LOW_HZ * Math.exp(i * ratio), high = LOW_HZ * Math.exp((i + 1) * ratio);
            frequency[i] = low + random.nextDouble() * (high - low);
            band[i] = high - low;
            phaseU[i] = 2 * Math.PI * random.nextDouble(); phaseV[i] = 2 * Math.PI * random.nextDouble();
        }
    }

    /** Kaimal spectral densities {S_u, S_v} (m^2/s^2 per Hz) at frequency f for a 10 m wind u10 with friction velocity ustar. */
    public static double[] spectra(double f, double u10, double ustar) {
        double n = f * HEIGHT / u10, scale = ustar * ustar / f;
        return new double[]{scale * 105 * n / Math.pow(1 + 33 * n, 5.0 / 3), scale * 17 * n / Math.pow(1 + 9.5 * n, 5.0 / 3)};
    }

    private Amplitudes amplitudes(double u10) {
        Amplitudes a = amplitudes;
        if (a.u10() == u10) { return a; }
        double ustar = SeaWind.frictionVelocity(u10);
        double[] au = new double[COMPONENTS], av = new double[COMPONENTS];
        for (int i = 0; i < COMPONENTS; i++) {
            double[] s = spectra(frequency[i], u10, ustar);
            au[i] = Math.sqrt(2 * s[0] * band[i]); av[i] = Math.sqrt(2 * s[1] * band[i]);
        }
        a = new Amplitudes(u10, au, av);
        amplitudes = a;
        return a;
    }

    /**
     * Gust velocity along and across the wind (m/s) into out[0], out[1], at time t (s) and along-wind distance s (m)
     * from the reference point, for a 10 m mean wind u10 (m/s); zero in calm air.
     */
    public void sample(double u10, double t, double along, double[] out) {
        out[0] = 0; out[1] = 0;
        if (!(u10 > 0)) { return; }
        Amplitudes a = amplitudes(u10);
        double lag = t - along / u10, u = 0, v = 0;
        for (int i = 0; i < COMPONENTS; i++) {
            double theta = 2 * Math.PI * frequency[i] * lag;
            u += a.u()[i] * Math.cos(theta + phaseU[i]); v += a.v()[i] * Math.cos(theta + phaseV[i]);
        }
        out[0] = u; out[1] = v;
    }

    /** Standard deviations {sigma_u, sigma_v} of the synthesised gusts for a 10 m wind, m/s. */
    public double[] deviations(double u10) {
        if (!(u10 > 0)) { return new double[]{0, 0}; }
        Amplitudes a = amplitudes(u10);
        double su = 0, sv = 0;
        for (int i = 0; i < COMPONENTS; i++) { su += 0.5 * a.u()[i] * a.u()[i]; sv += 0.5 * a.v()[i] * a.v()[i]; }
        return new double[]{Math.sqrt(su), Math.sqrt(sv)};
    }
}

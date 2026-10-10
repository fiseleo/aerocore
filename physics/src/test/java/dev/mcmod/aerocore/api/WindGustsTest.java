package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class WindGustsTest {
    @Test void varianceIsTheKaimalSpectrumsIntegral() {
        WindGusts gusts = new WindGusts(42);
        for (double u : new double[]{5, 10, 18}) {
            double ustar = SeaWind.frictionVelocity(u), su = 0, sv = 0;
            // The spectra integrated over the synthesised band, independently on a fine logarithmic grid.
            int steps = 20000;
            double ratio = Math.log(WindGusts.HIGH_HZ / WindGusts.LOW_HZ) / steps;
            for (int i = 0; i < steps; i++) {
                double f = WindGusts.LOW_HZ * Math.exp((i + 0.5) * ratio), df = f * ratio;
                double[] s = WindGusts.spectra(f, u, ustar);
                su += s[0] * df; sv += s[1] * df;
            }
            double[] d = gusts.deviations(u);
            System.out.printf("U10 %.0f m/s: sigma_u %.3f m/s (band integral %.3f, full Kaimal 2.18 u* = %.3f), sigma_v %.3f (%.3f), I_u %.3f%n",
                    u, d[0], Math.sqrt(su), 2.18 * ustar, d[1], Math.sqrt(sv), d[0] / u);
            assertEquals(Math.sqrt(su), d[0], 0.05 * Math.sqrt(su));
            assertEquals(Math.sqrt(sv), d[1], 0.05 * Math.sqrt(sv));
            assertEquals(2.18 * ustar, d[0], 0.12 * 2.18 * ustar, "Most of the surface-layer variance lies in the band");
        }
    }

    @Test void gustFactorMatchesWieringaOverOpenWater() {
        // Largest 3 s mean in each 10 min over that 10 min's mean, averaged over 60 windows, against Wieringa (1973):
        // G = 1 + (1.42 + 0.3013 ln(990 / (U t) - 4)) / ln(z / z0), z0 from the same COARE roughness.
        WindGusts gusts = new WindGusts(7);
        double u = 10, dt = 0.25, window = 600, sum = 0;
        int windows = 60, per = (int) (window / dt), avg = (int) (3 / dt);
        double[] out = new double[2], series = new double[per];
        for (int w = 0; w < windows; w++) {
            double mean = 0;
            for (int i = 0; i < per; i++) { gusts.sample(u, (w * window) + i * dt, 0, out); series[i] = u + out[0]; mean += series[i]; }
            mean /= per;
            double best = 0, run = 0;
            for (int i = 0; i < per; i++) { run += series[i]; if (i >= avg) { run -= series[i - avg]; } if (i >= avg - 1) { best = Math.max(best, run / avg); } }
            sum += best / mean;
        }
        double g = sum / windows, z0 = SeaWind.roughness(SeaWind.frictionVelocity(u), u);
        double wieringa = 1 + (1.42 + 0.3013 * Math.log(990 / (u * 3) - 4)) / Math.log(10 / z0);
        System.out.printf("Gust factor G(3 s, 10 min) at U10 %.0f m/s: %.3f (Wieringa 1973 over water %.3f)%n", u, g, wieringa);
        assertEquals(wieringa, g, 0.07);
    }

    @Test void sameSeedSameGusts() {
        double[] a = new double[2], b = new double[2];
        new WindGusts(3).sample(12, 12345.6, 40, a);
        new WindGusts(3).sample(12, 12345.6, 40, b);
        assertArrayEquals(a, b, 0);
        // Frozen turbulence: 120 m downwind the same gust arrives 10 s later at 12 m/s.
        WindGusts g = new WindGusts(3);
        g.sample(12, 100, 0, a); g.sample(12, 110, 120, b);
        assertArrayEquals(a, b, 1e-9);
    }
}

package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SecondOrderSeaTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void oneDeepWaterWaveIsStokesSecondOrder() {
        // DNV-RP-C205 (2010) 3.2.3.3: eta = a cos(theta) + (k a^2 / 2) cos(2 theta) in deep water.
        LinearWave wave = new LinearWave(0.6, 20, 0.4, Double.POSITIVE_INFINITY, 0.3);
        CompiledWaves c = new CompiledWaves(new WaveField(List.of(wave)));
        double k = wave.wavenumber(), a = wave.amplitudeM();
        for (int i = 0; i < 50; i++) {
            double x = 3.1 * i, z = -1.7 * i, t = 0.37 * i, theta = wave.phaseAt(x, z, t);
            assertEquals(a * Math.cos(theta) + 0.5 * k * a * a * Math.cos(2 * theta), c.surface(x, z, t), 1e-12);
        }
        assertEquals(0.5 * k * a * a, c.meanLevel(), 1e-15);
    }

    @Test void finiteDepthBoundHarmonicApproachesStokes() {
        // The model's bound harmonic is (k a^2 / 2) coth(kh); Stokes' is (k a^2 / 4) cosh(kh)(2 + cosh 2kh) / sinh^3(kh).
        for (double kh : new double[]{1, 2, 3}) {
            double length = 30, k = 2 * Math.PI / length, depth = kh / k, a = 0.4;
            LinearWave wave = new LinearWave(a, length, 0, depth, 0);
            CompiledWaves c = new CompiledWaves(new WaveField(List.of(wave)));
            double crest = c.surface(0, 0, 0), trough = c.surface(length / 2, 0, 0);
            double bound = (crest + trough) / 2, model = 0.5 * k * a * a / Math.tanh(kh);
            double stokes = k * a * a / 4 * Math.cosh(kh) * (2 + Math.cosh(2 * kh)) / Math.pow(Math.sinh(kh), 3);
            assertEquals(model, bound, 1e-12, "kh " + kh);
            assertEquals(stokes / model, 1 + 1.5 / Math.pow(Math.sinh(kh), 2), 1e-9);
            if (kh >= 3) { assertEquals(stokes, bound, 0.02 * stokes); }
        }
    }

    @Test void slopesAndGridMatchThePointFormula() {
        WaveField sea = SeaSpectrum.synthesize(1.5, 7, 3.3, 0.2, 4, 25, 32, 11);
        CompiledWaves c = new CompiledWaves(sea);
        double[] out = new double[6];
        double h = 1e-5;
        for (int i = 0; i < 20; i++) {
            double x = 2.3 * i, z = 1.1 * i, t = 0.8 * i;
            c.surfaceTerms(x, z, t, out);
            assertEquals((c.surface(x + h, z, t) - c.surface(x - h, z, t)) / (2 * h), out[1] + out[4], 1e-6);
            assertEquals((c.surface(x, z + h, t) - c.surface(x, z - h, t)) / (2 * h), out[2] + out[5], 1e-6);
        }
        int nx = 5, nz = 7;
        double[] h2 = new double[nx * nz], sx = new double[nx * nz], sz = new double[nx * nz];
        c.gridSecondOrder(4, -3, 0.5, 0.25, nx, nz, 2.5, h2, sx, sz);
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            c.surfaceTerms(4 + 0.5 * i, -3 + 0.25 * j, 2.5, out);
            int p = i * nz + j;
            assertEquals(out[3], h2[p], 1e-10); assertEquals(out[4], sx[p], 1e-10); assertEquals(out[5], sz[p], 1e-10);
        }
    }

    @Test void keepsTheStillLevelAndRaisesTheCrests() {
        // JONSWAP Hs 1 m, Tp 6 s in deep water, sampled every 0.1 s at 40 points for 600 s.
        double hs = 1, tp = 6;
        WaveField sea = SeaSpectrum.synthesize(hs, tp, 3.3, 0, 4, Double.POSITIVE_INFINITY, 32, 5);
        CompiledWaves c = new CompiledWaves(sea);
        double[] out = new double[6];
        double sum = 0, count = 0;
        List<Double> linearCrests = new ArrayList<>(), raised = new ArrayList<>();
        for (int p = 0; p < 40; p++) {
            double x = 37.0 * p, z = 23.0 * p, previous = 0, crestLinear = 0, crestTotal = 0;
            for (int i = 0; i < 6000; i++) {
                c.surfaceTerms(x, z, 0.1 * i, out);
                double linear = out[0], total = out[0] + out[3];
                sum += total; count++;
                if (previous <= 0 && linear > 0) {
                    if (crestLinear > 0) { linearCrests.add(crestLinear); raised.add(crestTotal - crestLinear); }
                    crestLinear = 0; crestTotal = 0;
                }
                if (linear > crestLinear) { crestLinear = linear; crestTotal = total; }
                previous = linear;
            }
        }
        double mean = sum / count;
        System.out.printf("Second-order sea: mean level %.4f m, %d crests%n", mean, linearCrests.size());
        assertEquals(0, mean, 0.01, "The mean-level correction keeps the still level");
        // Highest 10 % of crests: raised on average, by less than Forristall's (2000) short-crested second-order fit
        // predicts above Rayleigh at P = 1e-2 (alpha 0.3602, beta 1.9543 for S1 0.0255, Ur 0; DNV-RP-C205 3.5.10).
        Integer[] order = new Integer[linearCrests.size()];
        for (int i = 0; i < order.length; i++) { order[i] = i; }
        Arrays.sort(order, (a, b) -> Double.compare(linearCrests.get(b), linearCrests.get(a)));
        double rise = 0, crest = 0;
        int top = order.length / 10;
        for (int i = 0; i < top; i++) { rise += raised.get(order[i]); crest += linearCrests.get(order[i]); }
        rise /= top; crest /= top;
        double rayleigh = hs * Math.sqrt(Math.log(100) / 8), forristall = 0.3602 * hs * Math.pow(Math.log(100), 1 / 1.9543);
        System.out.printf("Top 10 %% crests: linear %.3f m, raised by %.4f m; Forristall - Rayleigh at 1e-2: %.4f m%n", crest, rise, forristall - rayleigh);
        assertTrue(rise > 0.005, "Crests must be raised: " + rise);
        assertTrue(rise < forristall - rayleigh + 0.01, "Not more than second-order theory: " + rise);
    }
}

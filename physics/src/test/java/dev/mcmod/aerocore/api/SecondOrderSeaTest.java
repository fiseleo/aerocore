package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SecondOrderSeaTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void oneWaveIsStokesSecondOrderInAnyDepth() {
        // DNV-RP-C205 (2010) 3.2.3: bound harmonic (k a^2 / 4) cosh(kh)(2 + cosh 2kh) / sinh^3(kh), (k a^2 / 2) in deep water.
        LinearWave deep = new LinearWave(0.6, 20, 0.4, Double.POSITIVE_INFINITY, 0.3);
        CompiledWaves c = new CompiledWaves(new WaveField(List.of(deep)));
        double k = deep.wavenumber(), a = deep.amplitudeM();
        for (int i = 0; i < 50; i++) {
            double x = 3.1 * i, z = -1.7 * i, t = 0.37 * i, theta = deep.phaseAt(x, z, t);
            assertEquals(a * Math.cos(theta) + 0.5 * k * a * a * Math.cos(2 * theta), c.surface(x, z, t), 1e-12);
        }
        for (double kh : new double[]{1, 2, 3}) {
            double length = 30, kk = 2 * Math.PI / length, depth = kh / kk, amp = 0.4;
            CompiledWaves w = new CompiledWaves(new WaveField(List.of(new LinearWave(amp, length, 0, depth, 0))));
            double bound = (w.surface(0, 0, 0) + w.surface(length / 2, 0, 0)) / 2;
            double stokes = kk * amp * amp / 4 * Math.cosh(kh) * (2 + Math.cosh(2 * kh)) / Math.pow(Math.sinh(kh), 3);
            assertEquals(stokes, bound, 1e-9 * stokes, "kh " + kh);
            // A uniform wave train keeps the still level: the surface averages to zero over a wavelength.
            double mean = 0;
            for (int q = 0; q < 64; q++) { mean += w.surface(length * q / 64, 0, 0) / 64; }
            assertEquals(0, mean, 1e-9);
        }
    }

    @Test void collinearDeepWaterPairsFollowLonguetHiggins() {
        double[] k = SecondOrderCorrection.kernels(0.3, 0, 0.5, 0, Double.POSITIVE_INFINITY);
        assertEquals(0.3 + 0.5, k[0], 1e-12);
        assertEquals(-(0.5 - 0.3), k[1], 1e-12);
    }

    /** Exact Sharma-Dean double sum (Forristall 2000 eq. 11). */
    private static double exact(WaveField sea, double x, double z, double t) {
        List<LinearWave> w = sea.components();
        double sum = 0;
        for (LinearWave a : w) for (LinearWave b : w) {
            double[] kern = SecondOrderCorrection.kernels(a.wavenumber() * Math.cos(a.directionRadians()), a.wavenumber() * Math.sin(a.directionRadians()),
                    b.wavenumber() * Math.cos(b.directionRadians()), b.wavenumber() * Math.sin(b.directionRadians()), a.depthM());
            double pa = a.phaseAt(x, z, t), pb = b.phaseAt(x, z, t);
            sum += a.amplitudeM() * b.amplitudeM() * (kern[0] * Math.cos(pa + pb) + kern[1] * Math.cos(pa - pb));
        }
        return sum / 4;
    }

    @Test void matchesTheExactSecondOrderSumAndItsGrid() {
        for (double depth : new double[]{Double.POSITIVE_INFINITY, 12}) {
            WaveField sea = SeaSpectrum.synthesize(1.5, 7, 3.3, 0.2, 4, depth, 32, 11);
            CompiledWaves c = new CompiledWaves(sea);
            double[] out = new double[6];
            double sumSq = 0, errSq = 0;
            for (int i = 0; i < 200; i++) {
                double x = 2.3 * i, z = 1.1 * i * (i % 3), t = 0.8 * i;
                c.surfaceTerms(x, z, t, out);
                double e = exact(sea, x, z, t);
                sumSq += e * e; errSq += (out[3] - e) * (out[3] - e);
            }
            double relative = Math.sqrt(errSq / sumSq);
            System.out.printf("Second order, depth %s: correction rank %d, rms error %.4f of the exact sum%n", depth, c.correctionRank(), relative);
            assertTrue(relative < 0.02, "error " + relative);
        }
        WaveField sea = SeaSpectrum.synthesize(1.5, 7, 3.3, 0.2, 4, 25, 32, 11);
        CompiledWaves c = new CompiledWaves(sea);
        double[] out = new double[6];
        double h = 1e-5;
        for (int i = 0; i < 20; i++) {
            double x = 2.3 * i, z = 1.1 * i, t = 0.8 * i;
            c.surfaceTerms(x, z, t, out);
            assertEquals((c.surface(x + h, z, t) - c.surface(x - h, z, t)) / (2 * h), out[1] + out[4], 1e-6);
            assertEquals((c.surface(x, z + h, t) - c.surface(x, z - h, t)) / (2 * h), out[2] + out[5], 1e-6);
            int n = c.size();
            double[] cs = new double[n], sn = new double[n];
            c.phases(x, z, t, cs, sn);
            assertEquals(out[3], c.secondOrderHeight(cs, sn, 0), 1e-12);
        }
        int nx = 5, nz = 7;
        double[] h2 = new double[nx * nz], sx = new double[nx * nz], sz = new double[nx * nz];
        c.gridSecondOrder(4, -3, 0.5, 0.25, nx, nz, 2.5, h2, sx, sz);
        c.gridCorrection(4, -3, 0.5, 0.25, nx, nz, 2.5, h2, sx, sz);
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) {
            c.surfaceTerms(4 + 0.5 * i, -3 + 0.25 * j, 2.5, out);
            int p = i * nz + j;
            assertEquals(out[3], h2[p], 1e-10); assertEquals(out[4], sx[p], 1e-10); assertEquals(out[5], sz[p], 1e-10);
        }
    }

    /**
     * Crests of a JONSWAP sea (Hs 1 m, Tp 6 s, gamma 3.3, deep water): the second-order rise over the linear crests at
     * exceedance 1e-2 matches Forristall's (2000) fit to second-order simulations (DNV-RP-C205 3.5.10), long-crested
     * (spreading cos^30) and short-crested (cos^4). The mean level stays at still water.
     */
    @Test void crestsRiseAsForristallsDistribution() {
        double hs = 1, tp = 6;
        for (double spreading : new double[]{30, 4}) {
            WaveField sea = SeaSpectrum.synthesize(hs, tp, 3.3, 0, spreading, Double.POSITIVE_INFINITY, 32, 5);
            CompiledWaves c = new CompiledWaves(sea);
            double m0 = 0, m1 = 0;
            for (LinearWave w : sea.components()) { m0 += 0.5 * w.amplitudeM() * w.amplitudeM(); m1 += 0.5 * w.amplitudeM() * w.amplitudeM() * w.angularFrequency(); }
            double t1 = 2 * Math.PI * m0 / m1, steepness = 2 * Math.PI * hs / (G * t1 * t1);
            boolean longCrested = spreading > 10;
            double alpha = 0.3536 + (longCrested ? 0.2892 : 0.2568) * steepness, beta = 2 - (longCrested ? 2.1597 : 1.7912) * steepness;
            double p = 0.01, forristall = alpha * hs * Math.pow(-Math.log(p), 1 / beta), rayleigh = hs * Math.sqrt(-Math.log(p) / 8);
            double[] out = new double[6];
            double sum = 0, count = 0;
            List<Double> linear = new ArrayList<>(), total = new ArrayList<>();
            for (int point = 0; point < 40; point++) {
                double x = 37.0 * point, z = 23.0 * point, prevLin = 0, prevTot = 0, crestLin = 0, crestTot = 0;
                for (int i = 0; i < 6000; i++) {
                    c.surfaceTerms(x, z, 0.1 * i, out);
                    double lin = out[0], tot = out[0] + out[3];
                    sum += tot; count++;
                    if (prevLin <= 0 && lin > 0) { if (crestLin > 0) { linear.add(crestLin); } crestLin = 0; }
                    if (prevTot <= 0 && tot > 0) { if (crestTot > 0) { total.add(crestTot); } crestTot = 0; }
                    crestLin = Math.max(crestLin, lin); crestTot = Math.max(crestTot, tot);
                    prevLin = lin; prevTot = tot;
                }
            }
            Collections.sort(linear); Collections.sort(total);
            double qLin = linear.get((int) ((1 - p) * linear.size())), qTot = total.get((int) ((1 - p) * total.size()));
            double ratio = qTot / qLin, expected = forristall / rayleigh;
            System.out.printf("cos^%.0f: S1 %.4f, crests at 1e-2: linear %.3f m, second order %.3f m, ratio %.4f; Forristall/Rayleigh %.4f; mean level %.4f m%n",
                    spreading, steepness, qLin, qTot, ratio, expected, sum / count);
            assertEquals(0, sum / count, 0.01, "still level");
            assertEquals(expected, ratio, 0.02, "crest rise");
        }
    }
}

package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class SeaSpectrumTest {
    @Test void jonswapVarianceMatchesTheSignificantHeight() {
        // Hs = 4 sqrt(m0); A_gamma = 1 - 0.287 ln(gamma) is the DNV normalisation, accurate to about 1 %.
        for (double gamma : new double[]{1, 2, 3.3, 5}) {
            double m0 = SeaSpectrum.moment(0, 2, 7, gamma);
            assertEquals(2.0, 4 * Math.sqrt(m0), 0.02 * 2, "gamma=" + gamma);
        }
        // Pierson-Moskowitz (gamma 1): Tz = Tp / 1.408 (DNV-RP-C205 3.5.5.4), Tz = 2 pi sqrt(m0 / m2).
        double tz = 2 * Math.PI * Math.sqrt(SeaSpectrum.moment(0, 1, 6, 1) / SeaSpectrum.moment(2, 1, 6, 1));
        assertEquals(6 / 1.408, tz, 0.02 * 6 / 1.408);
        assertEquals(Math.exp(5.75 - 1.15 * 4), SeaSpectrum.recommendedGamma(1, 4), 1e-12);
        assertEquals(5, SeaSpectrum.recommendedGamma(4, 6));
        assertEquals(1, SeaSpectrum.recommendedGamma(1, 6));
    }
    @Test void spreadingIsNormalisedAndGammaIsExact() {
        for (double n : new double[]{2, 4, 10}) {
            double sum = 0;
            int steps = 20000;
            for (int i = 0; i < steps; i++) { sum += SeaSpectrum.spreading(-Math.PI / 2 + Math.PI * (i + 0.5) / steps, n) * Math.PI / steps; }
            assertEquals(1, sum, 1e-6, "n=" + n);
        }
        assertEquals(Math.log(24), SeaSpectrum.logGamma(5), 1e-12);
        assertEquals(Math.log(Math.sqrt(Math.PI)), SeaSpectrum.logGamma(0.5), 1e-12);
    }
    @Test void synthesisedSeaHasTheRequestedHeightDirectionAndDispersion() {
        double hs = 1.5, tp = 6, mean = 0.4;
        WaveField sea = SeaSpectrum.synthesize(hs, tp, 3.3, mean, 4, Double.POSITIVE_INFINITY, 64, 42);
        // Variance at a point over a long record (many periods of the slowest beat).
        double sum = 0;
        int n = 200000;
        for (int i = 0; i < n; i++) { double eta = sea.elevation(3, -7, i * 0.05); sum += eta * eta; }
        assertEquals(hs, 4 * Math.sqrt(sum / n), 0.06 * hs);
        // Energy-weighted mean direction close to the requested one, with spread consistent with cos^4 (mean |dtheta| ~ 0.38 rad).
        double c = 0, s = 0, energy = 0, spread = 0;
        for (LinearWave w : sea.components()) {
            double e = w.amplitudeM() * w.amplitudeM();
            c += e * Math.cos(w.directionRadians()); s += e * Math.sin(w.directionRadians()); energy += e;
            spread += e * Math.abs(Math.IEEEremainder(w.directionRadians() - mean, 2 * Math.PI));
        }
        assertEquals(mean, Math.atan2(s, c), 0.15);
        assertTrue(spread / energy > 0.15 && spread / energy < 0.7, "Spread " + spread / energy);
        for (LinearWave w : sea.components()) { assertTrue(2 * w.amplitudeM() / w.wavelengthM() <= LinearWave.MAX_STEEPNESS); }
        // Finite-depth dispersion: the wavelength returned for omega satisfies omega^2 = g k tanh(k h).
        double lambda = SeaSpectrum.wavelength(1.2, 5);
        assertEquals(1.2, new LinearWave(0.1, lambda, 0, 5, 0).angularFrequency(), 1e-10);
    }
    @Test void compiledWavesAndTheirGridEqualTheWaveField() {
        WaveField sea = SeaSpectrum.synthesize(1.2, 5, 3.3, -0.7, 4, 12, 48, 9);
        CompiledWaves fast = new CompiledWaves(sea);
        int nx = 17, nz = 23;
        double[] h = new double[nx * nz], sx = new double[nx * nz], sz = new double[nx * nz];
        fast.grid(-3.5, 10.25, 0.25, 0.5, nx, nz, 7.3, h, sx, sz);
        for (int i = 0; i < nx; i++) for (int k = 0; k < nz; k++) {
            double x = -3.5 + i * 0.25, z = 10.25 + k * 0.5;
            Vec3d slope = sea.slope(x, z, 7.3);
            assertEquals(sea.elevation(x, z, 7.3), h[i * nz + k], 1e-10);
            assertEquals(slope.x(), sx[i * nz + k], 1e-10);
            assertEquals(slope.z(), sz[i * nz + k], 1e-10);
            double[] point = fast.elevationAndSlope(x, z, 7.3);
            assertEquals(h[i * nz + k], point[0], 1e-10);
            assertEquals(sea.elevation(x, z, 7.3), fast.elevation(x, z, 7.3), 1e-12);
        }
        // Column kinematics equal the per-wave methods in finite and deep water, above and below still level.
        for (WaveField field : new WaveField[]{sea, SeaSpectrum.synthesize(0.8, 4, 2, 1.1, 10, Double.POSITIVE_INFINITY, 24, 3)}) {
            CompiledWaves c = new CompiledWaves(field);
            double[] cos = new double[c.size()], sin = new double[c.size()], out = new double[6];
            for (double y : new double[]{0.4, -0.3, -2.7, -11, -20}) {
                Vec3d point = new Vec3d(2.3, 60 + y, -4.1);
                c.phases(point.x(), point.z(), 3.9, cos, sin);
                c.kinematics(cos, sin, y, out);
                Vec3d u = field.velocity(point, 60, 3.9), a = field.acceleration(point, 60, 3.9);
                assertEquals(u.x(), out[0], 1e-10); assertEquals(u.y(), out[1], 1e-10); assertEquals(u.z(), out[2], 1e-10);
                assertEquals(a.x(), out[3], 1e-10); assertEquals(a.y(), out[4], 1e-10); assertEquals(a.z(), out[5], 1e-10);
                assertEquals(field.dynamicPressureHead(point, 60, 3.9), c.pressureHead(cos, y), 1e-10);
                assertEquals(field.elevation(point.x(), point.z(), 3.9), c.elevation(cos), 1e-12);
                assertEquals(u.x(), c.velocity(point, 60, 3.9).x(), 1e-12);
            }
            assertEquals(field.maxAmplitude(), c.maxAmplitude(), 1e-12);
            // Grid phases and one-pass segments equal the point queries, including segments across still water and the bed.
            int n = c.size();
            double[] gc = new double[6 * n], gs = new double[6 * n], seg = new double[8];
            c.gridPhases(1.1, -2.5, 0.3, 0.45, 2, 3, 5.2, gc, gs);
            for (int i = 0; i < 2; i++) for (int j = 0; j < 3; j++) {
                double x = 1.1 + 0.3 * i, z = -2.5 + 0.45 * j;
                c.phases(x, z, 5.2, cos, sin);
                int offset = (i * 3 + j) * n;
                assertEquals(c.elevation(cos), c.elevation(gc, offset), 1e-12);
                for (double[] ends : new double[][]{{-1.3, -0.4}, {-0.6, 0.35}, {-0.2, -0.1}, {-13, -11.5}, {-25, -0.5}}) {
                    c.segment(gc, gs, offset, ends[0], ends[1], seg);
                    Vec3d mid = new Vec3d(x, 60 + (ends[0] + ends[1]) / 2, z);
                    Vec3d u = field.velocity(mid, 60, 5.2), a = field.acceleration(mid, 60, 5.2);
                    assertEquals(field.dynamicPressureHead(new Vec3d(x, 60 + ends[0], z), 60, 5.2), seg[0], 1e-10);
                    assertEquals(field.dynamicPressureHead(new Vec3d(x, 60 + ends[1], z), 60, 5.2), seg[1], 1e-10);
                    assertEquals(u.x(), seg[2], 1e-10); assertEquals(u.y(), seg[3], 1e-10); assertEquals(u.z(), seg[4], 1e-10);
                    assertEquals(a.x(), seg[5], 1e-10); assertEquals(a.y(), seg[6], 1e-10); assertEquals(a.z(), seg[7], 1e-10);
                }
            }
        }
    }
    @Test void rejectsBreakingSeas() {
        assertThrows(IllegalArgumentException.class, () -> SeaSpectrum.synthesize(4, 4, 3.3, 0, 4, Double.POSITIVE_INFINITY, 32, 1));
        assertThrows(IllegalArgumentException.class, () -> SeaSpectrum.synthesize(1, 6, 3.3, 0, 4, 2, 32, 1));
    }
}

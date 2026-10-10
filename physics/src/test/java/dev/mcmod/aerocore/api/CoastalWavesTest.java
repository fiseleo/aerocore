package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CoastalWavesTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    private static float[] grid(int nx, int nz, java.util.function.BiFunction<Integer, Integer, Double> depth) {
        float[] d = new float[nx * nz];
        for (int i = 0; i < nx; i++) for (int j = 0; j < nz; j++) { d[i * nz + j] = (float) (double) depth.apply(i, j); }
        return d;
    }

    /** {gain, d delta/dx, d delta/dz} of component 0 at (x, z). */
    private static double[] envelope(CoastalWaves c, double x, double z) {
        double[] re = new double[c.components()], im = new double[re.length], rx = new double[re.length], ix = new double[re.length],
                rz = new double[re.length], iz = new double[re.length];
        c.envelopes(x, z, re, im, rx, ix, rz, iz);
        double g2 = re[0] * re[0] + im[0] * im[0];
        return new double[]{Math.sqrt(g2), (re[0] * ix[0] - im[0] * rx[0]) / g2, (re[0] * iz[0] - im[0] * rz[0]) / g2};
    }

    @Test void uniformOpenWaterIsLeftAlone() {
        WaveField sea = SeaSpectrum.synthesize(1, 6, 3.3, 0.4, 4, 20, 16, 1);
        CoastalWaves c = CoastalWaves.build(0, 0, 2, 51, 51, grid(51, 51, (i, j) -> 20.0), sea);
        assertTrue(c.uniform());
    }

    @Test void planeBeachFollowsSnellAndShoaling() {
        // Depth 20 m at x = 0 falling 0.1 per metre; an 8 s wave arriving at 30 deg to the beach normal (+x).
        double theta0 = Math.toRadians(30), h0 = 20, period = 8, omega = 2 * Math.PI / period;
        double k0 = CoastalWaves.wavenumber(omega, h0);
        LinearWave wave = new LinearWave(0.05, 2 * Math.PI / k0, theta0, h0, 0);
        assertEquals(omega, wave.angularFrequency(), 1e-9);
        int nx = 96, nz = 151;
        CoastalWaves c = CoastalWaves.build(0, 0, 2, nx, nz, grid(nx, nz, (i, j) -> Math.max(0, h0 - 0.1 * 2 * i)), new WaveField(List.of(wave)));
        for (double x : new double[]{60, 100, 140}) {
            double z = 200, h = h0 - 0.1 * x, k = CoastalWaves.wavenumber(omega, h);
            double[] e = envelope(c, x, z), g = {e[0]};
            double kx = k0 * Math.cos(theta0) + e[1], kz = k0 * Math.sin(theta0) + e[2];
            double theta = Math.atan2(kz, kx);
            // Snell: k sin(theta) is conserved along a straight beach.
            double expectedTheta = Math.asin(k0 * Math.sin(theta0) / k);
            double cg0 = CoastalWaves.groupVelocity(omega, k0, h0), cg = CoastalWaves.groupVelocity(omega, k, h);
            double expectedGain = Math.sqrt(cg0 / cg) * Math.sqrt(Math.cos(theta0) / Math.cos(expectedTheta));
            System.out.printf("Beach h %.1f m: direction %.2f deg (Snell %.2f), |k| %.4f (%.4f), gain %.3f (shoaling x refraction %.3f)%n",
                    h, Math.toDegrees(theta), Math.toDegrees(expectedTheta), Math.hypot(kx, kz), k, g[0], expectedGain);
            assertEquals(expectedTheta, theta, Math.toRadians(1.5));
            assertEquals(k, Math.hypot(kx, kz), 0.03 * k);
            assertEquals(expectedGain, g[0], 0.05 * expectedGain);
        }
    }

    @Test void anIslandCastsAShadow() {
        // 20 m deep, a 20 x 20 m island in the middle, a 6 s wave from -x.
        double omega = 2 * Math.PI / 6, k0 = CoastalWaves.wavenumber(omega, 20);
        LinearWave wave = new LinearWave(0.05, 2 * Math.PI / k0, 0, 20, 0);
        int n = 101;
        CoastalWaves c = CoastalWaves.build(0, 0, 2, n, n, grid(n, n, (i, j) -> Math.abs(i - 50) <= 5 && Math.abs(j - 50) <= 5 ? 0.0 : 20.0), new WaveField(List.of(wave)));
        double lee = envelope(c, 130, 100)[0], side = envelope(c, 130, 170)[0], front = envelope(c, 60, 100)[0];
        StringBuilder profile = new StringBuilder("Island lee profile:");
        for (int x = 112; x <= 200; x += 8) { profile.append(String.format(" %d:%.2f", x, envelope(c, x, 100)[0])); }
        System.out.println(profile);
        System.out.printf("Island: gain in front %.3f, 20 m in its lee %.3f, beside %.3f%n", front, lee, side);
        assertFalse(c.uniform());
        assertEquals(1, front, 0.02);
        assertEquals(1, side, 0.05);
        assertTrue(lee < 0.7, "The lee must be sheltered: " + lee);
    }

    @Test void breakingSaturatesTheSurfZone() {
        double omega = 2 * Math.PI / 8, h0 = 15, k0 = CoastalWaves.wavenumber(omega, h0);
        WaveField sea = new WaveField(List.of(new LinearWave(0.8, 2 * Math.PI / k0, 0, h0, 0)));
        double hs0 = 4 * Math.sqrt(0.5 * 0.8 * 0.8);
        int nx = 80, nz = 21;
        CoastalWaves c = CoastalWaves.build(0, 0, 2, nx, nz, grid(nx, nz, (i, j) -> Math.max(0, h0 - 0.1 * 2 * i)), sea);
        for (double x : new double[]{120, 135, 145}) {
            double h = h0 - 0.1 * x;
            double hs = envelope(c, x, 20)[0] * hs0;
            System.out.printf("Surf: h %.1f m, Hs %.2f m (cap %.2f), breaking %.2f%n", h, hs, CoastalWaves.SATURATED_HS_PER_DEPTH * h, c.breaking(x, 20));
            assertTrue(hs <= CoastalWaves.SATURATED_HS_PER_DEPTH * h * 1.05 + 0.02, "Hs " + hs + " at depth " + h);
        }
        assertTrue(c.breaking(135, 20) > 0.05, "Waves break in the surf zone");
        assertEquals(0, c.breaking(20, 20), 1e-6);
    }

    @Test void anEnclosedPoolKeepsItsWavesAndItsIslandShadow() {
        // A 4 m deep, 44 x 28 m pool walled by land on every side (no open edge), with a 6 x 6 m island; 4 s waves from -x.
        double omega = 2 * Math.PI / 4, k0 = CoastalWaves.wavenumber(omega, 4);
        LinearWave wave = new LinearWave(0.1, 2 * Math.PI / k0, 0, 4, 0);
        int nx = 26, nz = 18;
        CoastalWaves c = CoastalWaves.build(0, 0, 2, nx, nz, grid(nx, nz, (i, j) -> {
            boolean wall = i < 2 || j < 2 || i >= nx - 2 || j >= nz - 2, island = Math.abs(i - 12) <= 1 && Math.abs(j - 8) <= 1;
            return wall || island ? 0.0 : 4.0;
        }), new WaveField(List.of(wave)));
        double front = envelope(c, 14, 16)[0], lee = envelope(c, 34, 16)[0], side = envelope(c, 34, 6)[0];
        System.out.printf("Enclosed pool: gain upwind of the island %.3f, 10 m in its lee %.3f, beside %.3f%n", front, lee, side);
        assertEquals(1, front, 0.05, "Open water of the pool keeps the sea");
        assertEquals(1, side, 0.1);
        assertTrue(lee < 0.75, "The island shades its lee: " + lee);
    }

    @Test void surveyRoundingAndFarCoordinatesDoNotCastShadows() {
        // The GameTest pool surveyed in decimetres (3.9 m) under a sea measured at 3.888 m, near the origin and 1.27e7 m out.
        for (double[] setup : new double[][]{{0, 0, 3.888}, {12729667, -7610527, 3.888}, {0, 0, 3.9}, {12729667, -7610527, 3.9}}) {
            double ox = setup[0], oz = setup[1], water = setup[2];
            WaveField sea = new WaveField(List.of(new LinearWave(0.12, 8, 0, 3.888, 0)));
            int nx = 14, nz = 13;
            float[] d = grid(nx, nz, (i, j) -> {
                int x = -1 + 2 * i, z = -5 + 2 * j;
                boolean wet = x >= 1 && x <= 22 && z >= 1 && z <= 14 && !(x >= 7 && x <= 9 && z >= 6 && z <= 9);
                return wet ? water : 0.0;
            });
            CoastalWaves c = CoastalWaves.build(ox - 1, oz - 5, 2, nx, nz, d, sea);
            StringBuilder b = new StringBuilder();
            double[] gain = new double[nx];
            for (int i = 0; i < nx; i++) {
                double[] re = new double[1], im = new double[1];
                c.envelopes(ox - 1 + 2 * i, oz + 8, re, im, null, null, null, null);
                gain[i] = Math.hypot(re[0], im[0]);
                b.append(String.format(" %.2f", gain[i]));
            }
            System.out.println("Pool row origin " + ox + " depth " + water + ":" + b);
            for (int i = 1; i <= 3; i++) { assertEquals(1, gain[i], 0.02, "Lit water upwind of the island"); }
            assertTrue(gain[6] < 0.75, "The island's lee");
        }
    }

    @Test void aWindSeaGrowsWithFetchBehindLandInsteadOfStayingInShadow() {
        // A 12 m/s wind toward +x over a 200 x 100 m, 4 m deep lake (land all round) with a 10 x 10 m island; the sea is
        // the one this wind raises over 100 m of fetch.
        double u = 12, design = 100;
        double[] hsTp = SeaWind.sea(u, design, Double.POSITIVE_INFINITY);
        WaveField sea = SeaSpectrum.synthesize(hsTp[0], hsTp[1], SeaSpectrum.recommendedGamma(hsTp[0], hsTp[1]), 0, 4, 4, 16, 5);
        int nx = 104, nz = 54;
        float[] d = grid(nx, nz, (i, j) -> {
            boolean lake = i >= 2 && i < nx - 2 && j >= 2 && j < nz - 2, island = Math.abs(i - 40) <= 2 && Math.abs(j - 27) <= 2;
            return lake && !island ? 4.0 : 0.0;
        });
        CoastalWaves c = CoastalWaves.build(0, 0, 2, nx, nz, d, sea, new CoastalWaves.WindSea(u, 0, 0, hsTp[1]));
        // The gain is relative to the Hs of the components as synthesised (a few percent off the target with 16 bins).
        double hsSea = 4 * Math.sqrt(sea.components().stream().mapToDouble(w -> 0.5 * w.amplitudeM() * w.amplitudeM()).sum());
        double[] lee = new double[4];
        StringBuilder row = new StringBuilder();
        for (double x : new double[]{8, 40, 100, 160, 200}) { row.append(String.format(" x%.0f:%.2f", x, envelope(c, x, 30)[0])); }
        for (int k = 0; k < 4; k++) { lee[k] = envelope(c, 86 + 20 * k, 54)[0]; }
        System.out.printf("Wind sea on a lake (Hs at 100 m fetch %.3f m, Tp %.2f s): gain along z = 30:%s; behind the island %.2f %.2f %.2f %.2f%n",
                hsTp[0], hsTp[1], row, lee[0], lee[1], lee[2], lee[3]);
        // The gain is the fetch growth Hs(fetch) / Hs(100 m) (JONSWAP's square root of fetch) until the sea reaches
        // the steepness limit at its own peak period (its frequencies stay as synthesised).
        double steepest = Math.max(hsSea, SeaWind.MAX_STEEPNESS * Atmosphere.GRAVITY_MPS2 * hsTp[1] * hsTp[1] / (2 * Math.PI));
        for (double x : new double[]{40, 100, 160}) {
            double expected = Math.min(SeaWind.sea(u, x - 2, Double.POSITIVE_INFINITY)[0], steepest) / hsSea;
            // Within 10 %: the fetch is measured on the 2 m grid and the growth averaged over 3 x 3 nodes.
            assertEquals(expected, envelope(c, x, 30)[0], 0.10 * expected, "Growth with fetch at x = " + x);
        }
        assertTrue(envelope(c, 8, 30)[0] < 0.4, "Small waves at the upwind shore");
        // Behind the island the sea starts again from the island's shore and regrows downwind.
        assertTrue(lee[0] < lee[1] && lee[1] < lee[2] && lee[2] < lee[3], "Regrowth behind the island");
        assertTrue(lee[0] < 0.8 * envelope(c, 86, 30)[0], "Sheltered close behind the island");
        // With the upwind shore open to the grid's edge, the open water beyond the grid adds its fetch.
        float[] bay = grid(nx, nz, (i, j) -> j >= 2 && j < nz - 2 && i < nx - 2 ? 4.0 : 0.0);
        CoastalWaves open = CoastalWaves.build(0, 0, 2, nx, nz, bay, sea, new CoastalWaves.WindSea(u, 0, 1000, hsTp[1]));
        double far = SeaWind.sea(u, 1000 + 10, Double.POSITIVE_INFINITY)[0] / hsSea, limited = Math.min(far, steepest / hsSea);
        System.out.printf("Bay open to 1 km of sea upwind: gain at x = 8 m %.2f (fetch growth %.2f, steepness limit %.2f)%n",
                envelope(open, 8, 30)[0], far, steepest / hsSea);
        assertEquals(limited, envelope(open, 8, 30)[0], 0.05 * limited, "Fetch beyond the grid counts, up to the steepness limit");
        // Nowhere steeper than DNV's limit at the spectrum's peak period (a 0.14 lake grew 3 times past it and foamed over).
        for (double x = 4; x < 200; x += 6) for (double z = 6; z < 100; z += 6) {
            assertTrue(envelope(open, x, z)[0] * hsSea <= 1.05 * steepest, "Within the steepness limit at " + x + ", " + z);
        }
    }

    @Test void largestRegionBuildsInAboutASecond() {
        // A 128 m region: a 129 x 129 survey of a sloping, island-studded bed under a 32-component sea (reported, not asserted).
        WaveField sea = SeaSpectrum.synthesize(1.2, 6, 3.3, 0.3, 4, 15, 32, 3);
        float[] bed = grid(129, 129, (i, j) -> Math.max(0, 15 - 0.08 * i + 3 * Math.sin(j / 9.0) - ((i - 70) * (i - 70) + (j - 60) * (j - 60) < 60 ? 20 : 0)));
        CoastalWaves.build(0, 0, 2, 129, 129, bed, sea);
        long start = System.nanoTime();
        CoastalWaves c = CoastalWaves.build(0, 0, 2, 129, 129, bed, sea);
        System.out.printf("Coastal build, 129 x 129 nodes x 32 components: %.0f ms, largest gain %.2f%n", (System.nanoTime() - start) / 1e6, c.maxGain());
        assertFalse(c.uniform());
    }

    @Test void crestsSharpenAsTheWaterShoals() {
        // One 8 s wave over the 1:10 beach: crest height over trough depth grows toward the shore (local kernels).
        double omega = 2 * Math.PI / 8, k0 = CoastalWaves.wavenumber(omega, 20);
        WaveField sea = new WaveField(List.of(new LinearWave(0.3, 2 * Math.PI / k0, 0, 20, 0)));
        CoastalWaves c = beach(sea, 96, 41);
        CompiledWaves waves = new CompiledWaves(sea);
        double[] terms = new double[6], ratio = new double[3], depths = {14, 6, 3};
        for (int q = 0; q < 3; q++) {
            double x = 10 * (20 - depths[q]), crest = -1e9, trough = 1e9;
            for (double t = 0; t < 8; t += 0.02) {
                waves.surfaceTerms(x, 40, t, c, terms);
                crest = Math.max(crest, terms[0] + terms[3]); trough = Math.min(trough, terms[0] + terms[3]);
            }
            ratio[q] = crest / -trough;
        }
        System.out.printf("Crest over trough on the beach at 14 / 6 / 3 m: %.3f / %.3f / %.3f%n", ratio[0], ratio[1], ratio[2]);
        assertTrue(ratio[0] > 1 && ratio[1] > ratio[0] && ratio[2] > ratio[1], "Crests sharpen toward the shore");
    }

    /** A short-crested sea on the plane beach of planeBeachFollowsSnellAndShoaling. */
    private static CoastalWaves beach(WaveField sea, int nx, int nz) {
        return CoastalWaves.build(0, 0, 2, nx, nz, grid(nx, nz, (i, j) -> Math.max(0, 20 - 0.1 * 2 * i)), sea);
    }

    @Test void coastalSurfaceGridAndLocalViewAgree() {
        WaveField sea = SeaSpectrum.synthesize(0.6, 5, 3.3, Math.toRadians(20), 4, 20, 12, 7);
        CoastalWaves coast = beach(sea, 80, 60);
        CompiledWaves waves = new CompiledWaves(sea);
        double t = 37.3;
        // The renderer's grid (gridCoastal + gridCorrection) equals the point evaluation at its points.
        int n = 9;
        double x0 = 101, z0 = 47, step = 0.75;
        double[] h = new double[n * n], hx = new double[n * n], hz = new double[n * n], h2 = new double[n * n], h2x = new double[n * n], h2z = new double[n * n];
        waves.gridCoastal(coast, x0, z0, step, step, n, n, t, h, hx, hz, h2, h2x, h2z);
        waves.gridCorrection(coast, x0, z0, step, step, n, n, t, h2, h2x, h2z);
        double[] terms = new double[6], worst = new double[6];
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            int p = i * n + j;
            waves.surfaceTerms(x0 + i * step, z0 + j * step, t, coast, terms);
            double[] grid = {h[p], hx[p], hz[p], h2[p], h2x[p], h2z[p]};
            for (int q = 0; q < 6; q++) { worst[q] = Math.max(worst[q], Math.abs(grid[q] - terms[q])); }
        }
        System.out.printf("Coastal grid vs points: worst differences %s%n", java.util.Arrays.toString(worst));
        for (double w : worst) { assertEquals(0, w, 1e-9); }
        // The linear slope is the derivative of the coastal surface (inside one envelope cell).
        double x = 121.3, z = 51.1, d = 1e-4;
        waves.surfaceTerms(x, z, t, coast, terms);
        double[] east = new double[6], west = new double[6], north = new double[6], south = new double[6];
        waves.surfaceTerms(x + d, z, t, coast, east); waves.surfaceTerms(x - d, z, t, coast, west);
        waves.surfaceTerms(x, z + d, t, coast, north); waves.surfaceTerms(x, z - d, t, coast, south);
        assertEquals((east[0] - west[0]) / (2 * d), terms[1], 1e-6);
        assertEquals((north[0] - south[0]) / (2 * d), terms[2], 1e-6);
        // A body's local view reproduces the coastal surface at its point and approximates it a few metres away.
        double rms = 0, err = 0, slope = 0, slopeErr = 0;
        int samples = 0;
        for (double bx = 96; bx <= 140; bx += 11) for (double tt = 0; tt < 20; tt += 2.5) {
            CompiledWaves view = waves.at(coast, bx, 40, 1);
            double[] local = new double[6];
            view.surfaceTerms(bx, 40, tt, local);
            waves.surfaceTerms(bx, 40, tt, coast, terms);
            assertEquals(terms[0] + terms[3], local[0] + local[3], 1e-9, "Same surface height at the view's point");
            slope += terms[1] * terms[1] + terms[2] * terms[2];
            slopeErr += Math.pow(terms[1] - local[1], 2) + Math.pow(terms[2] - local[2], 2);
            view.surfaceTerms(bx + 2, 41.5, tt, local);
            waves.surfaceTerms(bx + 2, 41.5, tt, coast, terms);
            rms += Math.pow(terms[0], 2);
            err += Math.pow(local[0] - terms[0], 2);
            samples++;
        }
        double relative = Math.sqrt(err / rms), slopeRelative = Math.sqrt(slopeErr / slope);
        System.out.printf("Local view: slope rms error %.3f at the point, height rms error %.3f at 2.5 m (%d samples)%n", slopeRelative, relative, samples);
        assertTrue(slopeRelative < 0.1, "Slope at the point " + slopeRelative);
        assertTrue(relative < 0.1, "Height 2.5 m away " + relative);
        // Frequencies are unchanged, so the view does not drift from the surface over time.
        for (LinearWave w : waves.at(coast, 120, 40, 1).field().components()) {
            boolean found = sea.components().stream().anyMatch(o -> Math.abs(o.angularFrequency() - w.angularFrequency()) < 1e-9 * o.angularFrequency());
            assertTrue(found, "Local wave keeps its frequency");
        }
    }
}

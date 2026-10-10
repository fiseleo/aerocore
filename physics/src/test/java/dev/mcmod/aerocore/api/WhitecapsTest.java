package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class WhitecapsTest {
    @Test void publishedRelations() {
        // Pierson-Moskowitz: Hs 1 m is a 6.4-6.9 m/s wind at 10 m; Monahan & O'Muircheartaigh: about 1 % whitecaps at 10 m/s.
        double u = Whitecaps.equivalentWind(1);
        assertTrue(u > 6.4 && u < 6.9, "Wind " + u);
        assertEquals(3.84e-6 * Math.pow(10, 3.41), Whitecaps.coverage(10), 1e-12);
        assertTrue(Whitecaps.coverage(10) > 0.008 && Whitecaps.coverage(10) < 0.012);
        assertEquals(Whitecaps.coverage(Whitecaps.MAX_WIND), Whitecaps.coverage(30), 0);
        assertEquals(1.959964, Whitecaps.inverseNormal(0.975), 1e-6);
        assertEquals(-2.326348, Whitecaps.inverseNormal(0.01), 1e-6);
        assertEquals(3.719016, Whitecaps.inverseNormal(1 - 1e-4), 1e-5);
        assertEquals(0.5, Whitecaps.foam(1, false, Whitecaps.FOAM_DECAY_S * Math.log(2)), 1e-12);
        // Next to calm (a region's tapered rim, deep shelter): nothing breaks, rather than an out-of-range probability.
        for (double calm : new double[]{0, 1e-9, 1e-5, 0.01}) { assertEquals(Double.POSITIVE_INFINITY, Whitecaps.thresholdAtWind(1, calm), "Wind " + calm); }
        assertTrue(Double.isFinite(Whitecaps.thresholdAtWind(1, 0.5)));
    }

    @Test void breakingAreaMatchesTheActiveWhitecapFraction() {
        // A 3 m, 9 s sea: the surface beyond the threshold covers about W_A = W(U10) / 10.
        double hs = 3;
        CompiledWaves sea = new CompiledWaves(SeaSpectrum.synthesize(hs, 9, 3.3, 0, 4, Double.POSITIVE_INFINITY, 32, 3));
        double threshold = Whitecaps.threshold(sea.accelerationDeviation(), hs);
        double expected = 0.1 * Whitecaps.coverage(Whitecaps.equivalentWind(hs));
        assertTrue(threshold < Whitecaps.LIMIT * Atmosphere.GRAVITY_MPS2);
        int n = sea.size(), hits = 0, samples = 0;
        double[] cos = new double[n], sin = new double[n];
        for (int i = 0; i < 400; i++) for (int j = 0; j < 400; j++) {
            sea.phases(7.3 * i, 5.1 * j, 0.9 * (i + j), cos, sin);
            if (sea.verticalAcceleration(cos, 0) < -threshold) { hits++; }
            samples++;
        }
        double fraction = (double) hits / samples;
        System.out.printf("Whitecaps: Hs %.1f m, U10 %.1f m/s, W %.4f, threshold %.3f g, breaking fraction %.5f (W_A %.5f)%n", hs,
                Whitecaps.equivalentWind(hs), Whitecaps.coverage(Whitecaps.equivalentWind(hs)), threshold / Atmosphere.GRAVITY_MPS2, fraction, expected);
        assertEquals(expected, fraction, 0.35 * expected);
    }

    @Test void aShortFetchSeaFoamsAsMuchAsItsWindSays() {
        // A pond's wind sea at the steepness limit (10 m/s over 87 m, 2.9 m deep): its linear accelerations, carried by
        // centimetre waves, deviate by about a quarter of g, so the calibrated threshold lies past 0.388 g; capped there
        // (until 0.14) the pond foamed over at tens of times Monahan's coverage.
        WaveField pond = SeaSpectrum.synthesize(0.036354523816339336, 0.5910911491441727, 5, 0.349, 4, 2.8888893, 32, 7);
        CompiledWaves sea = new CompiledWaves(pond);
        double threshold = Whitecaps.thresholdAtWind(sea.accelerationDeviation(), 10), expected = 0.1 * Whitecaps.coverage(10);
        int n = sea.size(), hits = 0, samples = 0;
        double[] cos = new double[n], sin = new double[n];
        for (int i = 0; i < 400; i++) for (int j = 0; j < 400; j++) {
            sea.phases(0.37 * i, 0.29 * j, 0.11 * (i + j), cos, sin);
            if (sea.verticalAcceleration(cos, 0) < -threshold) { hits++; }
            samples++;
        }
        double fraction = (double) hits / samples, capped = 0;
        for (int i = 0; i < 400; i++) for (int j = 0; j < 400; j++) {
            sea.phases(0.37 * i, 0.29 * j, 0.11 * (i + j), cos, sin);
            if (sea.verticalAcceleration(cos, 0) < -Whitecaps.LIMIT * Atmosphere.GRAVITY_MPS2) { capped++; }
        }
        System.out.printf("Pond: acceleration deviation %.2f g, threshold %.2f g, breaking %.5f (W_A %.5f; at 0.388 g it would be %.4f)%n",
                sea.accelerationDeviation() / Atmosphere.GRAVITY_MPS2, threshold / Atmosphere.GRAVITY_MPS2, fraction, expected, capped / samples);
        assertTrue(threshold > Whitecaps.LIMIT * Atmosphere.GRAVITY_MPS2, "Calibrated past the smooth-crest limit");
        assertEquals(expected, fraction, 0.5 * expected);
        assertTrue(capped / samples > 10 * expected, "The old cap foamed far too much");
    }
}

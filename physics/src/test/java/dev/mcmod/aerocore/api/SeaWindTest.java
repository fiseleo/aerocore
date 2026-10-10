package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SeaWindTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void dragCoefficientMatchesLargeAndPond() {
        // Large and Pond (1981, J. Phys. Oceanogr. 11:324), neutral: 1.2e-3 for 4-11 m/s, (0.49 + 0.065 U) 1e-3 for 11-25 m/s.
        for (double u : new double[]{5, 8, 10, 12, 15, 20}) {
            double cd = SeaWind.dragCoefficient(u), reference = (u < 11 ? 1.2 : 0.49 + 0.065 * u) * 1e-3;
            System.out.printf("U10 %.0f m/s: Cd10 %.3e (Large and Pond %.3e), u* %.3f m/s, z0 %.2e m%n", u, cd, reference,
                    SeaWind.frictionVelocity(u), SeaWind.roughness(SeaWind.frictionVelocity(u), u));
            assertEquals(reference, cd, 0.16 * reference);
        }
        assertEquals(10, SeaWind.speedAt(10, 10), 1e-9, "The profile passes through U10");
    }

    @Test void profileNearTheSurfaceLiesBetweenTheUsualPowerLaws() {
        // Over the sea the wind at 1 m is commonly put between the 1/7 power law (0.72 U10) and 1/11 (0.81 U10).
        for (double u : new double[]{5, 10, 20}) {
            double ratio = SeaWind.speedAt(u, 1) / u;
            assertTrue(ratio > 0.72 && ratio < 0.85, "U(1 m)/U10 at " + u + " m/s: " + ratio);
        }
        assertTrue(SeaWind.speedAt(10, 2) > SeaWind.speedAt(10, 1), "Wind grows with height");
    }

    @Test void fetchLimitedSeaMatchesTheCoastalEngineeringManual() {
        // CEM II-2-36 (EM 1110-2-1100, 2015) in friction-velocity form with C_D = 0.001 (1.1 + 0.035 U10): an independent
        // fit of the same growth; its worked example II-2-9 uses 0.751 for the period coefficient.
        double[][] cases = {{10, 10e3}, {10, 100e3}, {20, 10e3}, {15, 30e3}};
        for (double[] c : cases) {
            double u = c[0], x = c[1], ustar2 = 0.001 * (1.1 + 0.035 * u) * u * u, ustar = Math.sqrt(ustar2);
            double scaled = G * x / ustar2, hs = 4.13e-2 * Math.sqrt(scaled) * ustar2 / G, tp = 0.751 * Math.cbrt(scaled) * ustar / G;
            double[] sea = SeaWind.sea(u, x, Double.POSITIVE_INFINITY);
            System.out.printf("U10 %.0f m/s, fetch %.0f km: Hs %.2f m (CEM %.2f), Tp %.2f s (CEM %.2f)%n", u, x / 1e3, sea[0], hs, sea[1], tp);
            assertEquals(hs, sea[0], 0.12 * hs);
            assertEquals(tp, sea[1], 0.15 * tp);
        }
        // Growth stops at the fully developed Pierson-Moskowitz sea, which the fetch law reaches smoothly.
        double[] developed = SeaWind.sea(10, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        double[] long_ = SeaWind.sea(10, 1e8, Double.POSITIVE_INFINITY), justShort = SeaWind.sea(10, 0.97 * 2.08e4 * 100 / G, Double.POSITIVE_INFINITY);
        assertArrayEquals(developed, long_, 1e-12);
        assertEquals(developed[0], justShort[0], 0.03 * developed[0]);
        assertEquals(developed[1], justShort[1], 0.03 * developed[1]);
        // Shallow water holds the period (CEM II-2-39) and the height (depth saturation).
        double[] shallow = SeaWind.sea(15, 50e3, 2);
        assertEquals(9.78 * Math.sqrt(2 / G), shallow[1], 1e-9);
        assertTrue(1.86 * shallow[0] <= LinearWave.MAX_HEIGHT_TO_DEPTH * 2 + 1e-12);
        // Short fetch: held at the steepness limit, so the spectrum can be synthesised.
        double[] young = SeaWind.sea(12, 100, Double.POSITIVE_INFINITY);
        assertEquals(SeaWind.MAX_STEEPNESS, 2 * Math.PI * young[0] / (G * young[1] * young[1]), 1e-8);
        SeaSpectrum.synthesize(young[0], young[1], SeaSpectrum.recommendedGamma(young[0], young[1]), 0, 4, Double.POSITIVE_INFINITY, 32, 1);
    }

    @Test void growthTakesMinutesOnALakeAndHoursAtSea() {
        // CEM II-2-35 (Resio and Vincent 1982): time to fetch-limited growth.
        double lake = SeaWind.growthTime(10, 100), coast = SeaWind.growthTime(10, 10e3), ocean = SeaWind.growthTime(10, Double.POSITIVE_INFINITY);
        System.out.printf("Growth time at 10 m/s: 100 m fetch %.0f s, 10 km %.1f h, fully developed %.1f h%n", lake, coast / 3600, ocean / 3600);
        assertEquals(77.23 * Math.pow(100, 0.67) / (Math.pow(10, 0.34) * Math.pow(G, 0.33)), lake, 1e-9);
        assertTrue(lake > 60 && lake < 900, "Minutes on a lake");
        assertTrue(coast > 3600 && ocean > 10 * 3600, "Hours over the sea");
        assertTrue(SeaWind.growthTime(20, 100) < lake, "A stronger wind gets there sooner");
    }

    @Test void shelterFollowsTheWindbreakMeasurements() {
        // Brandle, Hodges and Zhou (2002), fig. 2.
        double[] at = {5, 10, 15, 20, 30};
        double[] solid = {0.25, 0.70, 0.90, 0.95, 1.00}, trees = {0.25, 0.35, 0.65, 0.85, 0.95};
        for (int i = 0; i < at.length; i++) {
            assertEquals(solid[i], SeaWind.shelter(at[i], false), 1e-12);
            assertEquals(trees[i], SeaWind.shelter(at[i], true), 1e-12);
        }
        assertEquals(0.25, SeaWind.shelter(1, false), 1e-12);
        assertEquals(1, SeaWind.shelter(50, true), 1e-12);
    }

    @Test void driftIsAFewPercentOfTheWindAndConfinedNearTheSurface() {
        double surface = SeaWind.drift(10, 0);
        System.out.printf("Wind drift at 10 m/s: %.3f m/s at the surface (%.1f %% of U10), %.3f m/s at 0.3 m%n", surface, 100 * surface / 10, SeaWind.drift(10, 0.3));
        // Wu (1975, 1983): the wind's own surface drift is a few percent of the wind, below the 3-3.5 % total with Stokes drift.
        assertTrue(surface / 10 > 0.012 && surface / 10 < 0.03);
        assertTrue(SeaWind.drift(10, 0.3) < 0.1 * surface);
        assertEquals(0, SeaWind.drift(0, 0), 0);
    }

    @Test void stillAirStaysFiniteAndContinuous() {
        // A region's tapered rim gives winds down to 1e-8 m/s; the profile must stay finite (0.16 crashed a rowing boat).
        for (double u : new double[]{1e-12, 1e-8, 1e-5, 0.01, 0.0999999, 0.1, 0.2}) {
            for (double z : new double[]{0.05, 0.5, 2, 10}) {
                double v = SeaWind.speedAt(u, z);
                assertTrue(Double.isFinite(v) && v >= 0, "speed at u10 " + u + ", z " + z + ": " + v);
            }
            assertTrue(Double.isFinite(SeaWind.frictionVelocity(u)) && SeaWind.frictionVelocity(u) > 0);
        }
        assertEquals(SeaWind.speedAt(SeaWind.LIGHT_AIR, 3), SeaWind.speedAt(SeaWind.LIGHT_AIR * (1 - 1e-12), 3), 1e-9, "Continuous at LIGHT_AIR");
        assertEquals(1e-8, SeaWind.speedAt(1e-8, 10), 1e-10, "Still about U10 at 10 m");
    }
}

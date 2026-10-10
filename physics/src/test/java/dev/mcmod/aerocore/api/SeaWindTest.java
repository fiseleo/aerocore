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
        assertTrue(shallow[0] <= CoastalWaves.SATURATED_HS_PER_DEPTH * 2 + 1e-12);
    }

    @Test void driftIsAFewPercentOfTheWindAndConfinedNearTheSurface() {
        double surface = SeaWind.drift(10, 0);
        System.out.printf("Wind drift at 10 m/s: %.3f m/s at the surface (%.1f %% of U10), %.3f m/s at 0.3 m%n", surface, 100 * surface / 10, SeaWind.drift(10, 0.3));
        // Wu (1975, 1983): the wind's own surface drift is a few percent of the wind, below the 3-3.5 % total with Stokes drift.
        assertTrue(surface / 10 > 0.012 && surface / 10 < 0.03);
        assertTrue(SeaWind.drift(10, 0.3) < 0.1 * surface);
        assertEquals(0, SeaWind.drift(0, 0), 0);
    }
}

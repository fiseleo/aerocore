package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SurfaceWindTest {
    @Test void lightAirStaysFinite() {
        // The world's wind tapers to nothing under roofs and at region rims: every conversion must stay finite.
        for (double u : new double[]{1e-12, 1e-8, 1e-5, 0.01, 0.0999999, 0.1, 0.2}) {
            double sea = SurfaceWind.overWater(u);
            assertTrue(Double.isFinite(sea) && sea > 0 && sea < 2 * u, "over water from " + u + ": " + sea);
            for (double z : new double[]{0.05, 2, 10, 80}) {
                double v = SurfaceWind.speed(u, z, SurfaceWind.OPEN, 0, 0);
                assertTrue(Double.isFinite(v) && v >= 0, "speed " + u + " at " + z + ": " + v);
            }
        }
        assertEquals(SurfaceWind.overWater(SeaWind.LIGHT_AIR), SurfaceWind.overWater(SeaWind.LIGHT_AIR * (1 - 1e-12)), 1e-9);
    }

    @Test void openTerrainIsTheReferenceAndTheBlendingHeightJoinsEverySurface() {
        assertEquals(10, SurfaceWind.speed(10, 10, SurfaceWind.OPEN, 0, 0), 1e-12, "Reference exposure at 10 m");
        for (double z0 : new double[]{SurfaceWind.SEA, SurfaceWind.SMOOTH, SurfaceWind.ROUGH, SurfaceWind.CLOSED}) {
            double below = SurfaceWind.speed(8, SurfaceWind.BLENDING_HEIGHT - 1e-9, z0, 0, 0), above = SurfaceWind.speed(8, SurfaceWind.BLENDING_HEIGHT, z0, 0, 0);
            assertEquals(above, below, 1e-6, "Continuous at the blending height over z0 = " + z0);
        }
        // Log law: the ratio of two heights over a forest with displacement is ln((z2 - d) / z0) / ln((z1 - d) / z0).
        double h = 15, d = 0.7 * h, z0 = 0.1 * h;
        assertEquals(Math.log((40 - d) / z0) / Math.log((25 - d) / z0),
                SurfaceWind.speed(10, 40, z0, d, h) / SurfaceWind.speed(10, 25, z0, d, h), 1e-12);
        // Inside the canopy it falls off exponentially from its top (Cionco a = 2): at mid-height exp(-1).
        double top = SurfaceWind.speed(10, h, z0, d, h);
        assertEquals(Math.exp(-1), SurfaceWind.speed(10, h / 2, z0, d, h) / top, 1e-12);
        // At 10 m a forest is far calmer than open land, and the reference profile is constant above 200 m.
        assertTrue(SurfaceWind.speed(10, 10, z0, d, h) < 0.4 * 10);
        assertEquals(SurfaceWind.reference(10, 200), SurfaceWind.speed(10, 320, SurfaceWind.OPEN, 0, 0), 1e-12);
    }

    @Test void windOverWaterIsAFifthStrongerThanOverLand() {
        // Same wind aloft, smoother sea: over-water 10 m winds are about 1.1-1.25 times those over open land
        // (Hsu 1986; WMO-No. 8 guidance on land-sea conversion).
        for (double u : new double[]{5, 10, 15}) {
            double ratio = SurfaceWind.overWater(u) / u;
            assertTrue(ratio > 1.08 && ratio < 1.25, "Sea / land at " + u + " m/s: " + ratio);
        }
        assertEquals(0, SurfaceWind.overWater(0), 0);
    }

    @Test void turbulenceIntensityMatchesEurocode() {
        // EN 1991-1-4 terrain category II (z0 = 0.05 m) at 10 m: I_v = 1 / ln(10 / 0.05) = 0.189.
        assertEquals(0.189, SurfaceWind.intensity(10, 0.05, 0), 5e-4);
        // sigma_u = 2.5 u*: I U = 2.5 u* by the log law.
        double u = SurfaceWind.speed(10, 10, SurfaceWind.ROUGH, 0, 0), ustar = SurfaceWind.frictionVelocity(10, SurfaceWind.ROUGH, 0);
        assertEquals(2.5 * ustar, SurfaceWind.intensity(10, SurfaceWind.ROUGH, 0) * u, 1e-9);
    }

    @Test void hillsSpeedTheWindUpAtTheirCrest() {
        // Taylor and Lee: a 3D hill of slope H / L_h = 0.25 speeds the surface wind at its crest by 1.6 x 0.25 = 40 %,
        // halving at about z = 0.17 L_h (exp(-4 z / L_h) = 1/2).
        assertEquals(0.4, SurfaceWind.hillSpeedUp(50, 200, 0, 1), 1e-12);
        assertEquals(0.2, SurfaceWind.hillSpeedUp(50, 200, 200 * Math.log(2) / 4, 1), 1e-12);
        assertEquals(0.2, SurfaceWind.hillSpeedUp(50, 200, 0, 0.5), 1e-12, "Half-way up the slope (rule)");
        assertEquals(SurfaceWind.HILL_B * SurfaceWind.MAX_HILL_SLOPE, SurfaceWind.hillSpeedUp(100, 50, 0, 1), 1e-12, "Slope capped at separation");
        assertEquals(0, SurfaceWind.hillSpeedUp(0, 100, 0, 1), 0);
    }

    @Test void effectiveRoughnessIsTheLogarithmicMean() {
        assertEquals(Math.sqrt(SurfaceWind.SEA * SurfaceWind.CLOSED), SurfaceWind.effectiveRoughness(new double[]{SurfaceWind.SEA, SurfaceWind.CLOSED}, new double[]{1, 1}), 1e-12);
    }

    @Test void synopticWindWandersSlowlyAndStaysWithinItsBounds() {
        SurfaceWind.Synoptic a = new SurfaceWind.Synoptic(42), b = new SurfaceWind.Synoptic(42);
        double sum = 0, maxStep = 0, previous = a.direction(0);
        int n = 0;
        for (double t = 0; t < 200000; t += 10) {
            double f = a.speedFactor(t), dir = a.direction(t);
            assertTrue(f >= 0.75 - 1e-12 && f <= 1.25 + 1e-12);
            assertEquals(f, b.speedFactor(t), 0, "Same seed, same wind");
            maxStep = Math.max(maxStep, Math.abs(dir - previous));
            previous = dir; sum += f; n++;
        }
        assertEquals(1, sum / n, 0.02, "Mean factor 1");
        // Over one tick (0.05 s) the direction moves less than a hundredth of a degree (about 10 degrees a minute at most).
        assertTrue(maxStep / 200 < Math.toRadians(1e-2), "Slow: " + maxStep);
        assertNotEquals(new SurfaceWind.Synoptic(1).direction(0), new SurfaceWind.Synoptic(2).direction(0));
    }
}

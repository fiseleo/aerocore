package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class WaterOpticsTest {
    @Test void fresnelMatchesTheTextbookLimits() {
        double n = WaterOptics.REFRACTIVE_INDEX;
        assertEquals(Math.pow((n - 1) / (n + 1), 2), WaterOptics.fresnel(1), 1e-12, "Normal incidence, 2.0 %");
        assertEquals(1, WaterOptics.fresnel(0), 1e-12, "Grazing");
        // Brewster's angle: the p reflectance vanishes, so the mean is half the s reflectance there.
        double brewster = Math.atan(n), ci = Math.cos(brewster), ct = Math.sqrt(1 - Math.pow(Math.sin(brewster) / n, 2));
        double rs = Math.pow((ci - n * ct) / (ci + n * ct), 2);
        assertEquals(rs / 2, WaterOptics.fresnel(ci), 1e-12);
        // From below: total internal reflection beyond asin(1/n) = 48.6 deg, the same 2 % straight up.
        double critical = Math.asin(1 / n);
        assertEquals(1, WaterOptics.fresnelFromWater(Math.cos(critical + 1e-6)), 1e-12);
        assertTrue(WaterOptics.fresnelFromWater(Math.cos(critical - 0.05)) < 0.5);
        assertEquals(WaterOptics.fresnel(1), WaterOptics.fresnelFromWater(1), 1e-12);
    }

    @Test void smithShadowingFollowsTheExactGaussianForm() {
        // Lambda(a) = (exp(-a^2) / (a sqrt(pi)) - erfc(a)) / 2 (Smith 1967); the rational fit is within 2.5 % + 0.005
        // (it drops to 0 at a = 1.6, where the exact value is about 0.002).
        double v = 0.05, sigma = Math.sqrt(v / 2);
        for (double a : new double[]{0.2, 0.5, 1.0, 1.4}) {
            double theta = Math.atan(1 / (a * sigma)), exact = (Math.exp(-a * a) / (a * Math.sqrt(Math.PI)) - erfc(a)) / 2;
            assertEquals(exact, WaterOptics.smithLambda(Math.cos(theta), v), 0.005 + 0.025 * exact, "a = " + a);
        }
        assertEquals(0, WaterOptics.smithLambda(1, v), 0, "Nothing hidden from straight above");
        assertTrue(WaterOptics.smithLambda(0.02, v) > 1, "Most facets hidden at grazing");
        // Toward the horizon the glitter levels off: Cox and Munk's 1 / cos(view) growth is cancelled by the masking.
        double sunUp = Math.sin(Math.toRadians(20)), sunOut = Math.cos(Math.toRadians(20));
        double[] radiance = new double[3];
        double[] elevations = {1, 0.3, 0.1};
        for (int i = 0; i < 3; i++) {
            double e = Math.toRadians(elevations[i]);
            radiance[i] = WaterOptics.glitter(0, 1, 0, -Math.cos(e), Math.sin(e), 0, sunOut, sunUp, 0, v);
        }
        System.out.printf("Glitter toward a 20 deg sun at view elevations 1, 0.3, 0.1 deg: %.3f %.3f %.3f%n", radiance[0], radiance[1], radiance[2]);
        assertTrue(radiance[2] < 1.3 * radiance[0] && radiance[1] < 1.3 * radiance[0], "Levels off toward the horizon");
    }

    /** erfc by Abramowitz and Stegun 7.1.26 (absolute error below 1.5e-7). */
    private static double erfc(double x) {
        double t = 1 / (1 + 0.3275911 * x);
        return t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429)))) * Math.exp(-x * x);
    }

    @Test void ripplesTravelAtTheCapillaryGravitySpeeds() {
        // The slowest capillary-gravity wave: 23.1 cm/s at 1.71 cm (clean water, 20 C; Lighthill, Waves in Fluids, 3.2).
        double k = WaterOptics.CAPILLARY_WAVENUMBER;
        assertEquals(0.0171, 2 * Math.PI / k, 2e-4);
        assertEquals(0.231, WaterOptics.rippleFrequency(k) / k, 2e-3);
        for (double q : new double[]{0.5, 0.9, 1.1, 2}) { assertTrue(WaterOptics.rippleFrequency(q * k) / (q * k) > WaterOptics.rippleFrequency(k) / k); }
        // Gravity waves well above it.
        assertEquals(Math.sqrt(Atmosphere.GRAVITY_MPS2 * 2), WaterOptics.rippleFrequency(2), 1e-4);
    }

    @Test void ripplesCarryCoxAndMunksSlopesLessTheSpectrums() {
        double wind = 10, direction = 0.4, resolved = 0.012;
        double[][] rows = WaterOptics.ripples(wind, direction, resolved, 3, 32, 7);
        assertEquals(32, rows.length);
        double total = 0, up = 0, cross = 0, previousK = 0;
        for (double[] r : rows) {
            double k = Math.hypot(r[0], r[1]), slope2 = 0.5 * Math.pow(r[2] * k, 2), angle = Math.atan2(r[1], r[0]) - direction;
            assertTrue(k > previousK && k >= 3 && k <= WaterOptics.CAPILLARY_WAVENUMBER, "Increasing, within the range");
            assertEquals(WaterOptics.rippleFrequency(k), r[3], 1e-12);
            assertTrue(Math.cos(angle) >= -1e-9, "Travelling downwind");
            assertTrue(r[2] / (2 * Math.PI / k) < 0.05, "Gentle: amplitude under 5 % of the wavelength");
            previousK = k;
            total += slope2; up += slope2 * Math.cos(angle) * Math.cos(angle); cross += slope2 * Math.sin(angle) * Math.sin(angle);
        }
        assertEquals(WaterOptics.slopeVariance(wind) - resolved, total, 1e-12, "Cox-Munk minus the spectrum");
        // The directions follow |cos|^n with n chosen for Cox and Munk's upwind/crosswind ratio (1.42 at 10 m/s);
        // with 32 random draws the ratio scatters, so only loosely.
        double[] v = WaterOptics.slopeVariances(wind);
        // Cox and Munk fitted the total and the two components separately: 5.12e-3 U against 3.16e-3 + 1.92e-3 = 5.08e-3 U.
        assertEquals(WaterOptics.slopeVariance(wind), v[0] + v[1], 0.01 * WaterOptics.slopeVariance(wind));
        System.out.printf("Ripple variance %.4f, upwind/crosswind %.2f (Cox-Munk %.2f)%n", total, up / cross, v[0] / v[1]);
        assertTrue(up / cross > 0.8 && up / cross < 3, "Upwind/crosswind ratio near Cox and Munk's");
        // Many seeds average to the ratio: the stratified expectation of the |cos|^n law is n + 1.
        double sumUp = 0, sumCross = 0;
        for (long seed = 0; seed < 400; seed++) {
            for (double[] r : WaterOptics.ripples(wind, direction, resolved, 3, 32, seed)) {
                double angle = Math.atan2(r[1], r[0]) - direction, s2 = 0.5 * Math.pow(r[2] * Math.hypot(r[0], r[1]), 2);
                sumUp += s2 * Math.cos(angle) * Math.cos(angle); sumCross += s2 * Math.sin(angle) * Math.sin(angle);
            }
        }
        assertEquals(v[0] / v[1], sumUp / sumCross, 0.05);
        // None when the spectrum already carries the measured slopes, reaches the capillary limit, or the same seed differs.
        assertEquals(0, WaterOptics.ripples(wind, direction, 1, 3, 32, 7).length);
        assertEquals(0, WaterOptics.ripples(wind, direction, resolved, 400, 32, 7).length);
        assertArrayEquals(rows[5], WaterOptics.ripples(wind, direction, resolved, 3, 32, 7)[5], 0);
    }
}

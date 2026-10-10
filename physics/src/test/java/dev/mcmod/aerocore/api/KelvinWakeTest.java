package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class KelvinWakeTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, RHO = 1000;

    @Test void thinShipWakeCarriesMichellsWaveResistance() {
        // Havelock's far-field energy flux of the synthesised amplitude function equals Michell's integral (Tuck 1989
        // form, implemented independently in HullResistance) with the same direction cut.
        SlenderHull hull = TestHull.HULL;
        SlenderHull.Particulars d = TestHull.model().design();
        for (double speed : new double[]{2.5, 3.5, 4.5}) {
            KelvinWake wake = KelvinWake.thinShip((x, depth) -> hull.halfBreadth(x, depth, d.waterlineY()), d.xAftM(), d.xForwardM(),
                    d.draftM(), 60, 12, speed, G, 0.15, 12);
            double lambdaMax = 1 / Math.cos(wake.thetaMax());
            double michell = HullResistance.michell((x, depth) -> hull.halfBreadth(x, depth, d.waterlineY()), d.xAftM(), d.xForwardM(),
                    d.draftM(), 60, 12, speed, RHO, G, lambdaMax);
            assertEquals(michell, wake.resistance(RHO), 0.01 * michell, "U=" + speed);
        }
    }

    @Test void patternMatchesDirectQuadratureAndStopsAtTheBow() {
        SlenderHull hull = TestHull.HULL;
        SlenderHull.Particulars d = TestHull.model().design();
        KelvinWake wake = KelvinWake.thinShip((x, depth) -> hull.halfBreadth(x, depth, d.waterlineY()), d.xAftM(), d.xForwardM(),
                d.draftM(), 60, 12, 3.5, G, 2, 40);
        KelvinWake.Pattern pattern = wake.pattern(40, 0.5);
        double max = pattern.maxElevation();
        assertTrue(max > 0.01, "A 1200 kg launch at 3.5 m/s makes centimetre waves: " + max);
        // Grid nodes behind the stern and beside the hull, within the fade-free part.
        for (double[] p : new double[][]{{-8, 0}, {-12.5, 3}, {-20, -6.5}, {-25, 8}, {1.5, 1.5}, {-1, -2}}) {
            double x = p[0], y = p[1];
            assertEquals(wake.elevation(x, y), pattern.elevation(x, y), 0.03 * max, "at " + x + ", " + y);
        }
        assertEquals(0, pattern.elevation(d.xForwardM() + 0.5, 0));
        assertEquals(0, wake.elevation(d.xForwardM() + 0.5, 0));
        // Symmetric hull: symmetric wake.
        assertEquals(pattern.elevation(-15, 4), pattern.elevation(-15, -4), 1e-6);
    }

    @Test void pressurePointWakeHasKelvinsDecayAndCuspAngle() {
        // Gaussian pressure point, a = 0.5 m, at U = 3 m/s (transverse wavelength 5.8 m).
        double speed = 3, a = 0.5;
        KelvinWake wake = KelvinWake.pressure(KelvinWake.gaussian(500, a), -a, a, a, RHO, speed, G, 0.4, 60);
        double k0 = wake.k0();
        double[] node = wake.node(wake.nodes() / 2);
        double a0 = Math.hypot(node[1], node[2]);
        // Along the track the transverse wave decays as |A(0)| sqrt(2 pi / (k0 d)) (stationary phase at theta = 0).
        for (double dist : new double[]{25, 40}) {
            double peak = 0;
            for (double x = -dist; x > -dist - wake.transverseWavelength(); x -= 0.02) { peak = Math.max(peak, Math.abs(wake.elevation(x, 0))); }
            assertEquals(a0 * Math.sqrt(2 * Math.PI / (k0 * dist)), peak, 0.1 * peak, "d=" + dist);
        }
        // Across the wake the strongest waves lie on the cusp lines at asin(1/3) = 19.47 deg.
        double dist = 50, best = 0, bestAngle = 0;
        for (double y = 0; y < 30; y += 0.05) {
            double h = Math.abs(wake.elevation(-dist, y));
            if (h > best) { best = h; bestAngle = Math.atan2(y, dist); }
        }
        assertEquals(Math.toDegrees(KelvinWake.CUSP_HALF_ANGLE), Math.toDegrees(bestAngle), 2.0);
        // Nothing ahead.
        assertEquals(0, wake.elevation(a + 0.1, 0));
    }

    @Test void slowMotionBelowTheResolvableWavelengthHasNoWake() {
        KelvinWake wake = KelvinWake.pressure(KelvinWake.gaussian(500, 0.5), -0.5, 0.5, 0.5, RHO, 0.5, G, 1, 20);
        assertTrue(wake.isEmpty());
        assertEquals(0, wake.resistance(RHO));
        assertEquals(0, wake.pattern(20, 0.5).maxElevation());
    }
}

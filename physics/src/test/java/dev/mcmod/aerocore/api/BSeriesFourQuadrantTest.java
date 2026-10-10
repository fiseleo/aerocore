package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.Locale;

import static org.junit.jupiter.api.Assertions.*;

/** The measured four-quadrant series against the independent open-water regression (Oosterveld and van Oossanen). */
class BSeriesFourQuadrantTest {
    private static final double SCALE_TO_KT = Math.PI / 8;

    @Test void aheadQuadrantAgreesWithTheRegression() {
        Object[][] cases = {{BSeriesFourQuadrant.B3_65_PITCH_1_0, new WageningenB(3, 0.65, 1.0)},
                {BSeriesFourQuadrant.B4_70_PITCH_1_0, new WageningenB(4, 0.70, 1.0)},
                {BSeriesFourQuadrant.B4_70_PITCH_1_4, new WageningenB(4, 0.70, 1.4)}};
        StringBuilder report = new StringBuilder();
        for (Object[] c : cases) {
            PropellerModel fourier = (PropellerModel) c[0];
            WageningenB regression = (WageningenB) c[1];
            for (double j : new double[]{0, 0.3, 0.6, 0.9}) {
                double beta = Math.atan2(j, 0.7 * Math.PI), scale = SCALE_TO_KT * (j * j + 0.49 * Math.PI * Math.PI);
                PropellerCoefficients f = fourier.sample(beta);
                double kt = f.thrust() * scale, kq = f.torque() * scale;
                report.append(String.format(Locale.ROOT, "%s J %.1f: KT %.4f (%.4f) 10KQ %.4f (%.4f)%n", regression, j, kt, regression.thrust(j),
                        10 * kq, 10 * regression.torque(j)));
                // Fourier fits and polynomials differ by a few per cent (both from MARIN data); 0.04 in K_T is about 6 % of bollard thrust.
                assertEquals(regression.thrust(j), kt, 0.04, report.toString());
                assertEquals(regression.torque(j), kq, 0.005, report.toString());
            }
        }
        System.out.println(report);
    }

    @Test void bollardValuesOfTheVerifiedTable() {
        // Report check: B4-70, P/D 1.0 at J = 0 gives K_T 0.4499 and K_Q 0.0661 from the series.
        PropellerCoefficients f = BSeriesFourQuadrant.B4_70_PITCH_1_0.sample(0);
        double scale = SCALE_TO_KT * 0.49 * Math.PI * Math.PI;
        assertEquals(0.4499, f.thrust() * scale, 0.001);
        assertEquals(0.0661, f.torque() * scale, 0.0005);
        // Crash-back (n < 0, advancing): the thrust reverses.
        assertTrue(BSeriesFourQuadrant.B4_70_PITCH_1_0.sample(Math.toRadians(150)).thrust() < 0);
    }
}

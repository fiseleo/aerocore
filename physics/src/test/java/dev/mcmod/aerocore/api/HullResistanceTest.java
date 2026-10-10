package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class HullResistanceTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void ittcLineMatchesItsDefinition() {
        assertEquals(4.6875e-3, HullResistance.ittc1957(1e6), 1e-7);
        assertEquals(3.0e-3, HullResistance.ittc1957(1e7), 1e-7);
        assertEquals(2.0833e-3, HullResistance.ittc1957(1e8), 1e-7);
        assertEquals(HullResistance.ittc1957(1e5), HullResistance.ittc1957(10), 0, "Held below Re = 1e5");
    }
    @Test void holtropFormFactorReproducesThePapersWorkedExample() {
        // Holtrop and Mennen (1982), ISP 29:166: L 205, B 32, T 10 m, C_P 0.5833, lcb -0.75 %, U-shaped stern (+10):
        // L_R = 81.385 m, c12 = 0.5102, 1 + k1 = 1.156.
        assertEquals(81.385, 205 * HullResistance.runLengthRatio(0.5833, -0.75), 0.005);
        assertEquals(0.5102, Math.pow(10.0 / 205, 0.2228446), 1e-4);
        assertEquals(1.156, HullResistance.holtropFormFactor(205, 32, 10, 0.5833, -0.75, 10), 0.0006);
    }
    @Test void hoernerFormFactorOfBodiesOfRevolution() {
        assertEquals(1.135, HullResistance.hoernerFormFactor(1.0 / 6), 0.001);
        assertEquals(1.054, HullResistance.hoernerFormFactor(0.1), 0.001);
    }
    @Test void michellIntegralReproducesTheWigleyHullCurve() {
        // Wigley hull y = (B/2)(1 - (2x/L)^2)(1 - (z/T)^2), L/B = 10, L/T = 16, S = 0.148791 L^2. Reference C_w x 1e3 from an
        // independent Michell table (thiru51/wigley-hull-openfoam-resistance): 1.06 / 1.83 / 1.25 / 2.73 at Fn 0.250 / 0.316 /
        // 0.350 / 0.400; the main hump 4.52 near Fn 0.50 was computed separately for this project.
        double length = 4, beam = length / 10, draft = length / 16, rho = 1000, area = 0.148791 * length * length;
        double[][] cases = {{0.25, 1.06}, {0.316, 1.83}, {0.35, 1.25}, {0.40, 2.73}, {0.50, 4.517}};
        for (double[] c : cases) {
            double speed = c[0] * Math.sqrt(G * length);
            double r = HullResistance.michell((x, z) -> beam / 2 * (1 - 4 * x * x / (length * length)) * (1 - z * z / (draft * draft)),
                    -length / 2, length / 2, draft, 80, 20, speed, rho, G, 200);
            double cw = r / (0.5 * rho * speed * speed * area) * 1e3;
            assertEquals(c[1], cw, 0.02 * c[1], "Fn=" + c[0]);
        }
    }
    @Test void michellResistanceScalesWithBeamSquaredAndIsDirectionFree() {
        double length = 3, draft = 0.2, speed = 2;
        DoubleBinaryOperatorFactory hull = b -> (x, z) -> b * (1 - x * x / 2.25) * (1 - z / draft) * (x > 0 ? 1 : 0.7 + 0.3 * (1 + x / 1.5));
        double r1 = HullResistance.michell(hull.of(0.1), -1.5, 1.5, draft, 60, 10, speed, 1000, G, 100);
        double r2 = HullResistance.michell(hull.of(0.2), -1.5, 1.5, draft, 60, 10, speed, 1000, G, 100);
        double reversed = HullResistance.michell((x, z) -> hull.of(0.1).applyAsDouble(-x, z), -1.5, 1.5, draft, 60, 10, speed, 1000, G, 100);
        assertEquals(4 * r1, r2, 1e-9 * r2);
        assertEquals(r1, reversed, 1e-6 * r1);
    }
    private interface DoubleBinaryOperatorFactory { java.util.function.DoubleBinaryOperator of(double halfBeam); }
}

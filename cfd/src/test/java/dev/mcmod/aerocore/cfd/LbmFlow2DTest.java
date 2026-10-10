package dev.mcmod.aerocore.cfd;

import dev.mcmod.aerocore.api.FluidState;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class LbmFlow2DTest {
    @Test void unforcedFluidStaysAtRestAndConservesMass() {
        LbmFlow2D flow = new LbmFlow2D(20, 16, 0.1, 0, 0, (x, y) -> x == 10 && y == 8);
        double mass = flow.diagnostics().mass();
        flow.advance(1000);
        assertEquals(mass, flow.diagnostics().mass(), 1e-9);
        assertEquals(0, flow.diagnostics().maxMach(), 1e-12);
        assertEquals(0, flow.diagnostics().obstacleForceX(), 1e-12);
        assertEquals(0, flow.diagnostics().obstacleForceY(), 1e-12);
    }

    @Test void poiseuilleProfileMatchesAnalyticSolutionAndConvergesUnderRefinement() {
        double coarseError = channelError(16, 6000);
        double fineError = channelError(32, 18000);
        assertTrue(coarseError < 0.01, "Coarse relative L2 error=" + coarseError);
        assertTrue(fineError < coarseError * 0.35, "Expected near second-order refinement; coarse=" + coarseError + ", fine=" + fineError);
        System.out.printf("Poiseuille L2 relative error: H=16 %.9g, H=32 %.9g%n", coarseError, fineError);
    }

    private static double channelError(int channelHeight, int steps) {
        double nu = 0.1, maximum = 0.02;
        double acceleration = 8 * nu * maximum / (channelHeight * channelHeight);
        LbmFlow2D flow = new LbmFlow2D(6, channelHeight + 2, nu, acceleration, 0, (x, y) -> false);
        double mass = flow.diagnostics().mass();
        flow.advance(steps);
        double errorSquared = 0, normSquared = 0;
        for (int y = 1; y <= channelHeight; y++) {
            double heightAboveWall = y - 0.5; // Halfway bounce-back walls at 0.5 and H+0.5.
            double expected = acceleration / (2 * nu) * heightAboveWall * (channelHeight - heightAboveWall);
            LbmFlow2D.Cell cell = flow.cell(3, y);
            errorSquared += Math.pow(cell.velocityX() - expected, 2);
            normSquared += expected * expected;
            assertEquals(0, cell.velocityY(), 1e-12);
            assertEquals(cell.velocityX(), flow.cell(0, y).velocityX(), 1e-12);
        }
        assertEquals(mass, flow.diagnostics().mass(), mass * 1e-10);
        return Math.sqrt(errorSquared / normSquared);
    }

    @Test void symmetricObstacleHasDragNoLiftAndPreservesMass() {
        LbmFlow2D flow = new LbmFlow2D(40, 24, 0.1, 1e-5, 0,
                (x, y) -> Math.pow(x - 12, 2) + Math.pow(y - 11.5, 2) < 9);
        double mass = flow.diagnostics().mass();
        flow.advance(4000);
        LbmFlow2D.Diagnostics d = flow.diagnostics();
        assertTrue(d.obstacleForceX() > 0);
        assertEquals(0, d.obstacleForceY(), 1e-10);
        assertEquals(mass, d.mass(), mass * 1e-10);
        assertTrue(d.maxMach() < 0.1);
        assertTrue(flow.cell(12, 12).solid());
        for (int y = 1; y < 23; y++) {
            assertEquals(flow.cell(5, y).velocityX(), flow.cell(5, 23 - y).velocityX(), 1e-11);
            assertEquals(flow.cell(5, y).velocityY(), -flow.cell(5, 23 - y).velocityY(), 1e-11);
        }
    }

    @Test void reportsOutOfEnvelopeInsteadOfSilentlyClampingTheFlow() {
        LbmFlow2D flow = new LbmFlow2D(8, 32, 0.05, 0.001, 0, (x, y) -> false);
        assertThrows(IllegalStateException.class, () -> flow.advance(1000));
        assertThrows(IllegalArgumentException.class, () -> new LbmFlow2D(8, 8, 0.001, 0, 0, (x, y) -> false));
        assertThrows(IllegalArgumentException.class, () -> new LbmFlow2D(8, 8, 0.1, 0, 0, (x, y) -> true));
    }

    @Test void siConversionPreservesReynoldsAndHasCorrectPressureForceDimensions() {
        FluidState water = FluidState.FRESH_WATER_20C;
        LatticeUnits units = LatticeUnits.forFluid(water, 0.001, 0.1);
        double reLattice = 0.02 * 10 / 0.1;
        double reSi = water.densityKgPerM3() * units.velocityMps(0.02) * 0.01 / water.dynamicViscosityPaS();
        assertEquals(reLattice, reSi, 1e-12);
        LatticeUnits simple = new LatticeUnits(0.5, 0.25, 1000);
        assertEquals(4, simple.velocityMps(2), 1e-12);
        assertEquals(8, simple.accelerationMps2(1), 1e-12);
        assertEquals(40, simple.pressureDifferencePa(1.03), 1e-10);
        assertEquals(2000, simple.forcePerDepthNpm(1), 1e-10);
    }
}

package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class PropellerTest {
    private static final BladeElementPropeller BEMT = new BladeElementPropeller(3, 0.2, 0.25, 1.0, BladeElementPropeller.THIN_SECTION, 24);
    private static final RigidBodyProperties BODY = new RigidBodyProperties(100, Vec3d.ZERO, new InertiaTensor(10, 10, 10, 0, 0, 0));
    private static final FluidState WATER = FluidState.FRESH_WATER_20C;

    @Test void staticThrustNeedsMoreThanIdealActuatorDiskPower() {
        // Momentum theory: hovering/bollard power is at least T^1.5 / sqrt(2 rho A), so figure of merit <= 1.
        PropellerSolver.Load load = run(BEMT, 0.3, 15, 0);
        double area = Math.PI * 0.3 * 0.3 / 4;
        double figureOfMerit = Math.pow(load.thrustN(), 1.5) / Math.sqrt(2 * WATER.densityKgPerM3() * area) / load.shaftPowerW();
        assertTrue(load.thrustN() > 0 && load.torqueNm() > 0);
        assertTrue(figureOfMerit > 0.3 && figureOfMerit < 1, "Figure of merit " + figureOfMerit);
    }
    @Test void forwardEfficiencyStaysBelowTheFroudeLimit() {
        double n = 15, d = 0.3, area = Math.PI * d * d / 4;
        for (double j : new double[]{0.2, 0.4, 0.6, 0.8}) {
            PropellerSolver.Load load = run(BEMT, d, n, j * n * d);
            double v = j * n * d, efficiency = load.thrustN() * v / load.shaftPowerW();
            double loading = load.thrustN() / (0.5 * WATER.densityKgPerM3() * v * v * area);
            double froude = 2 / (1 + Math.sqrt(1 + loading));
            assertTrue(load.thrustN() > 0, "J=" + j);
            assertTrue(efficiency > 0 && efficiency < froude, "J=" + j + " eta=" + efficiency + " ideal=" + froude);
        }
    }
    @Test void zeroThrustAdvanceRatioLiesJustBelowGeometricPitch() {
        double zero = Double.NaN;
        for (double j = 0.5; j <= 1.5; j += 0.002) {
            if (run(BEMT, 0.3, 15, j * 15 * 0.3).thrustN() <= 0) { zero = j; break; }
        }
        assertTrue(zero > 0.9 && zero < 1.0, "Zero-thrust J=" + zero + " for P/D=1");
    }
    @Test void loadsFollowPropellerSimilarityLaws() {
        PropellerModel table = BEMT.tabulate(361);
        PropellerSolver.Load base = run(table, 0.3, 10, 1.2), faster = run(table, 0.3, 20, 2.4), larger = run(table, 0.6, 10, 2.4);
        assertEquals(4 * base.thrustN(), faster.thrustN(), 1e-9 * Math.abs(base.thrustN()));
        assertEquals(4 * base.torqueNm(), faster.torqueNm(), 1e-9 * Math.abs(base.torqueNm()));
        assertEquals(16 * base.thrustN(), larger.thrustN(), 1e-9 * Math.abs(base.thrustN()));
        assertEquals(32 * base.torqueNm(), larger.torqueNm(), 1e-9 * Math.abs(base.torqueNm()));
        assertEquals(base.advanceRatio(), larger.advanceRatio(), 1e-12);
    }
    @Test void tabulatedModelMatchesDirectSolutionBetweenGridPoints() {
        // Working-state angles (10..49 deg) and their reversed mirrors, where the BEMT solution is smooth.
        PropellerModel table = BEMT.tabulate(361);
        for (double beta : new double[]{0.1876, 0.3012, 0.4567, 0.6543, 0.8321, 0.3012 - Math.PI, 0.6543 - Math.PI}) {
            PropellerCoefficients direct = BEMT.sample(beta), interpolated = table.sample(beta);
            double scale = Math.max(0.05, Math.abs(direct.thrust()));
            assertEquals(direct.thrust(), interpolated.thrust(), 0.01 * scale, "beta=" + beta);
            assertEquals(direct.torque(), interpolated.torque(), 0.01 * Math.max(0.01, Math.abs(direct.torque())), "beta=" + beta);
        }
    }
    @Test void reversingRotationAndAdvanceNegatesLoads() {
        PropellerSolver.Load forward = run(BEMT, 0.3, 12, 0.8), reverse = run(BEMT, 0.3, -12, -0.8);
        assertEquals(-forward.thrustN(), reverse.thrustN(), 1e-9 * Math.abs(forward.thrustN()));
        assertEquals(-forward.torqueNm(), reverse.torqueNm(), 1e-9 * Math.abs(forward.torqueNm()));
        assertEquals(forward.shaftPowerW(), reverse.shaftPowerW(), 1e-9 * forward.shaftPowerW());
    }
    @Test void lockedPropellerOnlyResistsTheFlow() {
        PropellerSolver.Load ahead = run(BEMT, 0.3, 0, 2), astern = run(BEMT, 0.3, 0, -2);
        assertTrue(ahead.thrustN() < 0 && astern.thrustN() > 0);
        assertEquals(0, ahead.shaftPowerW());
        assertTrue(Double.isNaN(ahead.advanceRatio()));
        assertEquals(0, run(BEMT, 0.3, 0, 0).thrustN());
    }
    @Test void thrustAndShaftReactionActOnTheBodyInWorldAxes() {
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.rotationVector(Vec3d.Z.scale(Math.PI / 2)), Vec3d.ZERO);
        Propeller propeller = new Propeller(new Vec3d(-1, 0, 0), Vec3d.X, 0.3, BEMT);
        PropellerSolver.Load load = PropellerSolver.load(s, BODY, propeller, 15, WATER);
        assertEquals(0, load.force().force().subtract(Vec3d.Y.scale(load.thrustN())).length(), 1e-9);
        assertEquals(0, load.force().couple().add(Vec3d.Y.scale(load.torqueNm())).length(), 1e-9);
        assertEquals(0, load.force().offset().subtract(new Vec3d(0, -1, 0)).length(), 1e-12);
        assertTrue(load.torqueNm() > 0);
    }
    @Test void tableRejectsDataOutsideItsRange() {
        PropellerTable table = new PropellerTable(List.of(new PropellerTable.Point(0, new PropellerCoefficients(0.5, 0.08)),
                new PropellerTable.Point(1, new PropellerCoefficients(-0.2, 0.02))));
        assertEquals(0.15, table.sample(0.5).thrust(), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> table.sample(1.1));
        assertThrows(IllegalArgumentException.class, () -> new BladeElementPropeller(1, 0.2, 0.25, 1, BladeElementPropeller.THIN_SECTION, 24));
    }

    @Test void bSeriesReproducesPublishedDesignExamples() {
        // MIT OCW 2.611 (2006) B5-75, P/D 1.0: intersection with K_T = 0.544 J^2 at J 0.64, eta 0.61;
        // with K_T = 0.544 J^4 at J 0.74, eta 0.67 (values read to two digits in the notes).
        WageningenB b575 = new WageningenB(5, 0.75, 1.0);
        double j2 = root(j -> b575.thrust(j) - 0.544 * j * j), j4 = root(j -> b575.thrust(j) - 0.544 * Math.pow(j, 4));
        assertEquals(0.64, j2, 0.006); assertEquals(0.61, b575.efficiency(j2), 0.006);
        assertEquals(0.74, j4, 0.006); assertEquals(0.67, b575.efficiency(j4), 0.006);
        // University of Michigan POP program output (NA470 example), with the Reynolds correction at Rn = 3.68e7.
        WageningenB pop = new WageningenB(4, 0.6293, 0.8171);
        assertEquals(0.1615, pop.thrust(0.5408, 3.68e7), 6e-5);
        assertEquals(0.02205, pop.torque(0.5408, 3.68e7), 6e-6);
        // Bernitsas et al. (1981) Fig. 21, B3-50: the P/D = 1.0 thrust curve ends at J = 1.08..1.09 (chart reading).
        assertEquals(1.085, new WageningenB(3, 0.5, 1.0).zeroThrustAdvanceRatio(), 0.02);
        assertThrows(IllegalArgumentException.class, () -> new WageningenB(8, 0.5, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new WageningenB(3, 0.5, 1.5));
    }
    @Test void demoScrewUsesTheRegressionAheadAndStaysContinuous() {
        PropellerModel screw = TestPlatform.PROPELLER.model();
        WageningenB series = TestPlatform.PROPELLER_SERIES;
        for (double j = 0; j <= 1.0; j += 0.05) {
            PropellerCoefficients table = screw.sample(Math.atan2(j, 0.7 * Math.PI)), data = series.regression(Math.atan2(j, 0.7 * Math.PI));
            assertEquals(data.thrust(), table.thrust(), 0.003, "C_T* at J=" + j);
            assertEquals(data.torque(), table.torque(), 0.0005, "C_Q* at J=" + j);
        }
        // The blending adds no jumps: all the way round, through the regression range, both 15 degree ramps and the
        // measured four-quadrant series, the table changes smoothly.
        // Each 1 degree step may change no more than twice what the measured series itself changes there, plus a margin.
        PropellerModel measured = BSeriesFourQuadrant.B3_65_PITCH_1_0;
        PropellerCoefficients previous = screw.sample(Math.toRadians(-180)), measuredPrevious = measured.sample(Math.toRadians(-180));
        for (int degree = -179; degree <= 180; degree++) {
            PropellerCoefficients next = screw.sample(Math.toRadians(degree)), measuredNext = measured.sample(Math.toRadians(degree));
            double thrustStep = 2 * Math.abs(measuredNext.thrust() - measuredPrevious.thrust()) + 0.02;
            double torqueStep = 2 * Math.abs(measuredNext.torque() - measuredPrevious.torque()) + 0.004;
            assertTrue(Math.abs(next.thrust() - previous.thrust()) < thrustStep && Math.abs(next.torque() - previous.torque()) < torqueStep, "Jump at " + degree);
            previous = next; measuredPrevious = measuredNext;
        }
        // A locked screw towed ahead only resists; bollard thrust follows K_T(0) rho n^2 D^4.
        assertTrue(screw.sample(Math.PI / 2).thrust() < 0);
        PropellerSolver.Load bollard = run(screw, 0.3, 15, 0);
        assertEquals(series.thrust(0) * WATER.densityKgPerM3() * 225 * Math.pow(0.3, 4), bollard.thrustN(), 0.005 * bollard.thrustN());
    }

    private static double root(java.util.function.DoubleUnaryOperator f) {
        double low = 0.05, high = 1.2;
        for (int i = 0; i < 80; i++) {
            double middle = 0.5 * (low + high);
            if (f.applyAsDouble(middle) > 0) { low = middle; } else { high = middle; }
        }
        return 0.5 * (low + high);
    }
    private static PropellerSolver.Load run(PropellerModel model, double d, double n, double speed) {
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, new Vec3d(speed, 0, 0), Orientation.IDENTITY, Vec3d.ZERO);
        return PropellerSolver.load(s, BODY, new Propeller(Vec3d.ZERO, Vec3d.X, d, model), n, WATER);
    }
}

package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class AirfoilValidationTest {
    @Test void stallReducesLiftAndIncreasesDragAfterTransition() {
        AeroCoefficients attached = SymmetricAirfoil.DEFAULT.sample(Math.toRadians(15));
        AeroCoefficients stalled = SymmetricAirfoil.DEFAULT.sample(Math.toRadians(25));
        assertFalse(attached.stalled());
        assertTrue(stalled.stalled());
        assertTrue(stalled.lift() < attached.lift());
        assertTrue(stalled.drag() > attached.drag());
    }

    @Test void stallTransitionIsContinuousAtBothBoundaries() {
        for (double degrees : new double[]{-25, -15, 15, 25}) {
            double angle = Math.toRadians(degrees);
            AeroCoefficients before = SymmetricAirfoil.DEFAULT.sample(angle - 1e-9);
            AeroCoefficients after = SymmetricAirfoil.DEFAULT.sample(angle + 1e-9);
            assertEquals(before.lift(), after.lift(), 2e-8);
            assertEquals(before.drag(), after.drag(), 2e-8);
        }
    }

    @Test void largerAspectRatioReducesInducedDrag() {
        SymmetricAirfoil longerWing = new SymmetricAirfoil(5.5, Math.toRadians(15), Math.toRadians(10), 0.025, 12, 0.8);
        assertTrue(longerWing.sample(0.1).drag() < SymmetricAirfoil.DEFAULT.sample(0.1).drag());
    }

    @Test void polarHasOddLiftAndEvenDrag() {
        for (int degrees = 0; degrees <= 180; degrees++) {
            AeroCoefficients positive = SymmetricAirfoil.DEFAULT.sample(Math.toRadians(degrees));
            AeroCoefficients negative = SymmetricAirfoil.DEFAULT.sample(Math.toRadians(-degrees));
            assertEquals(positive.lift(), -negative.lift(), 1e-12);
            assertEquals(positive.drag(), negative.drag(), 1e-12);
        }
    }

    @Test void rejectsInvalidGeometryAndPhysicalParameters() {
        assertThrows(IllegalArgumentException.class, () -> new Vec3d(Double.NaN, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> new Vec3d(0, Double.POSITIVE_INFINITY, 0));
        assertThrows(IllegalArgumentException.class, Vec3d.ZERO::normalized);
        assertThrows(IllegalArgumentException.class, () -> new AirState(-1, Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class, () -> new AirState(Double.NaN, Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class, () -> new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.X, 1, SymmetricAirfoil.DEFAULT));
        assertThrows(IllegalArgumentException.class, () -> new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 0, SymmetricAirfoil.DEFAULT));
        assertThrows(IllegalArgumentException.class, () -> new DragBody(Vec3d.ZERO, 1, -0.1));
        assertThrows(IllegalArgumentException.class, () -> new AeroCoefficients(1, -0.1, false));
        assertThrows(IllegalArgumentException.class, () -> new SymmetricAirfoil(5, 0.3, 0.2, 0.02, 6, 1.1));
        assertThrows(IllegalArgumentException.class, () -> new SymmetricAirfoil(5, 1, 1, 0.02, 6, 0.8));
        assertThrows(IllegalArgumentException.class, () -> SymmetricAirfoil.DEFAULT.sample(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> SymmetricAirfoil.DEFAULT.sample(4));
    }
}

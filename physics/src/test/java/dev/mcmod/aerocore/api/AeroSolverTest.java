package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AeroSolverTest {
    private static final AirState AIR = AirState.STILL_SEA_LEVEL;
    private static final LiftingSurface WING = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 10, SymmetricAirfoil.DEFAULT);

    private static BodyMotion flight(double speed, double degrees) {
        double angle = Math.toRadians(degrees);
        return new BodyMotion(new Vec3d(speed * Math.cos(angle), -speed * Math.sin(angle), 0), Vec3d.ZERO);
    }

    private static void assertVector(Vec3d expected, Vec3d actual) {
        assertEquals(expected.x(), actual.x(), 1e-8);
        assertEquals(expected.y(), actual.y(), 1e-8);
        assertEquals(expected.z(), actual.z(), 1e-8);
    }

    @Test void restingBodyHasNoForce() {
        assertEquals(Wrench.ZERO, AeroSolver.surface(flight(0, 0), AIR, WING).wrench());
    }

    @Test void vacuumHasNoForce() {
        AirState vacuum = new AirState(0, Vec3d.ZERO);
        assertEquals(Wrench.ZERO, AeroSolver.surface(flight(50, 10), vacuum, WING).wrench());
        assertEquals(Wrench.ZERO, AeroSolver.drag(flight(50, 10), vacuum, new DragBody(Vec3d.ZERO, 5, 0.8)));
    }

    @Test void analyticalLiftAndDragAtFiveDegrees() {
        SurfaceForces result = AeroSolver.surface(flight(30, 5), AIR, WING);
        double alpha = Math.toRadians(5);
        double cl = 5.5 * alpha;
        double q = 0.5 * 1.225 * 30 * 30;
        double lift = q * 10 * cl;
        double drag = q * 10 * (0.025 + cl * cl / (Math.PI * 6 * 0.8));
        assertEquals(alpha, result.angleOfAttackRadians(), 1e-12);
        assertEquals(q, result.dynamicPressurePa(), 1e-10);
        assertVector(new Vec3d(Math.sin(alpha) * lift, Math.cos(alpha) * lift, 0), result.lift());
        assertVector(new Vec3d(-Math.cos(alpha) * drag, Math.sin(alpha) * drag, 0), result.drag());
    }

    @Test void doublingSpeedQuadruplesForce() {
        Vec3d force = AeroSolver.surface(flight(20, 8), AIR, WING).wrench().force();
        assertVector(force.scale(4), AeroSolver.surface(flight(40, 8), AIR, WING).wrench().force());
    }

    @Test void densityAndAreaScaleForceLinearly() {
        Vec3d original = AeroSolver.surface(flight(20, 8), AIR, WING).wrench().force();
        assertVector(original.scale(2), AeroSolver.surface(flight(20, 8), new AirState(2.45, Vec3d.ZERO), WING).wrench().force());
        LiftingSurface twiceArea = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 20, SymmetricAirfoil.DEFAULT);
        assertVector(original.scale(2), AeroSolver.surface(flight(20, 8), AIR, twiceArea).wrench().force());
    }

    @Test void matchingWindCancelsAirspeed() {
        BodyMotion motion = flight(30, 5);
        AirState matchingWind = new AirState(1.225, motion.linearVelocity());
        assertEquals(Wrench.ZERO, AeroSolver.surface(motion, matchingWind, WING).wrench());
        assertEquals(Wrench.ZERO, AeroSolver.drag(motion, matchingWind, new DragBody(Vec3d.ZERO, 2, 1)));
    }

    @Test void commonVelocityShiftDoesNotChangeForce() {
        BodyMotion initial = flight(30, 5);
        Vec3d shift = new Vec3d(15, 2, -3);
        BodyMotion shifted = new BodyMotion(initial.linearVelocity().add(shift), Vec3d.ZERO);
        assertVector(AeroSolver.surface(initial, AIR, WING).wrench().force(),
                AeroSolver.surface(shifted, new AirState(1.225, shift), WING).wrench().force());
    }

    @Test void zeroAngleHasDragButNoLift() {
        SurfaceForces result = AeroSolver.surface(flight(20, 0), AIR, WING);
        assertVector(Vec3d.ZERO, result.lift());
        assertTrue(result.drag().x() < 0);
    }

    @Test void negativeAngleReversesLift() {
        SurfaceForces positive = AeroSolver.surface(flight(20, 8), AIR, WING);
        SurfaceForces negative = AeroSolver.surface(flight(20, -8), AIR, WING);
        assertEquals(positive.lift().y(), -negative.lift().y(), 1e-8);
        assertEquals(positive.drag().length(), negative.drag().length(), 1e-8);
    }

    @Test void forceAtOffsetCreatesCrossProductTorque() {
        LiftingSurface offsetWing = new LiftingSurface(new Vec3d(2, 0, 0), Vec3d.X, Vec3d.Y, 10, SymmetricAirfoil.DEFAULT);
        Wrench result = AeroSolver.surface(flight(30, 5), AIR, offsetWing).wrench();
        assertEquals(2 * result.force().y(), result.torque().z(), 1e-8);
        assertTrue(result.torque().z() > 0);
    }

    @Test void rotationChangesLocalFlow() {
        BodyMotion motion = new BodyMotion(Vec3d.ZERO, Vec3d.Z);
        assertVector(Vec3d.Y.scale(2), motion.velocityAt(Vec3d.X.scale(2)));
        LiftingSurface wing = new LiftingSurface(Vec3d.X.scale(2), Vec3d.X, Vec3d.Y, 10, SymmetricAirfoil.DEFAULT);
        Wrench result = AeroSolver.surface(motion, AIR, wing).wrench();
        assertTrue(result.force().y() < 0);
        assertTrue(result.torque().z() < 0, "Aerodynamic drag opposes rotation");
    }

    @Test void mirroredWingsCancelTorque() {
        LiftingSurface left = new LiftingSurface(Vec3d.Z.scale(3), Vec3d.X, Vec3d.Y, 5, SymmetricAirfoil.DEFAULT);
        LiftingSurface right = new LiftingSurface(Vec3d.Z.scale(-3), Vec3d.X, Vec3d.Y, 5, SymmetricAirfoil.DEFAULT);
        Wrench total = AeroSolver.solve(flight(30, 5), AIR, List.of(left, right), List.of());
        assertVector(Vec3d.ZERO, total.torque());
        assertVector(AeroSolver.surface(flight(30, 5), AIR, WING).wrench().force(), total.force());
    }

    @Test void rotatingCoordinateFrameRotatesOutput() {
        BodyMotion initial = flight(30, 5);
        // Proper rotation +90 degrees about world Y: (x,y,z) -> (z,y,-x).
        Vec3d originalVelocity = initial.linearVelocity();
        BodyMotion rotated = new BodyMotion(new Vec3d(originalVelocity.z(), originalVelocity.y(), -originalVelocity.x()), Vec3d.ZERO);
        LiftingSurface rotatedWing = new LiftingSurface(Vec3d.ZERO, Vec3d.Z.scale(-1), Vec3d.Y, 10, SymmetricAirfoil.DEFAULT);
        Vec3d originalForce = AeroSolver.surface(initial, AIR, WING).wrench().force();
        assertVector(new Vec3d(originalForce.z(), originalForce.y(), -originalForce.x()),
                AeroSolver.surface(rotated, AIR, rotatedWing).wrench().force());
    }

    @Test void pureSpanFlowRequiresBodyDrag() {
        BodyMotion motion = new BodyMotion(Vec3d.Z.scale(20), Vec3d.ZERO);
        assertEquals(Wrench.ZERO, AeroSolver.surface(motion, AIR, WING).wrench());
        Wrench drag = AeroSolver.drag(motion, AIR, new DragBody(Vec3d.ZERO, 2, 1));
        assertVector(new Vec3d(0, 0, -490), drag.force());
    }

    @Test void dragMatchesAnalyticalValueAndIncludesOffsetTorque() {
        Wrench drag = AeroSolver.drag(flight(20, 0), AIR, new DragBody(Vec3d.Y.scale(2), 3, 0.5));
        assertVector(new Vec3d(-367.5, 0, 0), drag.force());
        assertVector(new Vec3d(0, 0, 735), drag.torque());
    }

    @Test void liftIsPerpendicularAndDragIsDissipativeAcrossAngles() {
        Random random = new Random(12345);
        for (int i = 0; i < 500; i++) {
            BodyMotion motion = flight(1 + random.nextDouble() * 99, -180 + random.nextDouble() * 360);
            SurfaceForces result = AeroSolver.surface(motion, AIR, WING);
            assertEquals(0, result.lift().dot(motion.linearVelocity()), 1e-7);
            assertTrue(result.drag().dot(motion.linearVelocity()) <= 0);
        }
    }

    @Test void reverseFlowAndBroadsideRemainFiniteAndDissipative() {
        for (double angle : new double[]{-180, -90, 90, 180}) {
            BodyMotion motion = flight(30, angle);
            SurfaceForces result = AeroSolver.surface(motion, AIR, WING);
            assertTrue(result.coefficients().stalled());
            assertTrue(Double.isFinite(result.wrench().force().length()));
            assertTrue(result.wrench().force().dot(motion.linearVelocity()) < 0);
        }
    }

    @Test void emptyVehicleHasNoForce() {
        assertEquals(Wrench.ZERO, AeroSolver.solve(flight(30, 5), AIR, List.of(), List.of()));
    }

    @Test void solverCombinesWingAndBodyForces() {
        BodyMotion motion = flight(30, 5);
        DragBody drag = new DragBody(Vec3d.X, 2, 0.5);
        Wrench expected = AeroSolver.surface(motion, AIR, WING).wrench().add(AeroSolver.drag(motion, AIR, drag));
        Wrench actual = AeroSolver.solve(motion, AIR, List.of(WING), List.of(drag));
        assertVector(expected.force(), actual.force());
        assertVector(expected.torque(), actual.torque());
    }

    @Test void customPolarControlsForces() {
        LiftingSurface custom = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 2, alpha -> new AeroCoefficients(1, 0.1, false));
        SurfaceForces result = AeroSolver.surface(flight(20, 0), AIR, custom);
        assertVector(new Vec3d(0, 490, 0), result.lift());
        assertVector(new Vec3d(-49, 0, 0), result.drag());
    }
}

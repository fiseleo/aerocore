package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class FluidPhysicsTest {
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-9.80665);

    @Test void atmosphereMatchesIndependentStandardReferenceValues() {
        Atmosphere.Sample sea = Atmosphere.atAltitude(0, Vec3d.ZERO);
        assertEquals(288.15, sea.temperatureKelvin(), 1e-8);
        assertEquals(101325, sea.pressurePa(), 1e-6);
        assertEquals(1.225, sea.fluid().densityKgPerM3(), 1e-6);
        assertEquals(1.7894e-5, sea.fluid().dynamicViscosityPaS(), 2e-9);
        assertEquals(340.294, sea.fluid().speedOfSoundMps(), 0.001);
        Atmosphere.Sample high = Atmosphere.atAltitude(11000, Vec3d.X);
        assertEquals(216.65, high.temperatureKelvin(), 1e-8);
        assertEquals(22632.06, high.pressurePa(), 0.1);
        assertEquals(0.363918, high.fluid().densityKgPerM3(), 1e-6);
        assertEquals(Vec3d.X, high.fluid().velocity());
        assertEquals(5474.89, Atmosphere.atAltitude(20000, Vec3d.ZERO).pressurePa(), 0.1);
    }

    @Test void atmosphereIsContinuousAndDensityFallsWithHeight() {
        assertEquals(Atmosphere.atAltitude(11000 - 1e-6, Vec3d.ZERO).pressurePa(),
                Atmosphere.atAltitude(11000 + 1e-6, Vec3d.ZERO).pressurePa(), 1e-4);
        double previous = Double.POSITIVE_INFINITY;
        for (int h = -500; h <= 20000; h += 100) {
            double density = Atmosphere.atAltitude(h, Vec3d.ZERO).fluid().densityKgPerM3();
            assertTrue(density < previous);
            previous = density;
        }
    }

    @Test void dimensionsGiveExpectedReynoldsAndMach() {
        FlowConditions conditions = FlowConditions.of(new FluidState(2, 0.001, 400, Vec3d.ZERO), 20, 0.5);
        assertEquals(20000, conditions.reynoldsNumber(), 1e-9);
        assertEquals(0.05, conditions.machNumber(), 1e-12);
    }

    @Test void fluidAwarePolarReceivesSectionSpeedAndChord() {
        AirfoilPolar polar = new AirfoilPolar() {
            @Override public AeroCoefficients sample(double alpha) { throw new AssertionError("Flow overload required"); }
            @Override public AeroCoefficients sample(double alpha, FlowConditions flow) {
                assertEquals(20, flow.speedMps(), 1e-12);
                assertEquals(40000, flow.reynoldsNumber(), 1e-8);
                return new AeroCoefficients(1, 0.1, -0.2, false);
            }
        };
        LiftingSurface wing = new LiftingSurface(Vec3d.X.scale(2), Vec3d.X, Vec3d.Y, 3, 2, polar);
        SurfaceForces result = AeroSolver.surface(new BodyMotion(new Vec3d(20, 0, 7), Vec3d.ZERO),
                new FluidState(1, 0.001, 400, Vec3d.ZERO), wing);
        assertEquals(600, result.lift().y(), 1e-8);
        // Offset moment 1200 Nm plus intrinsic Cm moment -240 Nm.
        assertEquals(960, result.wrench().torque().z(), 1e-8);
    }

    @Test void waterAndAirUseTheSameLiftEquation() {
        BodyMotion motion = new BodyMotion(new Vec3d(4, -0.5, 0), Vec3d.ZERO);
        LiftingSurface wing = new LiftingSurface(Vec3d.ZERO, Vec3d.X, Vec3d.Y, 1, 0.5, SymmetricAirfoil.DEFAULT);
        FluidState air = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
        SurfaceForces airResult = AeroSolver.surface(motion, air, wing);
        SurfaceForces waterResult = AeroSolver.surface(motion, FluidState.FRESH_WATER_20C, wing);
        assertEquals(FluidState.FRESH_WATER_20C.densityKgPerM3() / air.densityKgPerM3(),
                waterResult.lift().length() / airResult.lift().length(), 1e-8);
        assertNotEquals(airResult.flow().reynoldsNumber(), waterResult.flow().reynoldsNumber());
    }

    @Test void fieldIsSampledAtEachWorldApplicationPoint() {
        FluidField field = (position, time) -> new FluidState(position.y(), 0.001, 400, Vec3d.ZERO);
        List<DragBody> bodies = List.of(new DragBody(Vec3d.Y, 1, 1), new DragBody(Vec3d.Y.scale(-1), 1, 1));
        Wrench result = AeroSolver.solve(new BodyMotion(Vec3d.X.scale(2), Vec3d.ZERO),
                Vec3d.Y.scale(10), field, 0, List.of(), bodies);
        assertEquals(-40, result.force().x(), 1e-8);
        assertEquals(4, result.torque().z(), 1e-8);
    }

    @Test void sphericalCapsHaveCorrectVolumeAndCentroid() {
        BuoyantSphere sphere = new BuoyantSphere(Vec3d.X, 2);
        assertEquals(0, HydroSolver.sphereImmersion(sphere, -2, Vec3d.Y).displacedVolumeM3());
        Immersion half = HydroSolver.sphereImmersion(sphere, 0, Vec3d.Y);
        assertEquals(sphere.volumeM3() / 2, half.displacedVolumeM3(), 1e-10);
        assertEquals(-0.75, half.centerOfBuoyancy().y(), 1e-10);
        assertEquals(0.5, half.fraction(), 1e-12);
        Immersion full = HydroSolver.sphereImmersion(sphere, 5, Vec3d.Y);
        assertEquals(sphere.volumeM3(), full.displacedVolumeM3(), 1e-10);
        assertEquals(sphere.offset(), full.centerOfBuoyancy());
        for (double d = -1.9; d < 2; d += 0.1) {
            assertEquals(sphere.volumeM3(), HydroSolver.sphereImmersion(sphere, d, Vec3d.Y).displacedVolumeM3()
                    + HydroSolver.sphereImmersion(sphere, -d, Vec3d.Y).displacedVolumeM3(), 1e-10);
        }
    }

    @Test void buoyancyBalancesDisplacedFluidWeightAndCreatesOffsetTorque() {
        FluidState water = FluidState.FRESH_WATER_20C;
        Wrench force = HydroSolver.buoyancy(water, new Immersion(2, new Vec3d(3, -1, 0), 0.5), GRAVITY);
        double expected = 998.2 * 2 * 9.80665;
        assertEquals(expected, force.force().y(), 1e-8);
        assertEquals(3 * expected, force.torque().z(), 1e-8);
        assertEquals(0, force.force().add(GRAVITY.scale(998.2 * 2)).length(), 1e-8);
        assertEquals(Wrench.ZERO, HydroSolver.buoyancy(water, new Immersion(0, Vec3d.ZERO, 0), GRAVITY));
    }

    @Test void immersionRotatesWithGravityDirection() {
        Immersion half = HydroSolver.sphereImmersion(new BuoyantSphere(Vec3d.ZERO, 1), 0, Vec3d.X);
        assertEquals(-0.375, half.centerOfBuoyancy().x(), 1e-12);
        assertEquals(0, half.centerOfBuoyancy().y(), 1e-12);
        Wrench wrench = HydroSolver.buoyancy(FluidState.FRESH_WATER_20C, half, Vec3d.X.scale(-10));
        assertTrue(wrench.force().x() > 0);
        assertEquals(0, wrench.torque().length(), 1e-12);
    }

    @Test void sphereDragApproachesStokesLimitAndDependsOnViscosity() {
        FluidState fluid = new FluidState(1000, 1, 1482, Vec3d.ZERO);
        BodyMotion motion = new BodyMotion(Vec3d.X.scale(1e-8), Vec3d.ZERO);
        BuoyantSphere sphere = new BuoyantSphere(Vec3d.Y, 0.01);
        Wrench drag = HydroSolver.sphereDrag(motion, fluid, sphere);
        double stokes = 6 * Math.PI * 1 * 0.01 * 1e-8;
        assertEquals(stokes, -drag.force().x(), stokes * 1e-4);
        assertEquals(-drag.force().x(), drag.torque().z(), 1e-15);
        Wrench thicker = HydroSolver.sphereDrag(motion, new FluidState(1000, 2, 1482, Vec3d.ZERO), sphere);
        assertEquals(2, thicker.force().x() / drag.force().x(), 1e-4);
    }

    @Test void currentCancelsDragAndDragOpposesRelativeMotion() {
        BuoyantSphere sphere = new BuoyantSphere(Vec3d.ZERO, 0.01);
        BodyMotion motion = new BodyMotion(Vec3d.X, Vec3d.ZERO);
        FluidState water = FluidState.FRESH_WATER_20C;
        assertEquals(Wrench.ZERO, HydroSolver.sphereDrag(motion, water.withVelocity(Vec3d.X), sphere));
        assertTrue(HydroSolver.sphereDrag(motion, water, sphere).force().dot(motion.linearVelocity()) < 0);
    }

    @Test void rejectsInvalidFluidGeometryAndUnsupportedEnvelope() {
        assertThrows(IllegalArgumentException.class, () -> Atmosphere.atAltitude(Double.NaN, Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class, () -> Atmosphere.atAltitude(20001, Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class, () -> new FluidState(1000, 0, 1482, Vec3d.ZERO));
        assertThrows(IllegalArgumentException.class, () -> new BuoyantSphere(Vec3d.ZERO, -1));
        assertThrows(IllegalArgumentException.class, () -> new Immersion(1, Vec3d.ZERO, 1.1));
        assertThrows(IllegalArgumentException.class, () -> HydroSolver.sphereDrag(
                new BodyMotion(Vec3d.X.scale(100), Vec3d.ZERO), FluidState.FRESH_WATER_20C, new BuoyantSphere(Vec3d.ZERO, 1)));
    }
}

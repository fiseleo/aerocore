package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class DynamicsTest {
    private static final Vec3d GRAVITY = Vec3d.Y.scale(-9.80665);
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();
    private static final FluidState WATER = FluidState.FRESH_WATER_20C;

    @Test void freeFallMatchesAnalyticalPositionAndVelocity() {
        TranslationState state = new TranslationState(Vec3d.Y.scale(100), Vec3d.X.scale(3));
        for (int i = 0; i < 200; i++) { state = LinearDynamics.constantForce(state, GRAVITY.scale(20), 20, 0.01); }
        assertEquals(6, state.position().x(), 1e-10);
        assertEquals(100 - 0.5 * 9.80665 * 4, state.position().y(), 1e-10);
        assertEquals(-9.80665 * 2, state.velocity().y(), 1e-10);
    }

    @Test void forceImpulseAndMassControlVelocityChange() {
        TranslationState state = new TranslationState(Vec3d.ZERO, Vec3d.ZERO);
        assertEquals(2, LinearDynamics.constantForce(state, Vec3d.X.scale(10), 5, 1).velocity().x(), 1e-12);
        assertEquals(1, LinearDynamics.constantForce(state, Vec3d.X.scale(10), 10, 1).velocity().x(), 1e-12);
    }

    @Test void quadraticDragMatchesExactDecayWithoutOvershoot() {
        FluidState fluid = new FluidState(2, 0.001, 300, Vec3d.ZERO);
        Vec3d result = LinearDynamics.quadraticDrag(Vec3d.X.scale(10), fluid, 3, 0.5, 5, 2);
        assertEquals(10.0 / 7, result.x(), 1e-12);
        Vec3d hugeStep = LinearDynamics.quadraticDrag(Vec3d.X.scale(1000), WATER, 1, 1, 1, 10);
        assertTrue(hugeStep.x() > 0 && hugeStep.x() < 1000);
    }

    @Test void dragIsGalileanInvariantAndDissipative() {
        Vec3d initial = new Vec3d(3, 4, -2), shift = new Vec3d(-7, 8, 1);
        Vec3d original = LinearDynamics.quadraticDrag(initial, WATER, 1, 0.47, 100, 0.5);
        Vec3d shifted = LinearDynamics.quadraticDrag(initial.add(shift), WATER.withVelocity(shift), 1, 0.47, 100, 0.5);
        assertEquals(0, original.add(shift).subtract(shifted).length(), 1e-12);
        assertTrue(original.lengthSquared() < initial.lengthSquared());
        assertEquals(shift, LinearDynamics.quadraticDrag(shift, WATER.withVelocity(shift), 1, 0.47, 100, 0.5));
    }

    @Test void neutralBuoyancyStaysAtRest() {
        double volume = 4.0 / 3 * Math.PI * 0.125;
        SphereDynamics.Properties properties = new SphereDynamics.Properties(WATER.densityKgPerM3() * volume, 0.5, 0.47);
        TranslationState start = new TranslationState(Vec3d.ZERO, Vec3d.ZERO);
        SphereDynamics.Result result = SphereDynamics.step(start, properties, AIR, WATER, 1, GRAVITY, 0.05);
        assertEquals(0, result.state().velocity().length(), 1e-12);
        assertEquals(0, result.netForce().length(), 1e-10);
    }

    @Test void heavySphereSinksAndLightSphereRisesWhenSubmerged() {
        TranslationState start = new TranslationState(Vec3d.ZERO, Vec3d.ZERO);
        assertTrue(SphereDynamics.step(start, new SphereDynamics.Properties(1000, 0.5, 0.47), AIR, WATER, 1, GRAVITY, 0.005).state().velocity().y() < 0);
        assertTrue(SphereDynamics.step(start, new SphereDynamics.Properties(250, 0.5, 0.47), AIR, WATER, 1, GRAVITY, 0.005).state().velocity().y() > 0);
    }

    @Test void floatingSphereApproachesDisplacementEquilibrium() {
        SphereDynamics.Properties properties = new SphereDynamics.Properties(250, 0.5, 0.47);
        BuoyantSphere sphere = new BuoyantSphere(Vec3d.ZERO, 0.5);
        TranslationState state = new TranslationState(Vec3d.Y.scale(1), Vec3d.ZERO);
        double fraction = 0;
        for (int i = 0; i < 24000; i++) {
            fraction = HydroSolver.sphereImmersion(sphere, -state.position().y(), Vec3d.Y).fraction();
            state = SphereDynamics.step(state, properties, AIR, WATER, fraction, GRAVITY, 0.005).state();
        }
        double expected = (250 / properties.volumeM3() - AIR.densityKgPerM3()) / (WATER.densityKgPerM3() - AIR.densityKgPerM3());
        assertEquals(expected, fraction, 0.03);
        assertTrue(Math.abs(state.velocity().y()) < 0.15, "Vertical speed=" + state.velocity().y());
    }

    @Test void reportedFluidForceMatchesIntegratedMomentum() {
        TranslationState state = new TranslationState(Vec3d.ZERO, new Vec3d(3, -7, 2));
        double dt = 0.005, mass = 250;
        SphereDynamics.Result result = SphereDynamics.step(state, new SphereDynamics.Properties(mass, 0.5, 0.47),
                AIR.withVelocity(Vec3d.X), WATER, 0.6, GRAVITY, dt);
        Vec3d fromMomentum = result.state().velocity().subtract(state.velocity()).scale(mass / dt);
        assertEquals(0, result.netForce().subtract(fromMomentum).length(), 1e-7);
    }

    @Test void invalidInputsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new SphereDynamics.Properties(0, 0.5, 0.47));
        assertThrows(IllegalArgumentException.class, () -> LinearDynamics.constantForce(new TranslationState(Vec3d.ZERO, Vec3d.ZERO), Vec3d.ZERO, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> SphereDynamics.step(new TranslationState(Vec3d.ZERO, Vec3d.ZERO),
                new SphereDynamics.Properties(250, 0.5, 0.47), AIR, WATER, Double.NaN, GRAVITY, 0.005));
    }
}

package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class DragHeatTest {
    private static final FluidState AIR = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid();

    @Test void dissipationAccountsForKineticEnergyLossAndConverges() {
        // Backward-Euler drag on body plus entrained water: E0 - E1 = dt * sum c|u1|^2 + |dv|^2_(M+Ma) / 2, so the
        // reported dissipation matches the energy actually removed up to an O(dt) numerical term. E includes the
        // added-mass kinetic energy, which the system drift carries along while the body rotates.
        double coarse = relativeEnergyError(0.01), fine = relativeEnergyError(0.0025);
        assertTrue(fine < 0.01, "fine=" + fine);
        assertTrue(fine < coarse * 0.5, "coarse=" + coarse + ", fine=" + fine);
    }
    @Test void dissipationIsFrameIndependentAndVanishesWhenMovingWithTheFlow() {
        RigidBodyProperties p = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);
        Vec3d shift = new Vec3d(0.7, 0, -0.4), v = new Vec3d(1, 0.2, 0.5), momentum = new Vec3d(30, -10, 20);
        double still = submergedStep(new RigidBodyState(Vec3d.ZERO, v, Orientation.IDENTITY, momentum), p, Vec3d.ZERO).totalW();
        double moving = submergedStep(new RigidBodyState(Vec3d.ZERO, v.add(shift), Orientation.IDENTITY, momentum), p, shift).totalW();
        assertTrue(still > 0);
        assertEquals(still, moving, still * 1e-9);
        assertEquals(0, submergedStep(new RigidBodyState(Vec3d.ZERO, shift, Orientation.IDENTITY, Vec3d.ZERO), p, shift).totalW(), 1e-12);
    }
    @Test void recoveryTemperatureWithUnitFactorIsTheStagnationTemperature() {
        // For an ideal gas T0/T = 1 + (gamma-1)/2 M^2, an independent form using the sound speed.
        ThermalFluidState air = Thermodynamics.dryAir(250, 80000, Vec3d.ZERO);
        double u = 200, mach = u / air.fluid().speedOfSoundMps();
        assertEquals(250 * 0.2 * mach * mach, HeatTransfer.recoveryTemperatureRise(air, u * u, 1), 1e-9);
        // Turbulent recovery factor of air, Pr^(1/3) with Pr = 0.71: about 0.89 as tabulated in heat-transfer texts.
        assertEquals(0.892, HeatTransfer.turbulentRecoveryFactor(air), 1e-3);
    }
    @Test void frictionHeatingIsBoundedByDissipatedPower() {
        ThermalFluidState water = Thermodynamics.liquidWater(293.15, 100000, Vec3d.ZERO);
        double r = HeatTransfer.turbulentRecoveryFactor(water);
        assertEquals(0, HeatTransfer.frictionHeatingW(500, water, 0, r, 10));
        assertEquals(500 * r * 4 / (2 * water.specificHeatCp()), HeatTransfer.frictionHeatingW(500, water, 4, r, 1000), 1e-12);
        assertEquals(0.01, HeatTransfer.frictionHeatingW(500, water, 4, r, 0.01));
        assertThrows(IllegalArgumentException.class, () -> HeatTransfer.frictionHeatingW(500, water, -1, r, 1));
    }
    @Test void dryAndWetMeanSquareSpeedsUseTheirOwnFluidVelocity() {
        RigidBodyProperties p = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        FloatingBodyDynamics.Environment half = (c, r) -> new FloatingBodyDynamics.Contact(0.5, Vec3d.ZERO,
                AIR.withVelocity(new Vec3d(4, 0, 0)), FluidState.FRESH_WATER_20C);
        FloatingBodyDynamics.DragDissipation d = FloatingBodyDynamics.step(s, p, TestPlatform.FLOATS, half, Vec3d.ZERO, 0.47, 0.005).dissipation();
        assertTrue(d.airMeanSquareSpeed() > 15 && d.airMeanSquareSpeed() <= 16, "Wind speed squared: " + d.airMeanSquareSpeed());
        assertTrue(d.waterMeanSquareSpeed() < 1e-4, "Still water sees only the small induced motion");
        assertTrue(d.airW() > 0);
    }

    private static double relativeEnergyError(double dt) {
        RigidBodyProperties p = TestPlatform.properties(new Vec3d(0.3, -0.2, 0.1));
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, new Vec3d(1.5, -0.4, 0.8),
                Orientation.rotationVector(new Vec3d(0.2, 0.1, -0.3)), new Vec3d(80, -40, 60));
        double initial = systemEnergy(s, p), dissipated = 0;
        for (int i = 0; i < (int) Math.round(2 / dt); i++) {
            FloatingBodyDynamics.Result result = FloatingBodyDynamics.step(s, p, TestPlatform.FLOATS, submerged(Vec3d.ZERO), Vec3d.ZERO, 0.47, dt);
            dissipated += result.dissipation().totalW() * dt;
            s = result.state();
        }
        double lost = initial - systemEnergy(s, p);
        return Math.abs(dissipated - lost) / lost;
    }
    /** Body kinetic energy plus 1/2 C_a rho V |v_point|^2 for each fully submerged float in still water. */
    private static double systemEnergy(RigidBodyState s, RigidBodyProperties p) {
        double energy = s.kineticEnergy(p);
        Vec3d omega = s.angularVelocity(p);
        for (BuoyantSphere f : TestPlatform.FLOATS) {
            double added = FloatingBodyDynamics.ADDED_MASS_COEFFICIENT * FluidState.FRESH_WATER_20C.densityKgPerM3() * f.volumeM3();
            energy += 0.5 * added * s.velocity().add(omega.cross(s.worldOffset(p, f.offset()))).lengthSquared();
        }
        return energy;
    }
    private static FloatingBodyDynamics.DragDissipation submergedStep(RigidBodyState s, RigidBodyProperties p, Vec3d flow) {
        return FloatingBodyDynamics.step(s, p, TestPlatform.FLOATS, submerged(flow), Vec3d.ZERO, 0.47, 0.005).dissipation();
    }
    private static FloatingBodyDynamics.Environment submerged(Vec3d flow) {
        return (c, r) -> new FloatingBodyDynamics.Contact(1, Vec3d.ZERO, AIR.withVelocity(flow), FluidState.FRESH_WATER_20C.withVelocity(flow));
    }
}

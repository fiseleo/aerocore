package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import static org.junit.jupiter.api.Assertions.*;

class WaveRadiationTest {
    @Test void snappedRadiiStayWithinOnePercentAndShareKernels() {
        for (double a = 0.001; a < 5; a *= 1.013) {
            double s = WaveRadiation.snapRadius(a);
            assertTrue(Math.abs(s - a) <= Math.max(0.0025 + 1e-12, 0.01 * a), "snap " + a + " -> " + s);
            assertEquals(s, WaveRadiation.snapRadius(s), 1e-12, "a snapped radius is on the grid");
        }
        assertEquals(0, WaveRadiation.snapRadius(0));
        double[] first = WaveRadiation.sharedKernel(0.4, 0.3, 1.2, 0.005, 1601);
        assertSame(first, WaveRadiation.sharedKernel(0.3, 0.4, 1.2, 0.005, 1601), "symmetric arguments share one kernel");
        assertArrayEquals(WaveRadiation.kernel(0.3, 0.4, 1.2, 0.005, 1601), first, 1e-12);
    }

    private static final double G = Atmosphere.GRAVITY_MPS2, DT = 0.005;

    @Test void hulmeTableIsKramersKroenigConsistent() {
        // A(0) - A(inf) = (2/pi) int B(omega)/omega^2 d omega = (1/pi) int (B/(rho V omega)) / ka d(ka) for the hemisphere.
        double sum = 0, step = 0.0005;
        for (double ka = step / 2; ka < 60; ka += step) { sum += WaveRadiation.hemisphereDamping(ka) / ka * step; }
        assertEquals(WaveRadiation.hemisphereAddedMass(0) - WaveRadiation.HEMISPHERE_ADDED_MASS_INFINITY, sum / Math.PI, 0.01);
    }
    @Test void besselJ0MatchesItsIntegralRepresentation() {
        for (double x = 0; x <= 25; x += 0.37) {
            double integral = 0; int n = 4000;
            for (int i = 0; i < n; i++) { integral += Math.cos(x * Math.sin(Math.PI * (i + 0.5) / n)); }
            assertEquals(integral / n, WaveRadiation.besselJ0(x), 2e-7, "x=" + x);
        }
    }
    @Test void waterplaneRadiusFollowsSphericalCapGeometry() {
        assertEquals(0.45, WaveRadiation.waterplaneRadius(0.45, 0.5), 1e-9);
        // Cap height h = R/2 holds 5/32 of the volume; its waterplane radius is sqrt(3)/2 R.
        assertEquals(Math.sqrt(3) / 2, WaveRadiation.waterplaneRadius(1, 5.0 / 32), 1e-9);
        assertEquals(0, WaveRadiation.waterplaneRadius(1, 0));
        assertEquals(0, WaveRadiation.waterplaneRadius(1, 1));
    }
    @Test void memoryReproducesHulmeDampingAndAddedMassUnderForcedHeave() {
        // Prescribed w = cos(omega t): the in-phase force gives B(omega); the quadrature part gives
        // A(omega) - A_inf = -(1/omega) int K sin, which must reproduce Hulme's independent added-mass column.
        double a = 0.5, volume = 2.0 / 3 * Math.PI * a * a * a;
        for (double ka : new double[]{0.3, 1.0, 2.0}) {
            double omega = Math.sqrt(ka * G / a);
            double[] projection = forcedResponse(List.of(Vec3d.ZERO), new double[]{a}, omega, new double[]{1});
            double damping = -2 * projection[0], added = 2 * projection[1] / omega;
            assertEquals(WaveRadiation.heaveDampingPerDensity(a, omega), damping, 0.02 * WaveRadiation.heaveDampingPerDensity(a, omega), "ka=" + ka);
            assertEquals(WaveRadiation.hemisphereAddedMass(ka) - WaveRadiation.HEMISPHERE_ADDED_MASS_INFINITY, added / volume, 0.03, "ka=" + ka);
        }
    }
    @Test void coincidentFloatsAddCoherentlyAndDistantFloatsDecouple() {
        double a = 0.44, omega = 5;
        List<Vec3d> together = List.of(Vec3d.ZERO, new Vec3d(1e-9, 0, 0));
        double single = -forcedResponse(List.of(Vec3d.ZERO), new double[]{a}, omega, new double[]{1})[0];
        double[] same = forcedResponses(together, new double[]{a, a}, omega, new double[]{1, 1});
        double[] anti = forcedResponses(together, new double[]{a, a}, omega, new double[]{1, -1});
        double inPhase = -(same[0] + same[1]), opposed = -(anti[0] + anti[1]);
        assertEquals(4 * single, inPhase, 0.01 * 4 * single, "Coincident sources radiate four times the power");
        assertEquals(0, opposed, 0.01 * single, "Opposed coincident sources cancel");
        double[] self = WaveRadiation.kernel(a, a, 0, DT, 1601), far = WaveRadiation.kernel(a, a, 60, DT, 1601);
        double selfMax = 0, farMax = 0;
        for (int i = 0; i < self.length; i++) { selfMax = Math.max(selfMax, Math.abs(self[i])); farMax = Math.max(farMax, Math.abs(far[i])); }
        assertTrue(farMax < 0.05 * selfMax, "Floats 60 m apart must barely interact: " + farMax / selfMax);
    }
    @Test void arrayRadiationOnlyRemovesEnergy() {
        // The cross-damping matrix sqrt(Bi Bj) J0(k dij) is positive semi-definite, so starting from rest the total
        // work done by radiation forces is never positive, whatever the velocity histories.
        Random random = new Random(3);
        List<Vec3d> layout = TestPlatform.FLOATS.stream().map(BuoyantSphere::offset).toList();
        for (int trial = 0; trial < 5; trial++) {
            WaveRadiation.Memory memory = new WaveRadiation.Memory(layout, DT);
            double[] radii = {0.30 + 0.15 * random.nextDouble(), 0.30 + 0.15 * random.nextDouble(), 0.30 + 0.15 * random.nextDouble(), 0.30 + 0.15 * random.nextDouble()};
            memory.setRadii(radii);
            double[][] amplitude = new double[4][4], frequency = new double[4][4], phase = new double[4][4];
            for (int i = 0; i < 4; i++) for (int q = 0; q < 4; q++) {
                amplitude[i][q] = random.nextDouble(); frequency[i][q] = 1 + 9 * random.nextDouble(); phase[i][q] = 2 * Math.PI * random.nextDouble();
            }
            double work = 0, scale = 0;
            for (int n = 0; n < 4000; n++) {
                double t = n * DT;
                double[] w = new double[4];
                for (int i = 0; i < 4; i++) for (int q = 0; q < 4; q++) { w[i] += amplitude[i][q] * Math.sin(frequency[i][q] * t + phase[i][q]) * Math.min(1, t); }
                memory.record(w, radii);
                double[] f = memory.verticalForces(998.2);
                for (int i = 0; i < 4; i++) { work += f[i] * w[i] * DT; scale += Math.abs(f[i] * w[i]) * DT; }
            }
            assertTrue(work < 1e-3 * scale, "Trial " + trial + ": radiation did positive work " + work + " of " + scale);
        }
    }
    @Test void floatingHemisphereFreeHeaveMatchesHulmeLinearTheory() {
        // Sphere floating half immersed: K = rho g pi a^2, mass = rho V with V the hemisphere volume, so linear
        // resonance obeys K = omega^2 (m + A(omega)), i.e. ka = 3 / (2 (1 + mu(ka))), and the damping ratio is
        // zeta = lambda(ka) / (2 (1 + mu(ka))) with Hulme's mu and lambda. This checks the instantaneous added mass
        // (FloatingBodyDynamics) together with the radiation memory against the exact floating-hemisphere result.
        double a = 0.5, rho = FluidState.FRESH_WATER_20C.densityKgPerM3();
        double air = Atmosphere.atAltitude(0, Vec3d.ZERO).fluid().densityKgPerM3(), sphere = 4.0 / 3 * Math.PI * a * a * a;
        double mass = (rho + air) * sphere / 2, inertia = 0.4 * mass * a * a;
        RigidBodyProperties p = new RigidBodyProperties(mass, Vec3d.ZERO, new InertiaTensor(inertia, inertia, inertia, 0, 0, 0));
        List<BuoyantSphere> hull = List.of(new BuoyantSphere(Vec3d.ZERO, a));
        WaveRadiation.Memory memory = new WaveRadiation.Memory(List.of(Vec3d.ZERO), DT);
        memory.setRadii(new double[]{a});
        RigidBodyState s = new RigidBodyState(new Vec3d(0, 0.005, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        double[] y = new double[3000];
        for (int n = 0; n < y.length; n++) {
            FloatingBodyDynamics.Result result = FloatingBodyDynamics.step(s, p, hull, TestPlatform.flatWater(0), Vec3d.Y.scale(-G), 0,
                    memory.loads(s, p, hull, Vec3d.Y, rho), null, DT);
            s = result.state();
            memory.record(s, p, hull, Vec3d.Y, result.wetFractions());
            y[n] = s.position().y();
        }
        List<Integer> peaks = new ArrayList<>();
        for (int n = 1; n < y.length - 1; n++) { if (y[n] > y[n - 1] && y[n] >= y[n + 1] && y[n] > 1e-6) { peaks.add(n); } }
        assertTrue(peaks.size() >= 4, "Hemisphere must oscillate: " + peaks.size());
        // Exact resonance from Hulme's table by fixed-point iteration on ka.
        double ka = 1;
        for (int i = 0; i < 50; i++) { ka = 3 / (2 * (1 + WaveRadiation.hemisphereAddedMass(ka))); }
        double omega = Math.sqrt(ka * G / a), zeta = WaveRadiation.hemisphereDamping(ka) / (2 * (1 + WaveRadiation.hemisphereAddedMass(ka)));
        int first = peaks.get(1), last = peaks.get(4);
        double measuredOmega = 2 * Math.PI * 3 / ((last - first) * DT);
        double decrement = Math.log(y[first] / y[last]) / 3;
        assertEquals(omega, measuredOmega, 0.03 * omega);
        assertEquals(2 * Math.PI * zeta / Math.sqrt(1 - zeta * zeta), decrement, 0.15 * 2 * Math.PI * zeta);
    }
    @Test void platformHeaveRingsForeverWithoutRadiationButDecaysWithIt() {
        // Quadratic drag off: without radiation only the tiny Stokes term damps heave.
        assertTrue(heaveAmplitudeRatio(false) > 0.95, "Without radiation heave must persist");
        assertTrue(heaveAmplitudeRatio(true) < 0.05, "Radiation must remove heave energy within 10 s");
    }

    /** Ratio of late (8..10 s) to early (0..2 s) peak-to-peak heave of the demonstrator platform. */
    private static double heaveAmplitudeRatio(boolean radiation) {
        RigidBodyProperties p = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);
        List<Vec3d> layout = TestPlatform.FLOATS.stream().map(BuoyantSphere::offset).toList();
        WaveRadiation.Memory memory = new WaveRadiation.Memory(layout, DT);
        double rho = FluidState.FRESH_WATER_20C.densityKgPerM3();
        RigidBodyState s = TestPlatform.atModelOrigin(new Vec3d(0, 0.106 + 0.02, 0), Orientation.IDENTITY, p);
        double earlyMin = Double.MAX_VALUE, earlyMax = -Double.MAX_VALUE, lateMin = Double.MAX_VALUE, lateMax = -Double.MAX_VALUE;
        for (int n = 0; n < 2000; n++) {
            List<AppliedForce> loads = radiation ? memory.loads(s, p, TestPlatform.FLOATS, Vec3d.Y, rho) : List.of();
            FloatingBodyDynamics.Result result = FloatingBodyDynamics.step(s, p, TestPlatform.FLOATS, TestPlatform.flatWater(0),
                    Vec3d.Y.scale(-G), 0, loads, null, DT);
            s = result.state();
            memory.record(s, p, TestPlatform.FLOATS, Vec3d.Y, result.wetFractions());
            double y = s.position().y();
            if (n < 400) { earlyMin = Math.min(earlyMin, y); earlyMax = Math.max(earlyMax, y); }
            if (n >= 1600) { lateMin = Math.min(lateMin, y); lateMax = Math.max(lateMax, y); }
        }
        return (lateMax - lateMin) / (earlyMax - earlyMin);
    }
    /** Mean of F cos(omega t) and F sin(omega t) for float 0 under w_i = amplitude_i cos(omega t), per unit density. */
    private static double[] forcedResponse(List<Vec3d> layout, double[] radii, double omega, double[] amplitudes) {
        double[] r = forcedProjection(layout, radii, omega, amplitudes);
        return new double[]{r[0], r[1]};
    }
    /** Mean of F_i cos(omega t) for every float. */
    private static double[] forcedResponses(List<Vec3d> layout, double[] radii, double omega, double[] amplitudes) {
        double[] r = forcedProjection(layout, radii, omega, amplitudes), out = new double[layout.size()];
        for (int i = 0; i < out.length; i++) { out[i] = r[2 * i] * amplitudes[i]; }
        return out;
    }
    private static double[] forcedProjection(List<Vec3d> layout, double[] radii, double omega, double[] amplitudes) {
        WaveRadiation.Memory memory = new WaveRadiation.Memory(layout, DT);
        memory.setRadii(radii);
        int floats = layout.size(), start = (int) Math.ceil(9 / DT), periods = 20;
        int end = start + (int) Math.round(periods * 2 * Math.PI / omega / DT);
        double[] sums = new double[2 * floats];
        for (int n = 0; n <= end; n++) {
            double t = n * DT, c = Math.cos(omega * t);
            double[] w = new double[floats];
            for (int i = 0; i < floats; i++) { w[i] = amplitudes[i] * c; }
            memory.record(w, radii);
            double[] f = memory.verticalForces(1);
            if (n > start) {
                for (int i = 0; i < floats; i++) { sums[2 * i] += f[i] * c; sums[2 * i + 1] += f[i] * Math.sin(omega * t); }
            }
        }
        for (int i = 0; i < sums.length; i++) { sums[i] /= end - start; }
        return sums;
    }
}

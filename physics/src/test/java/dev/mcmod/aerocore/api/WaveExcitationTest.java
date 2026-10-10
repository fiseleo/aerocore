package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

/** Heave excitation of a floating hemisphere: the diffraction table against independent references and in use. */
class WaveExcitationTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, RHO = FluidState.FRESH_WATER_20C.densityKgPerM3();
    /** ka values of Hulme's published table (WaveRadiation.HULME), excluding ka = 0. */
    private static final double[] HULME_KA = {0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0};

    @Test void excitationMagnitudeObeysHaskindWithHulmesPublishedDamping() {
        // Deep water: B = omega^3 |X|^2 / (2 rho g^3), i.e. |X^| = sqrt(4 b / (3 pi ka)) with b = B / (rho V omega).
        for (double ka : HULME_KA) {
            double[] x = WaveRadiation.hemisphereExcitation(ka);
            double haskind = Math.sqrt(4 * WaveRadiation.hemisphereDamping(ka) / (3 * Math.PI * ka));
            assertEquals(haskind, Math.hypot(x[0], x[1]), 0.003 * haskind, "ka=" + ka);
        }
    }
    @Test void froudeKrylovColumnMatchesTheIncidentPressureIntegral() {
        // X_FK / (rho g pi a^2 A) = 2 int_0^1 mu exp(-ka mu) J0(ka sqrt(1 - mu^2)) d mu over the hemisphere.
        for (double ka : HULME_KA) {
            int n = 20000;
            double sum = 0;
            for (int i = 0; i < n; i++) {
                double mu = (i + 0.5) / n;
                sum += mu * Math.exp(-ka * mu) * WaveRadiation.besselJ0(ka * Math.sqrt(1 - mu * mu));
            }
            assertEquals(2 * sum / n, WaveRadiation.hemisphereFroudeKrylov(ka), 2e-4, "ka=" + ka);
        }
    }
    @Test void longWaveExcitationLeadsByTheRadiationTerm() {
        // Long waves: Im X^ = -(2/3) ka b to leading order (the force leads the elevation by the damping term),
        // and Re X^ = 1 - (2/3) ka (1 + mu(0)).
        for (double ka : new double[]{0.01, 0.05, 0.1}) {
            double[] x = WaveRadiation.hemisphereExcitation(ka);
            assertEquals(-2.0 / 3 * ka * WaveRadiation.hemisphereDamping(ka), x[1], 0.05 * Math.abs(x[1]) + 1e-5, "ka=" + ka);
        }
        assertEquals(1 - 2.0 / 3 * 0.01 * (1 + WaveRadiation.hemisphereAddedMass(0)), WaveRadiation.hemisphereExcitation(0.01)[0], 2e-4);
        assertArrayEquals(new double[]{0, 0}, WaveRadiation.hemisphereDiffraction(0), 1e-12);
        assertArrayEquals(new double[]{0, 0}, WaveRadiation.hemisphereDiffraction(4), 1e-12);
    }
    @Test void sampledPressureWithTableDiffractionGivesTheExactFixedBodyForce() {
        // A fixed sphere centred on the still level: the dynamic part of the sampled pressure integral is the exact
        // Froude-Krylov force; adding heaveDiffraction must give rho g pi a^2 A X^ in magnitude and phase.
        double a = 0.5, amplitude = 0.01, volume = 4.0 / 3 * Math.PI * a * a * a;
        for (double ka : new double[]{0.3, 1.0, 1.5}) {
            LinearWave wave = new LinearWave(amplitude, 2 * Math.PI * a / ka, 0, Double.POSITIVE_INFINITY, 0);
            WaveField waves = new WaveField(List.of(wave));
            double still = HydroSolver.sphereInWaves(Vec3d.ZERO, a, WaveField.NONE, 0, 0, 41).pressureFraction();
            double omega = wave.angularFrequency(), period = wave.periodS(), cos = 0, sin = 0;
            int n = 400;
            for (int i = 0; i < n; i++) {
                double t = period * i / n;
                double froudeKrylov = RHO * G * volume * (HydroSolver.sphereInWaves(Vec3d.ZERO, a, waves, 0, t, 41).pressureFraction() - still);
                double force = froudeKrylov + WaveRadiation.heaveDiffraction(waves, 0, 0, t, a, RHO);
                // eta(0, t) = A cos(-omega t): project F onto cos(omega t) and sin(omega t).
                cos += force * Math.cos(omega * t); sin += force * Math.sin(omega * t);
            }
            double[] x = WaveRadiation.hemisphereExcitation(ka);
            double scale = RHO * G * Math.PI * a * a * amplitude;
            // F = scale (Re X cos(omega t) + Im X sin(omega t)) for theta = -omega t.
            assertEquals(x[0], 2 * cos / n / scale, 0.01, "Real part, ka=" + ka);
            assertEquals(x[1], 2 * sin / n / scale, 0.01, "Imaginary part, ka=" + ka);
        }
    }
    @Test void floatingHemisphereHeaveResponseMatchesFrequencyDomainTheory() {
        // Free half-immersed sphere in regular waves: time-domain pressure integral, radiation memory and table
        // diffraction must reproduce x/A = X^ C / (C - omega^2 (m + A(omega)) - i omega B(omega)), with Hulme's
        // A and B and C = rho g pi a^2. The superseded Morison diffraction is checked to be clearly worse near ka 1.4.
        double a = 0.5;
        for (double ka : new double[]{0.3, 0.9, 1.4}) {
            double omega = Math.sqrt(ka * G / a), hemisphere = 2.0 / 3 * Math.PI * a * a * a;
            double c = RHO * G * Math.PI * a * a, added = WaveRadiation.hemisphereAddedMass(ka) * RHO * hemisphere;
            double damping = WaveRadiation.hemisphereDamping(ka) * RHO * hemisphere * omega;
            double mass = (RHO + Atmosphere.atAltitude(0, Vec3d.ZERO).fluid().densityKgPerM3()) * hemisphere;
            double[] x = WaveRadiation.hemisphereExcitation(ka);
            // Complex division (X c) / (re + i im).
            double re = c - omega * omega * (mass + added), im = -omega * damping, denominator = re * re + im * im;
            double expected = Math.hypot((x[0] * c * re + x[1] * c * im) / denominator, (x[1] * c * re - x[0] * c * im) / denominator);
            double measured = heaveResponse(a, ka, true);
            assertEquals(expected, measured, 0.04 * expected, "Heave RAO, ka=" + ka);
            if (ka == 1.4) {
                double old = heaveResponse(a, ka, false);
                assertTrue(Math.abs(old - expected) > 3 * Math.abs(measured - expected), "Morison diffraction " + old + " vs exact " + expected);
            }
        }
    }

    /** Heave amplitude per wave amplitude of a free floating sphere, Fourier-projected after transients. */
    private static double heaveResponse(double a, double ka, boolean exactDiffraction) {
        double amplitude = 0.01, dt = 0.005;
        LinearWave wave = new LinearWave(amplitude, 2 * Math.PI * a / ka, 0, Double.POSITIVE_INFINITY, 0);
        WaveField waves = new WaveField(List.of(wave));
        double hemisphere = 2.0 / 3 * Math.PI * a * a * a, inertia;
        double mass = (RHO + Atmosphere.atAltitude(0, Vec3d.ZERO).fluid().densityKgPerM3()) * hemisphere;
        inertia = 0.4 * mass * a * a;
        RigidBodyProperties p = new RigidBodyProperties(mass, Vec3d.ZERO, new InertiaTensor(inertia, inertia, inertia, 0, 0, 0));
        List<BuoyantSphere> hull = List.of(new BuoyantSphere(Vec3d.ZERO, a));
        WaveRadiation.Memory memory = new WaveRadiation.Memory(List.of(Vec3d.ZERO), dt);
        memory.setRadii(new double[]{a});
        RigidBodyState s = new RigidBodyState(Vec3d.ZERO, Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        double omega = wave.angularFrequency(), period = wave.periodS();
        int start = (int) Math.ceil(40 / dt), end = start + (int) Math.round(10 * period / dt);
        double cos = 0, sin = 0;
        for (int i = 0; i < end; i++) {
            double t = i * dt;
            List<AppliedForce> loads = new java.util.ArrayList<>(memory.loads(s, p, hull, Vec3d.Y, RHO));
            if (exactDiffraction) { loads.addAll(memory.diffractionLoads(s, p, hull, RHO, waves, t)); }
            FloatingBodyDynamics.Result result = FloatingBodyDynamics.step(s, p, hull, TestPlatform.wavyWater(0, waves, t, exactDiffraction),
                    Vec3d.Y.scale(-G), 0, loads, null, dt);
            // Heave only, as in the frequency-domain heave equation: an unmoored sphere otherwise wanders ~1 m sideways
            // after the start-up transient, which shifts the phase of its forcing.
            s = new RigidBodyState(new Vec3d(0, result.state().position().y(), 0), new Vec3d(0, result.state().velocity().y(), 0),
                    result.state().orientation(), Vec3d.ZERO);
            memory.record(s, p, hull, Vec3d.Y, result.wetFractions());
            if (i >= start) {
                double time = (i + 1) * dt, y = s.position().y();
                cos += y * Math.cos(omega * time); sin += y * Math.sin(omega * time);
            }
        }
        return 2 * Math.hypot(cos, sin) / (end - start) / amplitude;
    }
}

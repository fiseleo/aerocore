package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import java.util.List;
import java.util.Random;
import static org.junit.jupiter.api.Assertions.*;

class WaveTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void dispersionMatchesDeepAndShallowWaterLimits() {
        LinearWave deep = new LinearWave(0.1, 10, 0, Double.POSITIVE_INFINITY, 0);
        double k = 2 * Math.PI / 10;
        assertEquals(Math.sqrt(G * k), deep.angularFrequency(), 1e-12);
        assertEquals(Math.sqrt(G / k), deep.phaseSpeedMps(), 1e-12);
        // kh = 0.063: c = sqrt(g h) (1 - (kh)^2/6 + ...), so the long-wave limit holds to 0.1 %.
        LinearWave shallow = new LinearWave(0.01, 100, 0, 1, 0);
        assertEquals(Math.sqrt(G), shallow.phaseSpeedMps(), Math.sqrt(G) * 1e-3);
        assertTrue(shallow.phaseSpeedMps() < Math.sqrt(G));
    }
    @Test void velocityIsDivergenceAndCurlFreeAndSatisfiesBoundaryConditions() {
        LinearWave wave = new LinearWave(0.3, 12, 0.7, 4, 0.4);
        double still = 2, h = 1e-4;
        Random random = new Random(7);
        for (int i = 0; i < 50; i++) {
            Vec3d p = new Vec3d(random.nextDouble() * 20, still - 0.1 - random.nextDouble() * 3.8, random.nextDouble() * 20);
            double t = random.nextDouble() * 10;
            double divergence = (wave.velocity(p.add(Vec3d.X.scale(h)), still, t).x() - wave.velocity(p.subtract(Vec3d.X.scale(h)), still, t).x()
                    + wave.velocity(p.add(Vec3d.Y.scale(h)), still, t).y() - wave.velocity(p.subtract(Vec3d.Y.scale(h)), still, t).y()
                    + wave.velocity(p.add(Vec3d.Z.scale(h)), still, t).z() - wave.velocity(p.subtract(Vec3d.Z.scale(h)), still, t).z()) / (2 * h);
            // Potential flow: d(u_x)/dy = d(u_y)/dx.
            double curl = (wave.velocity(p.add(Vec3d.Y.scale(h)), still, t).x() - wave.velocity(p.subtract(Vec3d.Y.scale(h)), still, t).x()
                    - wave.velocity(p.add(Vec3d.X.scale(h)), still, t).y() + wave.velocity(p.subtract(Vec3d.X.scale(h)), still, t).y()) / (2 * h);
            assertEquals(0, divergence, 1e-6);
            assertEquals(0, curl, 1e-6);
            // Linearized kinematic surface condition d(eta)/dt = w at the still level.
            double rate = (wave.elevation(p.x(), p.z(), t + h) - wave.elevation(p.x(), p.z(), t - h)) / (2 * h);
            assertEquals(rate, wave.velocity(new Vec3d(p.x(), still, p.z()), still, t).y(), 1e-6);
            // Impermeable flat bottom at depth h.
            assertEquals(0, wave.velocity(new Vec3d(p.x(), still - 4, p.z()), still, t).y(), 1e-12);
        }
    }
    @Test void surfaceSlopeIsTheElevationGradientAndScalingIsLinear() {
        WaveField sea = new WaveField(List.of(new LinearWave(0.3, 12, 0.7, 4, 0.4), new LinearWave(0.1, 5, -1.9, Double.POSITIVE_INFINITY, 2)));
        double h = 1e-5;
        Random random = new Random(19);
        for (int i = 0; i < 40; i++) {
            double x = random.nextDouble() * 30, z = random.nextDouble() * 30, t = random.nextDouble() * 10;
            Vec3d slope = sea.slope(x, z, t);
            assertEquals((sea.elevation(x + h, z, t) - sea.elevation(x - h, z, t)) / (2 * h), slope.x(), 1e-7);
            assertEquals((sea.elevation(x, z + h, t) - sea.elevation(x, z - h, t)) / (2 * h), slope.z(), 1e-7);
            assertEquals(0, slope.y());
            assertEquals(0.4 * sea.elevation(x, z, t), sea.scaled(0.4).elevation(x, z, t), 1e-12);
            assertEquals(0, sea.scaled(0.4).slope(x, z, t).subtract(slope.scale(0.4)).length(), 1e-12);
        }
        assertSame(WaveField.NONE, sea.scaled(0));
        assertEquals(5, sea.shortestWavelength());
        assertThrows(IllegalArgumentException.class, () -> sea.scaled(1.1));
    }
    @Test void accelerationIsTheTimeDerivativeAndObeysLinearEuler() {
        // du/dt from velocity(), and du/dt = -g grad(dynamic pressure head): ties the pressure to the kinematics.
        LinearWave wave = new LinearWave(0.25, 9, -0.6, 3.5, 0.3);
        double still = 1, h = 1e-4;
        Random random = new Random(11);
        for (int i = 0; i < 40; i++) {
            Vec3d p = new Vec3d(random.nextDouble() * 15, still - 0.1 - random.nextDouble() * 3.3, random.nextDouble() * 15);
            double t = random.nextDouble() * 8;
            Vec3d a = wave.acceleration(p, still, t);
            Vec3d dudt = wave.velocity(p, still, t + h).subtract(wave.velocity(p, still, t - h)).scale(1 / (2 * h));
            Vec3d gradient = new Vec3d(
                    wave.dynamicPressureHead(p.add(Vec3d.X.scale(h)), still, t) - wave.dynamicPressureHead(p.subtract(Vec3d.X.scale(h)), still, t),
                    wave.dynamicPressureHead(p.add(Vec3d.Y.scale(h)), still, t) - wave.dynamicPressureHead(p.subtract(Vec3d.Y.scale(h)), still, t),
                    wave.dynamicPressureHead(p.add(Vec3d.Z.scale(h)), still, t) - wave.dynamicPressureHead(p.subtract(Vec3d.Z.scale(h)), still, t)).scale(1 / (2 * h));
            assertEquals(0, a.subtract(dudt).length(), 1e-6);
            assertEquals(0, a.add(gradient.scale(G)).length(), 1e-6);
        }
    }
    @Test void pressureHeadEqualsElevationAtStillLevelAndDecaysWithDepth() {
        LinearWave deep = new LinearWave(0.3, 10, 0, Double.POSITIVE_INFINITY, 0);
        double eta = deep.elevation(2, 0, 1.5);
        assertEquals(eta, deep.dynamicPressureHead(new Vec3d(2, 0, 0), 0, 1.5), 1e-12);
        assertEquals(eta * Math.exp(-2 * Math.PI), deep.dynamicPressureHead(new Vec3d(2, -10, 0), 0, 1.5), 1e-12);
        LinearWave shallow = new LinearWave(0.1, 40, 0, 2, 0);
        // Long waves: pressure is nearly hydrostatic under the surface, K -> 1 at the bottom.
        double bottom = shallow.dynamicPressureHead(new Vec3d(0, -2, 0), 0, 0);
        assertEquals(1 / Math.cosh(2 * Math.PI / 40 * 2), bottom / shallow.elevation(0, 0, 0), 1e-12);
    }
    @Test void submergedSphereFeelsTheMorisonFroudeKrylovForce() {
        // For a small submerged body the vertical pressure force is rho V (g + a_z), so pressureFraction = 1 + a_z/g.
        WaveField waves = new WaveField(List.of(new LinearWave(0.2, 10, 0, Double.POSITIVE_INFINITY, 0)));
        for (double t : new double[]{0, 0.6, 1.3, 2.1}) {
            Vec3d center = new Vec3d(1.7, -1, 0);
            WaveImmersion w = HydroSolver.sphereInWaves(center, 0.05, waves, 0, t, 15);
            assertEquals(1, w.fraction(), 1e-12);
            assertEquals(1 + waves.acceleration(center, 0, t).y() / G, w.pressureFraction(), 2e-3);
        }
        WaveImmersion deep = HydroSolver.sphereInWaves(new Vec3d(0, -30, 0), 0.5, waves, 0, 0, 15);
        assertEquals(1, deep.pressureFraction(), 1e-6);
        WaveImmersion still = HydroSolver.sphereInWaves(new Vec3d(0, 0, 0), 0.5, WaveField.NONE, 0, 0, 15);
        assertEquals(still.fraction(), still.pressureFraction(), 1e-12);
    }
    @Test void halfFloatingSphereHeavesWithAddedMass() {
        // Linear heave: (m + C_a rho V_wet) z'' = -rho g A_wp z. For a half-immersed sphere m = rho V/2 and
        // A_wp = pi R^2, so omega^2 = g/R exactly when C_a = 0.5 (without added mass: 1.5 g/R).
        double radius = 0.5, rho = FluidState.FRESH_WATER_20C.densityKgPerM3(), mass = rho * 2.0 / 3 * Math.PI * Math.pow(radius, 3);
        RigidBodyProperties p = new RigidBodyProperties(mass, Vec3d.ZERO, new InertiaTensor(0.4 * mass * radius * radius,
                0.4 * mass * radius * radius, 0.4 * mass * radius * radius, 0, 0, 0));
        List<BuoyantSphere> floats = List.of(new BuoyantSphere(Vec3d.ZERO, radius));
        RigidBodyState s = new RigidBodyState(new Vec3d(0, 0.005, 0), Vec3d.ZERO, Orientation.IDENTITY, Vec3d.ZERO);
        double dt = 0.002, previous = s.position().y(), firstCrossing = Double.NaN, lastCrossing = Double.NaN;
        int crossings = 0;
        for (int i = 1; i <= 5000; i++) {
            s = FloatingBodyDynamics.step(s, p, floats, TestPlatform.flatWater(0), Vec3d.Y.scale(-G), 0, dt).state();
            double y = s.position().y();
            if (previous > 0 && y <= 0) {
                double time = (i - 1 + previous / (previous - y)) * dt;
                if (crossings == 0) { firstCrossing = time; }
                lastCrossing = time;
                crossings++;
            }
            previous = y;
        }
        assertTrue(crossings >= 5, "Sphere must oscillate: " + crossings);
        double period = (lastCrossing - firstCrossing) / (crossings - 1);
        assertEquals(2 * Math.PI * Math.sqrt(radius / G), period, 0.01 * 2 * Math.PI * Math.sqrt(radius / G));
    }
    @Test void elevationIsPeriodicWithZeroMean() {
        LinearWave wave = new LinearWave(0.25, 8, Math.toRadians(30), 3, 1.1);
        double sum = 0;
        int n = 400;
        for (int i = 0; i < n; i++) {
            double x = 8 * i / (double) n / Math.cos(Math.toRadians(30));
            sum += wave.elevation(x, 0, 0);
        }
        assertEquals(0, sum / n, 1e-12);
        assertEquals(wave.elevation(1.3, -2, 0.4), wave.elevation(1.3, -2, 0.4 + wave.periodS()), 1e-12);
    }
    @Test void rejectsSteepOrBreakingWaves() {
        assertThrows(IllegalArgumentException.class, () -> new LinearWave(0.6, 10, 0, Double.POSITIVE_INFINITY, 0));
        assertThrows(IllegalArgumentException.class, () -> new LinearWave(0.5, 20, 0, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> new LinearWave(0.1, 10, 0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> new LinearWave(-0.1, 10, 0, 5, 0));
    }
    @Test void platformFollowsLongWavesInHeaveAndPitch() {
        // lambda = 60 m >> 1.7 m float spacing and omega = 1.01 rad/s << heave resonance (~6.9 rad/s):
        // quasi-static response, so the heave RAO is ~1 and pitch follows the surface slope A k.
        // Amplitudes are Fourier projections at the wave frequency over whole periods, which rejects the
        // lightly damped free oscillation excited by starting the wave abruptly.
        LinearWave wave = new LinearWave(0.2, 60, 0, Double.POSITIVE_INFINITY, 0);
        WaveField waves = new WaveField(List.of(wave));
        RigidBodyProperties p = TestPlatform.properties(TestPlatform.DEFAULT_BALLAST);
        RigidBodyState s = TestPlatform.atModelOrigin(new Vec3d(0, 0.106, 0), Orientation.IDENTITY, p);
        double dt = 0.005, omega = wave.angularFrequency(), period = wave.periodS();
        int start = (int) Math.round(10 * period / dt), end = (int) Math.round(14 * period / dt);
        double heaveCos = 0, heaveSin = 0, pitchCos = 0, pitchSin = 0;
        for (int i = 0; i < end; i++) {
            s = FloatingBodyDynamics.step(s, p, TestPlatform.FLOATS, TestPlatform.wavyWater(0, waves, i * dt),
                    Vec3d.Y.scale(-G), 0.47, dt).state();
            if (i >= start) {
                double t = (i + 1) * dt, pitch = Math.asin(s.orientation().rotate(Vec3d.Y).x());
                heaveCos += s.position().y() * Math.cos(omega * t); heaveSin += s.position().y() * Math.sin(omega * t);
                pitchCos += pitch * Math.cos(omega * t); pitchSin += pitch * Math.sin(omega * t);
            }
        }
        double scale = 2.0 / (end - start);
        assertEquals(0.2, scale * Math.hypot(heaveCos, heaveSin), 0.01);
        assertEquals(0.2 * wave.wavenumber(), scale * Math.hypot(pitchCos, pitchSin), 0.15 * 0.2 * wave.wavenumber());
    }
}

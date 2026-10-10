package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class UnsteadyWakeTest {
    private static final double G = Atmosphere.GRAVITY_MPS2, RHO = 1000, DT = 0.05;

    @Test void fftMatchesTheDirectTransformAndInverts() {
        int n = 8;
        Fft2D fft = new Fft2D(n);
        Random random = new Random(3);
        double[] re = new double[n * n], im = new double[n * n];
        for (int i = 0; i < n * n; i++) { re[i] = random.nextGaussian(); im[i] = random.nextGaussian(); }
        double[] r0 = re.clone(), i0 = im.clone();
        fft.forward(re, im);
        for (int a = 0; a < n; a++) for (int b = 0; b < n; b++) {
            double sr = 0, si = 0;
            for (int x = 0; x < n; x++) for (int y = 0; y < n; y++) {
                double phase = -2 * Math.PI * (a * x + b * y) / n, c = Math.cos(phase), s = Math.sin(phase);
                sr += r0[x * n + y] * c - i0[x * n + y] * s; si += r0[x * n + y] * s + i0[x * n + y] * c;
            }
            assertEquals(sr, re[a * n + b], 1e-9); assertEquals(si, im[a * n + b], 1e-9);
        }
        fft.inverse(re, im);
        for (int i = 0; i < n * n; i++) { assertEquals(r0[i], re[i], 1e-12); assertEquals(i0[i], im[i], 1e-12); }
    }

    @Test void aBodyAtRestMakesNoWaves() {
        UnsteadyWake wake = new UnsteadyWake(64, 0.25, Double.POSITIVE_INFINITY, RHO, G, DT);
        UnsteadyWake.Load load = new UnsteadyWake.Load(UnsteadyWake.rectangle(2000, -0.7, 0.7, 0.7), 0.3, -0.2, 1, 0, 0);
        wake.reset(0, 0, load);
        for (int i = 0; i < 100; i++) { wake.step(0, 0, load); }
        assertEquals(0, wake.snapshot().maxElevation(), 1e-9);
        assertEquals(0, wake.waveResistance(), 1e-9);
    }

    /** A Gaussian pressure accelerated to U and held there: the wake behind matches the steady Kelvin pattern. */
    @Test void steadyCourseApproachesTheKelvinWake() {
        double speed = 3, a = 0.5, peak = 500, spacing = 0.25, lead = 8;
        UnsteadyWake.Forcing forcing = UnsteadyWake.of(KelvinWake.gaussian(peak, a));
        UnsteadyWake wake = new UnsteadyWake(128, spacing, Double.POSITIVE_INFINITY, RHO, G, DT);
        double x = 0;
        wake.reset(x - lead, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, 0));
        int steps = (int) (40 / DT);
        double resistance = 0;
        int averaged = 0;
        for (int i = 1; i <= steps; i++) {
            double t = i * DT, u = speed * Math.min(1, t / 4);
            x += u * DT;
            wake.step(x - lead, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, u));
            if (t > 30) { resistance += wake.waveResistance(); averaged++; }
        }
        resistance /= averaged;
        KelvinWake steady = KelvinWake.pressure(KelvinWake.gaussian(peak, a), -a, a, a, RHO, speed, G, 0.5, 40);
        double havelock = steady.resistance(RHO);
        UnsteadyWake.Snapshot s = wake.snapshot();
        double sumSq = 0, errSq = 0, max = 0;
        for (double dx = -14; dx <= -4; dx += 0.5) for (double dy = -4; dy <= 4; dy += 0.5) {
            double expected = steady.elevation(dx, dy), actual = s.elevation(x + dx, -dy);
            sumSq += expected * expected; errSq += (actual - expected) * (actual - expected);
            max = Math.max(max, Math.abs(expected));
        }
        double relative = Math.sqrt(errSq / sumSq);
        System.out.printf("Kelvin: R unsteady %.4f N, Havelock %.4f N; pattern rms error %.3f of rms, max %.4f m%n", resistance, havelock, relative, max);
        assertEquals(havelock, resistance, 0.05 * havelock);
        assertTrue(relative < 0.1, "Pattern error " + relative);
        // Nothing wraps around ahead of the bow.
        double ahead = 0;
        for (double dx = 2; dx <= 6; dx += 0.5) for (double dy = -6; dy <= 6; dy += 0.5) { ahead = Math.max(ahead, Math.abs(s.elevation(x + dx, dy))); }
        assertTrue(ahead < 0.05 * max, "Ahead of the bow " + ahead + " vs " + max);
    }

    /** The double-body pressure of a thin ship carries Michell's wave resistance. */
    @Test void thinShipForcingCarriesMichellsResistance() {
        SlenderHull hull = TestHull.HULL;
        SlenderHull.Particulars d = TestHull.model().design();
        double speed = 3.0, spacing = 0.5, lead = 16;
        UnsteadyWake.Forcing forcing = UnsteadyWake.thinShip((xx, depth) -> hull.halfBreadth(xx, depth, d.waterlineY()), d.xAftM(), d.xForwardM(),
                d.draftM(), 60, 12, RHO, 1.2 * Math.PI / spacing);
        UnsteadyWake wake = new UnsteadyWake(128, spacing, Double.POSITIVE_INFINITY, RHO, G, DT);
        double x = 0, resistance = 0;
        int averaged = 0;
        wake.reset(x - lead, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, 0));
        for (int i = 1; i <= (int) (50 / DT); i++) {
            double t = i * DT, u = speed * Math.min(1, t / 6);
            x += u * DT;
            wake.step(x - lead, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, u));
            if (t > 40) { resistance += wake.waveResistance(); averaged++; }
        }
        resistance /= averaged;
        // Michell's Kochin function with the solver's documented forcing filter applied to each direction.
        KelvinWake steady = KelvinWake.thinShip((xx, depth) -> hull.halfBreadth(xx, depth, d.waterlineY()), d.xAftM(), d.xForwardM(),
                d.draftM(), 60, 12, speed, G, 1.5 * spacing, 60);
        double step = 2 * steady.thetaMax() / steady.nodes(), havelock = 0;
        for (int j = 0; j < steady.nodes(); j++) {
            double[] node = steady.node(j);
            double c = Math.cos(node[0]), f = UnsteadyWake.forcingFilter(steady.k0() / (c * c), spacing);
            havelock += (node[1] * node[1] + node[2] * node[2]) * f * f * c * c * c;
        }
        havelock *= Math.PI / 2 * RHO * speed * speed * step;
        UnsteadyWake.Snapshot s = wake.snapshot();
        double sumSq = 0, errSq = 0;
        for (double dx = -24; dx <= -8; dx += 0.5) for (double dy = -6; dy <= 6; dy += 0.5) {
            double expected = 0;
            for (int j = 0; j < steady.nodes(); j++) {
                double[] node = steady.node(j);
                double c = Math.cos(node[0]), k = steady.k0() / (c * c), f = UnsteadyWake.forcingFilter(k, spacing);
                double phase = k * (dx * c + dy * Math.sin(node[0]));
                expected += f * (node[1] * Math.cos(phase) - node[2] * Math.sin(phase)) * step;
            }
            double actual = s.elevation(x + dx, -dy);
            sumSq += expected * expected; errSq += (actual - expected) * (actual - expected);
        }
        double relative = Math.sqrt(errSq / sumSq);
        System.out.printf("Thin ship: R unsteady %.2f N, filtered Michell %.2f N; pattern rms error %.3f%n", resistance, havelock, relative);
        assertEquals(havelock, resistance, 0.05 * havelock);
        assertTrue(relative < 0.1, "Pattern error " + relative);
    }

    /**
     * An accelerating, turning source: the grid following it (Doppler term) and a fixed grid it crosses (moving
     * pressure) give the same wave resistance and wake. The wake is compared at the fixed grid's nodes (where the
     * moving grid is interpolated) on waves resolved by about ten nodes, so interpolation stays small.
     */
    @Test void movingFrameAgreesWithAFixedGrid() {
        double speed = 2, spacing = 0.25;
        UnsteadyWake.Forcing forcing = UnsteadyWake.of(KelvinWake.gaussian(800, 0.5));
        UnsteadyWake fixed = new UnsteadyWake(256, spacing, 6, RHO, G, DT), moving = new UnsteadyWake(128, spacing, 6, RHO, G, DT);
        double x = -16, z = 0, heading = 0, fixedSum = 0, movingSum = 0;
        UnsteadyWake.Load start = new UnsteadyWake.Load(forcing, x, z, 1, 0, 0);
        fixed.reset(0, 0, start); moving.reset(x, z, start);
        for (int i = 1; i <= 240; i++) {
            heading += 0.12 * DT;
            double u = speed * Math.min(1, i * DT / 3);
            x += u * Math.cos(heading) * DT; z += u * Math.sin(heading) * DT;
            UnsteadyWake.Load load = new UnsteadyWake.Load(forcing, x, z, Math.cos(heading), Math.sin(heading), u);
            fixed.step(0, 0, load); moving.step(x, z, load);
            if (i > 160) { fixedSum += fixed.waveResistance(); movingSum += moving.waveResistance(); }
        }
        UnsteadyWake.Snapshot a = fixed.snapshot(), b = moving.snapshot();
        double sumSq = 0, errSq = 0;
        for (double px = Math.floor(x) - 8; px <= x + 2; px += spacing) for (double pz = Math.floor(z) - 6; pz <= z + 6; pz += spacing) {
            double ea = a.elevation(px, pz), eb = b.elevation(px, pz);
            sumSq += ea * ea; errSq += (ea - eb) * (ea - eb);
        }
        double relative = Math.sqrt(errSq / sumSq);
        System.out.printf("Moving vs fixed grid: rms difference %.4f of rms (max %.4f m); mean resistance over the last 4 s %.3f vs %.3f N%n",
                relative, a.maxElevation(), fixedSum / 80, movingSum / 80);
        assertTrue(a.maxElevation() > 1e-3);
        assertEquals(fixedSum, movingSum, 0.03 * Math.abs(fixedSum));
        assertTrue(relative < 0.08, "Difference " + relative);
    }

    /** A stopped source leaves its wake to propagate away rather than carrying it. */
    @Test void stoppingLeavesTheWakeBehind() {
        double spacing = 0.25;
        UnsteadyWake.Forcing forcing = UnsteadyWake.of(KelvinWake.gaussian(500, 0.5));
        UnsteadyWake wake = new UnsteadyWake(128, spacing, Double.POSITIVE_INFINITY, RHO, G, DT);
        double x = 0;
        wake.reset(x, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, 0));
        for (int i = 1; i <= 300; i++) {
            double u = 2.5 * Math.min(1, i * DT / 3);
            x += u * DT;
            wake.step(x - 6, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, u));
        }
        double near = rms(wake.snapshot(), x, 0, 3);
        double centre = x - 6;
        for (int i = 1; i <= 300; i++) { wake.step(centre, 0, new UnsteadyWake.Load(forcing, x, 0, 1, 0, 0)); }
        double after = rms(wake.snapshot(), x, 0, 3);
        System.out.printf("Stopping: rms near the source %.4f m while moving, %.4f m 15 s after stopping%n", near, after);
        assertTrue(near > 0.003);
        assertTrue(after < 0.3 * near, "Waves near a stopped source must run away: " + after + " vs " + near);
    }

    private static double rms(UnsteadyWake.Snapshot s, double x, double z, double radius) {
        double sum = 0;
        int count = 0;
        for (double dx = -radius; dx <= radius; dx += 0.25) for (double dz = -radius; dz <= radius; dz += 0.25) {
            double e = s.elevation(x + dx, z + dz);
            sum += e * e; count++;
        }
        return Math.sqrt(sum / count);
    }
}

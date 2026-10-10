package dev.mcmod.aerocore.api;

import java.util.function.DoubleBinaryOperator;

/**
 * Unsteady linear wake of one moving surface disturbance, integrated in time on a periodic n x n grid that travels with
 * it, so the wake follows the true track: turns curve it, stopping leaves it to run on, starting sends out the starting
 * waves. Linearised potential flow on water of depth h with surface pressure p(x, t):
 * <pre>  eta_t = phi_z,  phi_t + g eta + p / rho = 0  at z = 0,   so per Fourier mode  eta_tt + omega^2 eta = -K p / rho,
 *  K = k tanh(k h),  omega^2 = g K</pre>
 * (Wehausen and Laitone 1960, sec. 20; the forced Cauchy-Poisson problem). Each mode is kept as the complex amplitude
 * Y = FFT(eta + i K phi / omega) + p / (rho g), i.e. measured from the instantaneous hydrostatic depression -p / (rho g),
 * which a body at rest makes without waves. In a frame moving at W the mode obeys
 * <pre>  dY/dt = i (k.W - omega) Y - i (k.W) P / g,   P = FFT(p / rho),</pre>
 * integrated exactly over each step with P held at its mid-step value, so uniform motion has no time-step error and no
 * Courant limit; a change of P between steps enters as the jump (P_new - P_old) / g (eta itself cannot jump). With
 * constant speed the solution tends to the steady Kelvin wake (checked against KelvinWake in UnsteadyWakeTest).
 * <p>
 * Numerical choices, not physics: the domain is periodic, so a sponge 1/8 of the width deep at every edge damps eta
 * and phi (rate up to SPONGE_RATE, ramped as the cube of the depth into the layer); the forcing is filtered by
 * exp(-(k / 0.75 k_N)^8) and modes beyond 0.8 k_N are damped, both against grid-scale noise (k_N = pi / spacing). The
 * output is eta + p / (rho g): under the disturbance it shows the dynamic part, not the hydrostatic dent.
 * <p>
 * Thin ships enter as their linearised double-body pressure p = -rho U d(phi_0)/dx at the waterplane (the rigid-lid
 * potential phi_0 of the Michell centre-plane sources with their image); it has the same Kochin function as
 * KelvinWake.thinShip (derived in the method). Not modelled: nonlinearity and breaking, viscosity, currents, the body's
 * own oscillation (heave and pitch radiation), refraction by depth changes, reflection from shores.
 */
public final class UnsteadyWake {
    /** Damping rate at the outer edge of the sponge, 1/s. */
    public static final double SPONGE_RATE = 6;
    /** Damping rate of the shortest grid waves (at k = sqrt(2) k_N), 1/s. */
    private static final double SHORT_WAVE_RATE = 20;

    /**
     * A disturbance in its body frame: xi forward, zeta to the left of the motion. transform gives the pressure
     * transform p^(alpha, beta) = int int p(xi, zeta) exp(-i (alpha xi + beta zeta)) d xi d zeta (Pa m^2) at forward speed U
     * (m/s) into out {re, im}.
     */
    public interface Forcing {
        void transform(double alpha, double beta, double speed, double[] out);
        Forcing NONE = (alpha, beta, speed, out) -> { out[0] = 0; out[1] = 0; };
    }

    /** One step's disturbance: forcing, world origin of its body frame, forward direction (unit, world x and z), speed. */
    public record Load(Forcing forcing, double originX, double originZ, double forwardX, double forwardZ, double speed) {
        public Load {
            double length = Math.hypot(forwardX, forwardZ);
            if (!(length > 0) || !Double.isFinite(originX) || !Double.isFinite(originZ) || !Double.isFinite(speed)) {
                throw new IllegalArgumentException("Invalid wake load");
            }
            forwardX /= length; forwardZ /= length;
        }
    }

    /** Uniform pressure p0 on [x0, x1] x [-b, b] of the body frame. */
    public static Forcing rectangle(double pressurePa, double x0, double x1, double halfBeam) {
        if (!(x1 > x0) || !(halfBeam > 0)) { throw new IllegalArgumentException("Invalid pressure rectangle"); }
        double length = x1 - x0, centre = (x0 + x1) / 2, total = pressurePa * length * 2 * halfBeam;
        return (alpha, beta, speed, out) -> {
            double magnitude = total * sinc(alpha * length / 2) * sinc(beta * halfBeam);
            out[0] = magnitude * Math.cos(alpha * centre); out[1] = -magnitude * Math.sin(alpha * centre);
        };
    }

    /**
     * Pressure peak (1 - xi^2) along the body, xi from -1 at x0 to 1 at x1, uniform on [-b, b] across: the draft
     * distribution of a flat bottom with rocker. Its transform along x is 4 (sin u - u cos u) / u^3 times the half-length,
     * u = alpha (x1 - x0) / 2.
     */
    public static Forcing parabolic(double peakPa, double x0, double x1, double halfBeam) {
        if (!(x1 > x0) || !(halfBeam > 0)) { throw new IllegalArgumentException("Invalid pressure patch"); }
        double half = (x1 - x0) / 2, centre = (x0 + x1) / 2;
        return (alpha, beta, speed, out) -> {
            double u = alpha * half, along = Math.abs(u) < 1e-3 ? 4.0 / 3 * (1 - u * u / 10) : 4 * (Math.sin(u) - u * Math.cos(u)) / (u * u * u);
            double magnitude = peakPa * half * along * 2 * halfBeam * sinc(beta * halfBeam);
            out[0] = magnitude * Math.cos(alpha * centre); out[1] = -magnitude * Math.sin(alpha * centre);
        };
    }

    /**
     * A forcing acting at a depth instead of the surface: each wave's amplitude reduced by e^{-k depth}, the decay of a
     * linear wave's pressure with depth. Used for a flat bottom at draft T, whose surface-pressure model would otherwise
     * overweight the short waves a submerged hull cannot make.
     */
    public static Forcing submerged(Forcing forcing, double depth) {
        if (!(depth >= 0)) { throw new IllegalArgumentException("Depth must be non-negative"); }
        return (alpha, beta, speed, out) -> {
            forcing.transform(alpha, beta, speed, out);
            double decay = Math.exp(-Math.hypot(alpha, beta) * depth);
            out[0] *= decay; out[1] *= decay;
        };
    }

    /** A KelvinWake pressure transform (which carries the 1 / 4 pi^2 factor) as a forcing. */
    public static Forcing of(KelvinWake.PressureTransform transform) {
        return (alpha, beta, speed, out) -> {
            transform.at(alpha, beta, out);
            out[0] *= 4 * Math.PI * Math.PI; out[1] *= 4 * Math.PI * Math.PI;
        };
    }

    /**
     * Thin ship with half-breadth f(x, depth) on [xAft, xForward] x [0, draft] (as KelvinWake.thinShip). Its Michell
     * sources sigma = -2 U f_x on the centre plane, doubled by their image above z = 0, give at the surface the rigid-lid
     * potential phi_0^ = -(1 / k) int int sigma e^{-k depth} e^{-i alpha x}, using FT[1 / (4 pi r)] = e^{-k |z|} / (2k); its
     * pressure rho phi_0,t = -rho U d(phi_0)/dx is
     * <pre>  p^ = -2 i rho U^2 (alpha / k) Q(alpha, k),   Q = int int f_x e^{-k depth} e^{-i alpha x} dx d depth</pre>
     * (open transom ends counted as jumps of f). Inserted in KelvinWake's pressure formula it returns Michell's
     * A = -(2 k0 / pi) sec^3 theta Q, the thin-ship Kochin function. Q is tabulated on alpha in [-kMax, kMax] and
     * k in [0, sqrt(2) kMax] about the hull's mid-length and interpolated bilinearly.
     */
    public static Forcing thinShip(DoubleBinaryOperator halfBreadth, double xAft, double xForward, double draft, int nx, int nz,
                                   double density, double kMax) {
        Checks.positive(draft, "Draft"); Checks.positive(density, "Density"); Checks.positive(kMax, "Largest wavenumber");
        if (!(xForward > xAft) || nx < 2 || nz < 1) { throw new IllegalArgumentException("Invalid hull grid"); }
        double length = xForward - xAft, centre = (xAft + xForward) / 2;
        double[] xs = new double[nx + 1], zs = new double[nz + 1];
        double[][] f = new double[nz + 1][nx + 1];
        for (int p = 0; p <= nx; p++) { xs[p] = xAft + length * p / nx - centre; }
        for (int m = 0; m <= nz; m++) {
            zs[m] = draft * m / nz;
            for (int p = 0; p <= nx; p++) { f[m][p] = halfBreadth.applyAsDouble(xs[p] + centre, zs[m]); }
        }
        // Linear interpolation of e^{i alpha x} over the table step loses (step L / 2)^2 / 8: 0.5 % at a 0.4 rad phase step.
        double alphaStep = 0.4 / (length / 2), kStep = Math.min(0.05, 0.1 / draft), kTop = Math.sqrt(2) * kMax;
        int na = 2 * (int) Math.ceil(kMax / alphaStep) + 1, nk = (int) Math.ceil(kTop / kStep) + 2;
        double a0 = -(na / 2) * alphaStep;
        // S_m(alpha): the x integral on depth row m.
        double[][] sr = new double[nz + 1][na], si = new double[nz + 1][na];
        double[] er = new double[nx + 1], ei = new double[nx + 1];
        for (int a = 0; a < na; a++) {
            double alpha = a0 + a * alphaStep;
            for (int p = 0; p <= nx; p++) { er[p] = Math.cos(alpha * xs[p]); ei[p] = -Math.sin(alpha * xs[p]); }
            for (int m = 0; m <= nz; m++) {
                double re = f[m][0] * er[0] - f[m][nx] * er[nx], im = f[m][0] * ei[0] - f[m][nx] * ei[nx];
                for (int p = 0; p < nx; p++) {
                    double slope = (f[m][p + 1] - f[m][p]) / (xs[p + 1] - xs[p]);
                    if (slope == 0) { continue; }
                    if (Math.abs(alpha) < 1e-9) { re += slope * (xs[p + 1] - xs[p]); continue; }
                    // int s e^{-i alpha x} dx over the panel = i s (e1 - e0) / alpha.
                    re += slope * -(ei[p + 1] - ei[p]) / alpha; im += slope * (er[p + 1] - er[p]) / alpha;
                }
                sr[m][a] = re; si[m][a] = im;
            }
        }
        float[] qr = new float[na * nk], qi = new float[na * nk];
        double[] weights = new double[nz + 1];
        for (int q = 0; q < nk; q++) {
            depthWeights(zs, q * kStep, weights);
            for (int a = 0; a < na; a++) {
                double re = 0, im = 0;
                for (int m = 0; m <= nz; m++) { re += weights[m] * sr[m][a]; im += weights[m] * si[m][a]; }
                qr[q * na + a] = (float) re; qi[q * na + a] = (float) im;
            }
        }
        return (alpha, beta, speed, out) -> {
            double k = Math.hypot(alpha, beta), ga = (alpha - a0) / alphaStep, gk = k / kStep;
            out[0] = 0; out[1] = 0;
            if (!(k > 1e-9) || ga < 0 || ga >= na - 1 || gk >= nk - 1) { return; }
            int a = (int) ga, q = (int) gk;
            double fa = ga - a, fk = gk - q;
            int i00 = q * na + a, i10 = i00 + na;
            double re = (qr[i00] * (1 - fa) + qr[i00 + 1] * fa) * (1 - fk) + (qr[i10] * (1 - fa) + qr[i10 + 1] * fa) * fk;
            double im = (qi[i00] * (1 - fa) + qi[i00 + 1] * fa) * (1 - fk) + (qi[i10] * (1 - fa) + qi[i10 + 1] * fa) * fk;
            // Back from mid-length to the body origin: Q = e^{-i alpha centre} Q_c; then p^ = -2 i rho U^2 (alpha / k) Q.
            double c = Math.cos(alpha * centre), s = -Math.sin(alpha * centre);
            double qRe = re * c - im * s, qIm = re * s + im * c, scale = -2 * density * speed * speed * alpha / k;
            out[0] = -scale * qIm; out[1] = scale * qRe;
        };
    }

    /** Exact depth weights for int g(z) e^{-K z} dz with g piecewise linear on the nodes (as KelvinWake). */
    private static void depthWeights(double[] zs, double decay, double[] weights) {
        java.util.Arrays.fill(weights, 0);
        for (int m = 0; m + 1 < zs.length; m++) {
            double h = zs[m + 1] - zs[m], kh = decay * h, e0 = Math.exp(-decay * zs[m]), w0, w1;
            if (kh < 1e-4) { w0 = h / 2; w1 = h / 2; }
            else {
                double e = Math.exp(-kh), first = (1 - e) / decay, second = (1 - e * (1 + kh)) / (decay * decay * h);
                w0 = first - second; w1 = second;
            }
            weights[m] += e0 * w0; weights[m + 1] += e0 * w1;
        }
    }

    private static double sinc(double u) { return Math.abs(u) < 1e-8 ? 1 - u * u / 6 : Math.sin(u) / u; }

    /**
     * Wake elevation (eta + p / rho g, m) on the grid corner + (i, j) spacing, index i * n + j (i along world x, j along
     * world z). Bilinear between nodes; zero outside the grid.
     */
    public record Snapshot(double cornerX, double cornerZ, double spacing, int n, float[] height, double maxElevation) {
        public double centreX() { return cornerX + n / 2 * spacing; }
        public double centreZ() { return cornerZ + n / 2 * spacing; }
        /** Half the side of the grid, m. */
        public double halfSize() { return n * spacing / 2; }

        /** Elevation (m) and slopes d/dx, d/dz at world (x, z) into out[0..2] (overwritten). */
        public void sample(double x, double z, double[] out) {
            out[0] = 0; out[1] = 0; out[2] = 0;
            double gi = (x - cornerX) / spacing, gj = (z - cornerZ) / spacing;
            if (!(gi >= 0) || gi >= n - 1 || !(gj >= 0) || gj >= n - 1) { return; }
            int i = (int) gi, j = (int) gj;
            double fi = gi - i, fj = gj - j;
            int p = i * n + j;
            double h00 = height[p], h01 = height[p + 1], h10 = height[p + n], h11 = height[p + n + 1];
            out[0] = (h00 * (1 - fj) + h01 * fj) * (1 - fi) + (h10 * (1 - fj) + h11 * fj) * fi;
            out[1] = ((h10 - h00) * (1 - fj) + (h11 - h01) * fj) / spacing;
            out[2] = ((h01 - h00) * (1 - fi) + (h11 - h10) * fi) / spacing;
        }

        public double elevation(double x, double z) { double[] out = new double[3]; sample(x, z, out); return out[0]; }
    }

    private final int n;
    private final double spacing, density, gravity, dt;
    private final Fft2D fft;
    /** Mode wavenumbers per index (signed; Nyquist index zeroed), angular frequency, filters. */
    private final double[] k1, omega, cosOmega, sinOmega, forcingFilter, shortDamping, sponge;
    private final double[] yr, yi, pr, pi;
    private final double[] wr, wi, ar, ai, br, bi, out = new double[2];
    private double centreX, centreZ;
    private Load last;
    private boolean started;
    private volatile Snapshot snapshot;

    /**
     * n x n grid (power of two) of the given spacing (m) on water of the given depth (m, infinite for deep water),
     * advanced by timeStep (s) per step.
     */
    public UnsteadyWake(int n, double spacing, double depth, double density, double gravity, double timeStep) {
        Checks.positive(spacing, "Spacing"); Checks.positive(density, "Density");
        if (!(depth > 0)) { throw new IllegalArgumentException("Depth must be positive (infinite for deep water)"); }
        Checks.positive(gravity, "Gravity"); Checks.positive(timeStep, "Time step");
        if (n < 16) { throw new IllegalArgumentException("Grid too small"); }
        this.n = n; this.spacing = spacing; this.density = density; this.gravity = gravity; this.dt = timeStep;
        fft = new Fft2D(n);
        k1 = new double[n];
        for (int m = 0; m < n; m++) {
            int signed = m < n / 2 ? m : m - n;
            k1[m] = m == n / 2 ? 0 : 2 * Math.PI * signed / (n * spacing);
        }
        double nyquist = Math.PI / spacing;
        omega = new double[n * n]; cosOmega = new double[n * n]; sinOmega = new double[n * n];
        forcingFilter = new double[n * n]; shortDamping = new double[n * n];
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            int m = i * n + j;
            if (i == n / 2 || j == n / 2) { continue; }
            double k = Math.hypot(k1[i], k1[j]), big = Double.isInfinite(depth) ? 1 : Math.tanh(k * depth);
            omega[m] = Math.sqrt(gravity * k * big);
            cosOmega[m] = Math.cos(omega[m] * timeStep); sinOmega[m] = Math.sin(omega[m] * timeStep);
            forcingFilter[m] = forcingFilter(k, spacing);
            double excess = Math.max(0, (k / nyquist - 0.8) / (Math.sqrt(2) - 0.8));
            shortDamping[m] = Math.exp(-SHORT_WAVE_RATE * excess * excess * timeStep);
        }
        sponge = new double[n * n];
        int width = n / 8;
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            double depthInto = Math.max(edge(i, width), edge(j, width));
            sponge[i * n + j] = Math.exp(-SPONGE_RATE * depthInto * depthInto * depthInto * timeStep);
        }
        yr = new double[n * n]; yi = new double[n * n]; pr = new double[n * n]; pi = new double[n * n];
        wr = new double[n * n]; wi = new double[n * n];
        ar = new double[n]; ai = new double[n]; br = new double[n]; bi = new double[n];
        snapshot = new Snapshot(0, 0, spacing, n, new float[n * n], 0);
    }

    /** The forcing filter exp(-(k / 0.75 k_N)^8) of a grid with this spacing (k_N = pi / spacing). */
    public static double forcingFilter(double k, double spacing) { return Math.exp(-Math.pow(k * spacing / (0.75 * Math.PI), 8)); }

    /** Fraction (0 to 1) of the way into the sponge for index i. */
    private double edge(int i, int width) {
        int fromEdge = Math.min(i, n - 1 - i);
        return fromEdge >= width ? 0 : (width - fromEdge) / (double) width;
    }

    public int size() { return n; }
    public double spacing() { return spacing; }
    public double timeStep() { return dt; }
    public double centreX() { return centreX; }
    public double centreZ() { return centreZ; }
    public boolean started() { return started; }
    /** The latest published elevation; safe to read from any thread. */
    public Snapshot snapshot() { return snapshot; }

    /** Starts at rest: water in hydrostatic balance under the load (null for none), grid centred at (x, z). */
    public void reset(double x, double z, Load load) {
        centreX = x; centreZ = z;
        java.util.Arrays.fill(yr, 0); java.util.Arrays.fill(yi, 0);
        pressure(load, x - n / 2 * spacing, z - n / 2 * spacing);
        last = load;
        started = true;
        snapshot = new Snapshot(x - n / 2 * spacing, z - n / 2 * spacing, spacing, n, new float[n * n], 0);
    }

    /**
     * Advances one step: the grid centre moves to (x, z) during it, and load is the disturbance at the end of the
     * step (null for none). The pressure is held at the mid-step state (origins, directions and speeds averaged with
     * the previous load; the new forcing function).
     */
    public void step(double x, double z, Load load) {
        if (!started) { reset(x, z, load); return; }
        double velocityX = (x - centreX) / dt, velocityZ = (z - centreZ) / dt;
        double cornerX = (x + centreX) / 2 - n / 2 * spacing, cornerZ = (z + centreZ) / 2 - n / 2 * spacing;
        Load mid = load;
        if (load != null && last != null) {
            double fx = load.forwardX() + last.forwardX(), fz = load.forwardZ() + last.forwardZ();
            if (Math.hypot(fx, fz) < 1e-6) { fx = load.forwardX(); fz = load.forwardZ(); }
            mid = new Load(load.forcing(), (load.originX() + last.originX()) / 2, (load.originZ() + last.originZ()) / 2, fx, fz,
                    (load.speed() + last.speed()) / 2);
        }
        // The jump of P since the last step moves Y: Y = Z + P / g with Z continuous.
        double[] oldR = wr, oldI = wi;
        System.arraycopy(pr, 0, oldR, 0, n * n); System.arraycopy(pi, 0, oldI, 0, n * n);
        pressure(mid, cornerX, cornerZ);
        for (int m = 0; m < n * n; m++) { yr[m] += (pr[m] - oldR[m]) / gravity; yi[m] += (pi[m] - oldI[m]) / gravity; }
        // Doppler phases e^{i k.W dt}, separable in the two axes.
        for (int m = 0; m < n; m++) {
            double phase = k1[m] * velocityX * dt; ar[m] = Math.cos(phase); ai[m] = Math.sin(phase);
            phase = k1[m] * velocityZ * dt; br[m] = Math.cos(phase); bi[m] = Math.sin(phase);
        }
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            int m = i * n + j;
            double w = omega[m];
            if (w == 0) { yr[m] = 0; yi[m] = 0; continue; }
            double doppler = k1[i] * velocityX + k1[j] * velocityZ, lambda = doppler - w;
            double dr = ar[i] * br[j] - ai[i] * bi[j], di = ar[i] * bi[j] + ai[i] * br[j];
            double cw = cosOmega[m], sw = sinOmega[m];
            double er = dr * cw + di * sw, ei = di * cw - dr * sw;
            // phi = (e^{i lambda dt} - 1) / (i lambda), by series when lambda dt is small.
            double phr, phi, x1 = lambda * dt;
            if (Math.abs(x1) < 1e-3) { phr = dt * (1 - x1 * x1 / 6); phi = dt * x1 / 2; }
            else { phr = ei / lambda; phi = -(er - 1) / lambda; }
            // Forcing -i doppler P / g.
            double fr = doppler * pi[m] / gravity, fi = -doppler * pr[m] / gravity;
            double nr = er * yr[m] - ei * yi[m] + fr * phr - fi * phi;
            double ni = er * yi[m] + ei * yr[m] + fr * phi + fi * phr;
            yr[m] = nr * shortDamping[m]; yi[m] = ni * shortDamping[m];
        }
        centreX = x; centreZ = z; last = load;
        // Sponge in physical space, then publish.
        fft.inverse(yr, yi);
        float[] height = new float[n * n];
        double max = 0;
        for (int m = 0; m < n * n; m++) {
            yr[m] *= sponge[m]; yi[m] *= sponge[m];
            height[m] = (float) yr[m];
            max = Math.max(max, Math.abs(yr[m]));
        }
        fft.forward(yr, yi);
        snapshot = new Snapshot(x - n / 2 * spacing, z - n / 2 * spacing, spacing, n, height, max);
    }

    /** P = FFT(p / rho) on the grid with the given corner, from the load's body-frame transform. */
    private void pressure(Load load, double cornerX, double cornerZ) {
        if (load == null || load.forcing() == Forcing.NONE) {
            java.util.Arrays.fill(pr, 0); java.util.Arrays.fill(pi, 0);
            return;
        }
        double rx = load.originX() - cornerX, rz = load.originZ() - cornerZ, fx = load.forwardX(), fz = load.forwardZ();
        // Translation e^{-i k.r}, separable.
        for (int m = 0; m < n; m++) {
            ar[m] = Math.cos(k1[m] * rx); ai[m] = -Math.sin(k1[m] * rx);
            br[m] = Math.cos(k1[m] * rz); bi[m] = -Math.sin(k1[m] * rz);
        }
        double scale = 1 / (density * spacing * spacing);
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            int m = i * n + j;
            if (omega[m] == 0 || forcingFilter[m] < 1e-12) { pr[m] = 0; pi[m] = 0; continue; }
            double kx = k1[i], kz = k1[j];
            // Body frame: alpha along forward, beta along (-forwardZ, forwardX).
            load.forcing().transform(kx * fx + kz * fz, -kx * fz + kz * fx, load.speed(), out);
            double tr = ar[i] * br[j] - ai[i] * bi[j], ti = ar[i] * bi[j] + ai[i] * br[j], s = scale * forcingFilter[m];
            pr[m] = s * (out[0] * tr - out[1] * ti); pi[m] = s * (out[0] * ti + out[1] * tr);
        }
    }

    /**
     * Wave resistance of the current load, N: R = int int p d(eta)/d(xi) dA along its forward direction, the force the
     * waves take from it (Havelock's definition), evaluated by Parseval from the spectra.
     */
    public double waveResistance() {
        if (last == null) { return 0; }
        double fx = last.forwardX(), fz = last.forwardZ(), sum = 0;
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) {
            int m = i * n + j, mi = (n - i) % n, mj = (n - j) % n, neg = mi * n + mj;
            // Spectrum of the real eta: (Y(k) + conj(Y(-k))) / 2.
            double hr = (yr[m] + yr[neg]) / 2, hi = (yi[m] - yi[neg]) / 2;
            double kf = k1[i] * fx + k1[j] * fz;
            // conj(P) * i kf * H.
            sum += pr[m] * (-kf * hi) + pi[m] * (kf * hr);
        }
        return density * spacing * spacing * sum / ((double) n * n);
    }
}

package dev.mcmod.aerocore.api;

import java.util.function.DoubleBinaryOperator;

/**
 * Steady far-field Kelvin wake of a disturbance moving at constant speed U along its +x axis over deep, calm water
 * (linearised potential flow). Behind the disturbance the free-wave elevation is
 * <pre>  zeta(x, y) = Re int A(theta) exp(i k(theta) (x cos theta + y sin theta)) d theta,  k(theta) = k0 sec^2 theta,  k0 = g / U^2,</pre>
 * the residue of the linearised steady free-surface condition U^2 phi_xx + g phi_z = U p_x / rho with Rayleigh's
 * radiation condition (Wehausen and Laitone 1960, sec. 13; Newman 1977, sec. 6.18). Its energy flux gives Havelock's
 * (1934) wave resistance R = (pi/2) rho U^2 int |A|^2 cos^3 theta d theta.
 * <ul>
 * <li>Thin ship (Michell): centre-plane sources sigma = -2U f_x of a symmetric hull with half-breadth f(x, depth) give
 * A = -(2 k0 / pi) sec^3 theta Q, Q = int int f_x e^{-k depth} e^{-i k0 sec theta x} dx d depth, so that R equals Michell's
 * integral (checked against HullResistance.michell).</li>
 * <li>Surface pressure p(x, y): A = -i 4 pi k^2 Phat(k cos theta, k sin theta) / (rho g), with
 * Phat(alpha, beta) = (1 / 4 pi^2) int int p e^{-i (alpha x + beta y)} dx dy.</li>
 * </ul>
 * Far field only: the local (non-wave) disturbance beside the body, breaking, viscosity, finite depth and unsteady
 * motion (acceleration, turning) are not represented. Directions are cut where the wavelength 2 pi / k(theta) falls below
 * shortestWavelength (the shortest waves the caller can resolve). Points between the stern and the bow only receive
 * waves from the sources ahead of them; points ahead of the bow have none.
 */
public final class KelvinWake {
    /** Half-angle of the Kelvin wedge, asin(1/3) = 19.47 deg. */
    public static final double CUSP_HALF_ANGLE = Math.asin(1.0 / 3);
    /** Largest phase change between quadrature directions in a synthesised pattern, rad. */
    private static final double PHASE_STEP = 0.3;
    private static final int MAX_NODES = 24000;

    private final double speed, k0, thetaMax, sternX, bowX, halfWidth;
    private final double[] theta;
    /** Amplitude function per longitudinal segment (aft to forward) and node; segmentX are the segment centres. */
    private final double[][] segmentRe, segmentIm;
    private final double[] segmentX, re, im;

    private KelvinWake(double speed, double gravity, double thetaMax, double sternX, double bowX, double halfWidth,
                       double[] theta, double[][] segmentRe, double[][] segmentIm, double[] segmentX) {
        this.speed = speed; this.k0 = gravity / (speed * speed); this.thetaMax = thetaMax;
        this.sternX = sternX; this.bowX = bowX; this.halfWidth = halfWidth;
        this.theta = theta; this.segmentRe = segmentRe; this.segmentIm = segmentIm; this.segmentX = segmentX;
        re = new double[theta.length]; im = new double[theta.length];
        for (int s = 0; s < segmentRe.length; s++) for (int j = 0; j < theta.length; j++) { re[j] += segmentRe[s][j]; im[j] += segmentIm[s][j]; }
    }

    public double speed() { return speed; }
    public double k0() { return k0; }
    /** Transverse wavelength 2 pi U^2 / g, m. */
    public double transverseWavelength() { return 2 * Math.PI / k0; }
    public double thetaMax() { return thetaMax; }
    public double bowX() { return bowX; }
    public double sternX() { return sternX; }
    public boolean isEmpty() { return theta.length == 0; }
    /** Amplitude function A(theta) (m per radian) at node j: {theta, Re A, Im A}. */
    public double[] node(int j) { return new double[]{theta[j], re[j], im[j]}; }
    public int nodes() { return theta.length; }

    /** Largest |theta| for which the wave is at least shortestWavelength long, or NaN when even theta = 0 is shorter. */
    static double cutoff(double k0, double shortestWavelength) {
        double sec2 = 2 * Math.PI / (shortestWavelength * k0);
        return sec2 <= 1 ? Double.NaN : Math.acos(1 / Math.sqrt(sec2));
    }

    /** Node count for a pattern reaching patternLength behind the bow (phase step at most 0.3 rad between nodes). */
    static int nodesFor(double k0, double thetaMax, double reach, double halfWidth) {
        double sec = 1 / Math.cos(thetaMax), tan = Math.tan(thetaMax);
        double rate = k0 * (reach * sec * tan + halfWidth * (sec * sec * sec + sec * tan * tan));
        return (int) Math.clamp(Math.ceil(2 * thetaMax * rate / PHASE_STEP), 64, MAX_NODES);
    }

    /**
     * Thin-ship wake of a symmetric hull with half-breadth f(x, depth) (m) on x in [xAft, xForward] (bow at
     * xForward), depth in [0, draft], sampled on nx x nz panels and integrated as piecewise linear; open transom
     * ends count as sources. The amplitude function is resolved for a pattern patternLength (m) behind the bow.
     */
    public static KelvinWake thinShip(DoubleBinaryOperator halfBreadth, double xAft, double xForward, double draft, int nx, int nz,
                                      double speed, double gravity, double shortestWavelength, double patternLength) {
        Checks.positive(speed, "Speed"); Checks.positive(draft, "Draft"); Checks.positive(gravity, "Gravity");
        Checks.positive(shortestWavelength, "Shortest wavelength"); Checks.positive(patternLength, "Pattern length");
        if (!(xForward > xAft) || nx < 2 || nz < 1) { throw new IllegalArgumentException("Invalid hull grid"); }
        double k0 = gravity / (speed * speed), cut = cutoff(k0, shortestWavelength), length = xForward - xAft;
        double[] xs = new double[nx + 1], zs = new double[nz + 1];
        double[][] f = new double[nz + 1][nx + 1];
        double beam = 0;
        for (int p = 0; p <= nx; p++) { xs[p] = xAft + length * p / nx; }
        for (int m = 0; m <= nz; m++) {
            zs[m] = draft * m / nz;
            for (int p = 0; p <= nx; p++) { f[m][p] = halfBreadth.applyAsDouble(xs[p], zs[m]); beam = Math.max(beam, f[m][p]); }
        }
        int segments = Math.max(1, (int) Math.ceil(length / 0.5));
        double[] centres = new double[segments];
        for (int s = 0; s < segments; s++) { centres[s] = xAft + length * (s + 0.5) / segments; }
        if (Double.isNaN(cut)) { return new KelvinWake(speed, gravity, 0, xAft, xForward, beam, new double[0], new double[segments][0], new double[segments][0], centres); }
        int n = nodesFor(k0, cut, patternLength + length, halfWidth(patternLength + length, beam));
        double[] theta = new double[n];
        double[][] sr = new double[segments][n], si = new double[segments][n];
        double[] er = new double[nx + 1], ei = new double[nx + 1], weights = new double[nz + 1];
        for (int j = 0; j < n; j++) {
            theta[j] = -cut + (j + 0.5) * 2 * cut / n;
            double sec = 1 / Math.cos(theta[j]), kx = k0 * sec, decay = k0 * sec * sec;
            for (int p = 0; p <= nx; p++) { er[p] = Math.cos(kx * xs[p]); ei[p] = -Math.sin(kx * xs[p]); }
            depthWeights(zs, decay, weights);
            double scale = -2 * k0 / Math.PI * sec * sec * sec;
            for (int m = 0; m <= nz; m++) {
                double w = weights[m] * scale;
                if (w == 0) { continue; }
                // Open ends: f jumps from 0 at the stern (+f delta) and back to 0 at the bow (-f delta).
                sr[0][j] += w * f[m][0] * er[0]; si[0][j] += w * f[m][0] * ei[0];
                sr[segments - 1][j] -= w * f[m][nx] * er[nx]; si[segments - 1][j] -= w * f[m][nx] * ei[nx];
                for (int p = 0; p < nx; p++) {
                    double slope = (f[m][p + 1] - f[m][p]) / (xs[p + 1] - xs[p]);
                    if (slope == 0) { continue; }
                    // int s e^{-i kx x} dx over the panel = s (e1 - e0) / (-i kx) = i s (e1 - e0) / kx.
                    double dr = er[p + 1] - er[p], di = ei[p + 1] - ei[p];
                    int segment = Math.min(segments - 1, (int) (((xs[p] + xs[p + 1]) / 2 - xAft) / length * segments));
                    sr[segment][j] += w * slope * (-di) / kx; si[segment][j] += w * slope * dr / kx;
                }
            }
        }
        return new KelvinWake(speed, gravity, cut, xAft, xForward, beam, theta, sr, si, centres);
    }

    /** Pressure distribution transform Phat(alpha, beta) into out {re, im} (Pa m^2, per the 1/4 pi^2 convention). */
    public interface PressureTransform { void at(double alpha, double beta, double[] out); }

    /**
     * Wake of a surface pressure distribution centred near x = centreX, spanning [sternX, bowX] lengthwise and
     * halfBeam to each side, given by its Fourier transform.
     */
    public static KelvinWake pressure(PressureTransform transform, double sternX, double bowX, double halfBeam, double density,
                                      double speed, double gravity, double shortestWavelength, double patternLength) {
        Checks.positive(speed, "Speed"); Checks.positive(density, "Density"); Checks.positive(gravity, "Gravity");
        double k0 = gravity / (speed * speed), cut = cutoff(k0, shortestWavelength), length = bowX - sternX;
        if (!(length > 0)) { throw new IllegalArgumentException("Pressure patch needs bowX > sternX"); }
        double[] centre = {(sternX + bowX) / 2};
        if (Double.isNaN(cut)) { return new KelvinWake(speed, gravity, 0, sternX, bowX, halfBeam, new double[0], new double[1][0], new double[1][0], centre); }
        int n = nodesFor(k0, cut, patternLength + length, halfWidth(patternLength + length, halfBeam));
        double[] theta = new double[n], out = new double[2];
        double[][] sr = new double[1][n], si = new double[1][n];
        for (int j = 0; j < n; j++) {
            theta[j] = -cut + (j + 0.5) * 2 * cut / n;
            double c = Math.cos(theta[j]), k = k0 / (c * c), scale = 4 * Math.PI * k * k / (density * gravity);
            transform.at(k * c, k * Math.sin(theta[j]), out);
            // A = -i scale Phat.
            sr[0][j] = scale * out[1]; si[0][j] = -scale * out[0];
        }
        return new KelvinWake(speed, gravity, cut, sternX, bowX, halfBeam, theta, sr, si, centre);
    }

    /** Axisymmetric Gaussian pressure p0 exp(-r^2 / a^2) centred at the origin: Phat = p0 a^2 / (4 pi) exp(-k^2 a^2 / 4). */
    public static PressureTransform gaussian(double peakPa, double radiusM) {
        return (alpha, beta, out) -> {
            out[0] = peakPa * radiusM * radiusM / (4 * Math.PI) * Math.exp(-(alpha * alpha + beta * beta) * radiusM * radiusM / 4);
            out[1] = 0;
        };
    }

    /** Uniform pressure p0 on the rectangle [x0, x1] x [-b, b]. */
    public static PressureTransform rectangle(double pressurePa, double x0, double x1, double halfBeam) {
        return (alpha, beta, out) -> {
            double length = x1 - x0, centre = (x0 + x1) / 2;
            double magnitude = pressurePa / (4 * Math.PI * Math.PI) * length * 2 * halfBeam * sinc(alpha * length / 2) * sinc(beta * halfBeam);
            out[0] = magnitude * Math.cos(alpha * centre); out[1] = -magnitude * Math.sin(alpha * centre);
        };
    }

    private static double sinc(double u) { return Math.abs(u) < 1e-8 ? 1 - u * u / 6 : Math.sin(u) / u; }

    /** Havelock's wave resistance of the represented waves, N. */
    public double resistance(double density) {
        if (theta.length == 0) { return 0; }
        double sum = 0, step = 2 * thetaMax / theta.length;
        for (int j = 0; j < theta.length; j++) { double c = Math.cos(theta[j]); sum += (re[j] * re[j] + im[j] * im[j]) * c * c * c; }
        return Math.PI / 2 * density * speed * speed * sum * step;
    }

    /** Wave elevation at (x, y) in the body frame (bow along +x), by direct quadrature over every node; m. */
    public double elevation(double x, double y) {
        if (theta.length == 0 || x >= bowX) { return 0; }
        double[] ar = re, ai = im;
        if (x > sternX) {
            ar = new double[theta.length]; ai = new double[theta.length];
            partial(x, ar, ai);
        }
        double sum = 0, step = 2 * thetaMax / theta.length;
        for (int j = 0; j < theta.length; j++) {
            double c = Math.cos(theta[j]), k = k0 / (c * c), phase = k * (x * c + y * Math.sin(theta[j]));
            sum += ar[j] * Math.cos(phase) - ai[j] * Math.sin(phase);
        }
        return sum * step;
    }

    private void partial(double x, double[] ar, double[] ai) {
        for (int s = 0; s < segmentX.length; s++) {
            if (segmentX[s] <= x) { continue; }
            for (int j = 0; j < theta.length; j++) { ar[j] += segmentRe[s][j]; ai[j] += segmentIm[s][j]; }
        }
    }

    /** Half-width of the synthesised strip at distance d behind the bow: the Kelvin wedge plus margin. */
    static double halfWidth(double d, double halfBeam) { return Math.tan(CUSP_HALF_ANGLE + 0.1) * d + halfBeam + 2; }

    /**
     * Elevation on a grid in the body frame: rows every spacing metres behind the bow out to length, columns
     * across the Kelvin wedge. Each row sums the nodes with a stride that keeps the phase step between used nodes
     * at most 0.3 rad for that row, by phasor recurrence across it. The last quarter of the length fades out
     * (smoothstep), a truncation of the pattern rather than a physical decay.
     */
    public Pattern pattern(double length, double spacing) {
        Checks.positive(length, "Pattern length"); Checks.positive(spacing, "Pattern spacing");
        int rows = (int) Math.ceil(length / spacing) + 1;
        double reach = length + (bowX - sternX), maxHalf = halfWidth(reach, halfWidth);
        int half = (int) Math.ceil(maxHalf / spacing), columns = 2 * half + 1;
        float[] height = new float[rows * columns];
        if (theta.length == 0) { return new Pattern(bowX, spacing, rows, half, height); }
        double step = 2 * thetaMax / theta.length, sec = 1 / Math.cos(thetaMax), tan = Math.tan(thetaMax);
        double[] ar = new double[theta.length], ai = new double[theta.length];
        double[] row = new double[columns];
        for (int i = 1; i < rows; i++) {
            double x = bowX - i * spacing, d = i * spacing;
            double[] sr = re, si = im;
            if (x > sternX) {
                java.util.Arrays.fill(ar, 0); java.util.Arrays.fill(ai, 0);
                partial(x, ar, ai); sr = ar; si = ai;
            }
            double rowHalf = Math.min(maxHalf, halfWidth(d + (bowX - sternX), halfWidth));
            int used = Math.min(half, (int) Math.ceil(rowHalf / spacing));
            double rate = k0 * ((d + bowX - sternX) * sec * tan + rowHalf * (sec * sec * sec + sec * tan * tan));
            int stride = Math.max(1, (int) Math.floor(PHASE_STEP / rate / step));
            java.util.Arrays.fill(row, 0);
            double y0 = -used * spacing;
            for (int j = stride / 2; j < theta.length; j += stride) {
                double c = Math.cos(theta[j]), s = Math.sin(theta[j]), k = k0 / (c * c), w = stride * step;
                double phase = k * (x * c + y0 * s), cr = Math.cos(phase), ci = Math.sin(phase);
                double vr = w * (sr[j] * cr - si[j] * ci), vi = w * (sr[j] * ci + si[j] * cr);
                double stepR = Math.cos(k * s * spacing), stepI = Math.sin(k * s * spacing);
                for (int q = half - used; q <= half + used; q++) {
                    row[q] += vr;
                    double next = vr * stepR - vi * stepI;
                    vi = vr * stepI + vi * stepR; vr = next;
                }
            }
            double fade = d < 0.75 * length ? 1 : smoothstep((length - d) / (0.25 * length));
            for (int q = 0; q < columns; q++) { height[i * columns + q] = (float) (row[q] * fade); }
        }
        return new Pattern(bowX, spacing, rows, half, height);
    }

    private static double smoothstep(double s) { s = Math.clamp(s, 0, 1); return s * s * (3 - 2 * s); }

    /**
     * Exact linear depth weights for int g(z) e^{-K z} dz with g piecewise linear on the depth nodes (as in
     * HullResistance): weights[m] multiplies g(z_m).
     */
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
        if (zs.length == 1) { weights[0] = 0; }
    }

    /**
     * Wake elevation on a regular grid in the body frame: row i lies i spacing behind the bow, column q at lateral
     * offset (q - half) spacing. Bilinear between nodes; zero ahead of the bow, beside the strip and past its end.
     */
    public static final class Pattern {
        private final double bowX, spacing;
        private final int rows, half, columns;
        private final float[] height;
        private final double maxElevation;

        Pattern(double bowX, double spacing, int rows, int half, float[] height) {
            this.bowX = bowX; this.spacing = spacing; this.rows = rows; this.half = half; this.columns = 2 * half + 1; this.height = height;
            double max = 0;
            for (float h : height) { max = Math.max(max, Math.abs(h)); }
            maxElevation = max;
        }

        public double spacing() { return spacing; }
        public double length() { return (rows - 1) * spacing; }
        public double halfWidth() { return half * spacing; }
        public double maxElevation() { return maxElevation; }
        public double bowX() { return bowX; }

        /** Elevation (m) and slopes d/dx, d/dy at (x, y) in the body frame into out[0..2]. */
        public void sample(double x, double y, double[] out) {
            out[0] = 0; out[1] = 0; out[2] = 0;
            double gi = (bowX - x) / spacing, gq = y / spacing + half;
            if (!(gi >= 0) || gi >= rows - 1 || !(gq >= 0) || gq >= columns - 1) { return; }
            int i = (int) gi, q = (int) gq;
            double fi = gi - i, fq = gq - q;
            double h00 = height[i * columns + q], h01 = height[i * columns + q + 1], h10 = height[(i + 1) * columns + q], h11 = height[(i + 1) * columns + q + 1];
            out[0] = (h00 * (1 - fq) + h01 * fq) * (1 - fi) + (h10 * (1 - fq) + h11 * fq) * fi;
            // Row index grows toward -x.
            out[1] = -((h10 - h00) * (1 - fq) + (h11 - h01) * fq) / spacing;
            out[2] = ((h01 - h00) * (1 - fi) + (h11 - h10) * fi) / spacing;
        }

        public double elevation(double x, double y) { double[] out = new double[3]; sample(x, y, out); return out[0]; }
    }
}

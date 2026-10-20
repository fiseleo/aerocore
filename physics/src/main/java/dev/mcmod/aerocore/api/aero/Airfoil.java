package dev.mcmod.aerocore.api.aero;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * An airfoil's shape, chord 1 from the leading edge (0, 0) to the trailing edge (1, 0): its upper and lower surfaces
 * as heights at stations x (cosine-spaced from 0 to 1, so the leading edge is finely resolved). Made from
 * <ul>
 * <li>the NACA four-digit family (Abbott and von Doenhoff, Theory of Wing Sections, section 6.4): maximum camber m at
 *   chordwise position p, thickness t, the thickness laid perpendicular to the camber line; the trailing edge closed
 *   (the last coefficient -0.1036 instead of -0.1015);</li>
 * <li>Kulfan's class-shape transformation (CST, Kulfan and Bussoletti, AIAA 2006-6948): each surface
 *   y = sqrt(x) (1 - x) sum A_i B_i(x) + x dz, Bernstein polynomials B_i of the order the weights give, a trailing-edge
 *   thickness dz shared by the two surfaces;</li>
 * <li>or points (x, y) in the Selig order: from the trailing edge over the upper surface to the leading edge and back
 *   along the lower one.</li>
 * </ul>
 * Records compare by value, so analyses can be cached by shape.
 */
public record Airfoil(double[] x, double[] upper, double[] lower) {
    /** Stations the analytic shapes are sampled at. */
    public static final int STATIONS = 161;

    public Airfoil {
        Objects.requireNonNull(x, "x"); Objects.requireNonNull(upper, "upper"); Objects.requireNonNull(lower, "lower");
        if (x.length < 5 || upper.length != x.length || lower.length != x.length) { throw new IllegalArgumentException("Need matching stations"); }
        x = x.clone(); upper = upper.clone(); lower = lower.clone();
        for (int i = 1; i < x.length; i++) { if (!(x[i] > x[i - 1])) { throw new IllegalArgumentException("Stations must increase"); } }
        if (Math.abs(x[0]) > 1e-9 || Math.abs(x[x.length - 1] - 1) > 1e-9) { throw new IllegalArgumentException("Stations must run from 0 to 1"); }
        for (int i = 1; i < x.length - 1; i++) {
            if (!(upper[i] > lower[i])) { throw new IllegalArgumentException("The upper surface must lie above the lower one"); }
        }
    }

    @Override public boolean equals(Object o) {
        return o instanceof Airfoil a && Arrays.equals(x, a.x) && Arrays.equals(upper, a.upper) && Arrays.equals(lower, a.lower);
    }
    @Override public int hashCode() { return 31 * (31 * Arrays.hashCode(x) + Arrays.hashCode(upper)) + Arrays.hashCode(lower); }
    @Override public String toString() { return "Airfoil[t/c " + thickness() + ", camber " + camber() + "]"; }

    /** Cosine-spaced stations from 0 to 1. */
    static double[] stations(int n) {
        double[] s = new double[n];
        for (int i = 0; i < n; i++) { s[i] = 0.5 * (1 - Math.cos(Math.PI * i / (n - 1))); }
        return s;
    }

    /**
     * A NACA four-digit section: maximum camber (fraction of chord) at a chordwise position (fraction), thickness
     * (fraction). NACA 2412 is naca4(0.02, 0.4, 0.12).
     */
    public static Airfoil naca4(double camber, double camberPosition, double thickness) {
        if (!(thickness > 0 && thickness < 0.5)) { throw new IllegalArgumentException("Thickness must be in (0, 0.5)"); }
        if (!(camber >= 0 && camber < 0.2)) { throw new IllegalArgumentException("Camber must be in [0, 0.2)"); }
        if (camber > 0 && !(camberPosition > 0.05 && camberPosition < 0.95)) { throw new IllegalArgumentException("Camber position must be in (0.05, 0.95)"); }
        int n = STATIONS * 2;
        double[] s = stations(n), xu = new double[n], yu = new double[n], xl = new double[n], yl = new double[n];
        for (int i = 0; i < n; i++) {
            double xi = s[i], yt = 5 * thickness * (0.2969 * Math.sqrt(xi) - 0.1260 * xi - 0.3516 * xi * xi + 0.2843 * xi * xi * xi - 0.1036 * xi * xi * xi * xi);
            double yc = 0, slope = 0;
            if (camber > 0) {
                double m = camber, p = camberPosition;
                if (xi < p) { yc = m / (p * p) * (2 * p * xi - xi * xi); slope = 2 * m / (p * p) * (p - xi); }
                else { yc = m / ((1 - p) * (1 - p)) * (1 - 2 * p + 2 * p * xi - xi * xi); slope = 2 * m / ((1 - p) * (1 - p)) * (p - xi); }
            }
            double th = Math.atan(slope);
            xu[i] = xi - yt * Math.sin(th); yu[i] = yc + yt * Math.cos(th);
            xl[i] = xi + yt * Math.sin(th); yl[i] = yc - yt * Math.cos(th);
        }
        return resampled(xu, yu, xl, yl);
    }

    /** A CST section: weights of the upper and lower surfaces (the lower one's are usually negative) and the trailing-edge thickness (fraction of chord). */
    public static Airfoil cst(double[] upperWeights, double[] lowerWeights, double trailingEdgeThickness) {
        if (upperWeights.length == 0 || lowerWeights.length == 0) { throw new IllegalArgumentException("Each surface needs a weight"); }
        double[] s = stations(STATIONS), yu = new double[STATIONS], yl = new double[STATIONS];
        for (int i = 0; i < STATIONS; i++) {
            yu[i] = cstSurface(upperWeights, s[i]) + s[i] * trailingEdgeThickness / 2;
            yl[i] = cstSurface(lowerWeights, s[i]) - s[i] * trailingEdgeThickness / 2;
        }
        yu[0] = yl[0] = 0;
        return new Airfoil(s, yu, yl);
    }

    private static double cstSurface(double[] w, double x) {
        int n = w.length - 1;
        double sum = 0;
        for (int i = 0; i <= n; i++) { sum += w[i] * binomial(n, i) * Math.pow(x, i) * Math.pow(1 - x, n - i); }
        return Math.sqrt(x) * (1 - x) * sum;
    }
    private static double binomial(int n, int k) {
        double r = 1;
        for (int i = 1; i <= k; i++) { r = r * (n - k + i) / i; }
        return r;
    }

    /** CST weights (order n) that fit a NACA four-digit section by least squares on both surfaces: a start for a CST design. */
    public static double[][] cstFit(Airfoil a, int order) {
        double[][] out = new double[2][];
        for (int side = 0; side < 2; side++) {
            int m = order + 1;
            double[][] ata = new double[m][m];
            double[] atb = new double[m];
            for (int i = 1; i < a.x.length - 1; i++) {
                double xi = a.x[i], y = side == 0 ? a.upper[i] : a.lower[i], c = Math.sqrt(xi) * (1 - xi);
                double[] row = new double[m];
                for (int k = 0; k <= order; k++) { row[k] = c * binomial(order, k) * Math.pow(xi, k) * Math.pow(1 - xi, order - k); }
                for (int p = 0; p < m; p++) { atb[p] += row[p] * y; for (int q = 0; q < m; q++) { ata[p][q] += row[p] * row[q]; } }
            }
            out[side] = solve(ata, atb);
        }
        return out;
    }

    private static double[] solve(double[][] a, double[] b) {
        int n = b.length;
        double[][] m = new double[n][];
        for (int i = 0; i < n; i++) { m[i] = a[i].clone(); }
        double[] x = b.clone();
        for (int c = 0; c < n; c++) {
            int p = c;
            for (int i = c + 1; i < n; i++) { if (Math.abs(m[i][c]) > Math.abs(m[p][c])) { p = i; } }
            double[] t = m[c]; m[c] = m[p]; m[p] = t; double tb = x[c]; x[c] = x[p]; x[p] = tb;
            for (int i = c + 1; i < n; i++) { double f = m[i][c] / m[c][c]; for (int j = c; j < n; j++) { m[i][j] -= f * m[c][j]; } x[i] -= f * x[c]; }
        }
        for (int i = n - 1; i >= 0; i--) { for (int j = i + 1; j < n; j++) { x[i] -= m[i][j] * x[j]; } x[i] /= m[i][i]; }
        return x;
    }

    /** A section from Selig-ordered points (trailing edge, upper surface, leading edge, lower surface, trailing edge), chord normalised. */
    public static Airfoil of(List<double[]> points) {
        if (points.size() < 10) { throw new IllegalArgumentException("Need at least 10 points"); }
        int le = 0;
        for (int i = 1; i < points.size(); i++) { if (points.get(i)[0] < points.get(le)[0]) { le = i; } }
        double x0 = points.get(le)[0], y0 = points.get(le)[1], chord = Math.max(points.getFirst()[0], points.getLast()[0]) - x0;
        if (!(chord > 0)) { throw new IllegalArgumentException("No chord"); }
        int nu = le + 1, nl = points.size() - le;
        double[] xu = new double[nu], yu = new double[nu], xl = new double[nl], yl = new double[nl];
        for (int i = 0; i < nu; i++) { double[] p = points.get(le - i); xu[i] = (p[0] - x0) / chord; yu[i] = (p[1] - y0) / chord; }
        for (int i = 0; i < nl; i++) { double[] p = points.get(le + i); xl[i] = (p[0] - x0) / chord; yl[i] = (p[1] - y0) / chord; }
        return resampled(xu, yu, xl, yl);
    }

    /** Both surfaces, each given from the leading edge aft, resampled at common cosine stations. */
    private static Airfoil resampled(double[] xu, double[] yu, double[] xl, double[] yl) {
        double[] s = stations(STATIONS), u = new double[STATIONS], l = new double[STATIONS];
        for (int i = 0; i < STATIONS; i++) { u[i] = interpolate(xu, yu, s[i]); l[i] = interpolate(xl, yl, s[i]); }
        double y0 = 0.5 * (u[0] + l[0]);
        u[0] = l[0] = y0;
        return new Airfoil(s, u, l);
    }

    private static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) { return ys[0]; }
        for (int i = 1; i < xs.length; i++) {
            if (x <= xs[i]) { double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]); return ys[i - 1] + t * (ys[i] - ys[i - 1]); }
        }
        return ys[ys.length - 1];
    }

    /** Height of the upper or lower surface at a chordwise station. */
    public double upperAt(double xc) { return interpolate(x, upper, xc); }
    public double lowerAt(double xc) { return interpolate(x, lower, xc); }

    /** Largest thickness over chord. */
    public double thickness() {
        double t = 0;
        for (int i = 0; i < x.length; i++) { t = Math.max(t, upper[i] - lower[i]); }
        return t;
    }
    /** Chordwise position of the largest thickness. */
    public double thicknessPosition() {
        int k = 0;
        for (int i = 0; i < x.length; i++) { if (upper[i] - lower[i] > upper[k] - lower[k]) { k = i; } }
        return x[k];
    }
    /** Largest camber (mean line height) over chord. */
    public double camber() {
        double c = 0;
        for (int i = 0; i < x.length; i++) { double m = 0.5 * (upper[i] + lower[i]); if (Math.abs(m) > Math.abs(c)) { c = m; } }
        return c;
    }
    /** Cross-section area over chord squared. */
    public double area() {
        double a = 0;
        for (int i = 1; i < x.length; i++) { a += 0.5 * ((upper[i] - lower[i]) + (upper[i - 1] - lower[i - 1])) * (x[i] - x[i - 1]); }
        return a;
    }

    /**
     * Panel nodes for the analysis, clockwise from the trailing edge: along the lower surface to the leading edge, then
     * the upper one back (n panels, n even, cosine-spaced on each surface). The trailing edge is closed at its mean height.
     */
    double[][] nodes(int panels) {
        int half = panels / 2;
        double[] s = stations(half + 1);
        double[][] p = new double[panels + 1][];
        double te = 0.5 * (upper[upper.length - 1] + lower[lower.length - 1]);
        for (int i = 0; i <= half; i++) {
            double xi = s[half - i];
            p[i] = new double[]{xi, i == 0 ? te : lowerAt(xi)};
        }
        for (int i = 1; i <= half; i++) {
            double xi = s[i];
            p[half + i] = new double[]{xi, i == half ? te : upperAt(xi)};
        }
        return p;
    }
}

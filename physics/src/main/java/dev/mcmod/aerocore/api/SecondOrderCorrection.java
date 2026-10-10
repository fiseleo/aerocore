package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The full second-order surface of a directional sea (Sharma and Dean 1981, as given by Forristall 2000, eqs. 11-18)
 * minus its Choppy-Wave-Model part that CompiledWaves evaluates cheaply:
 * <pre>  eta2 = (1/4) sum_i sum_j a_i a_j [K+_ij cos(psi_i + psi_j) + K-_ij cos(psi_i - psi_j)]</pre>
 * K+ (crests sharpened, troughs flattened) and K- (set-down under wave groups) depend on both wave vectors and the depth.
 * The model's own kernels are cos(dtheta) (k_j coth_i + k_i coth_j) for K+ and minus that for K- (zero on the diagonal
 * with the mean-level term). The difference C+- = a_i a_j (K+-_SD - K+-_CWM) / 4 is written as c'Pc + s'Qs with
 * c, s the cosines and sines of the phases, P = C+ + C-, Q = C- - C+, and both symmetric matrices are replaced by
 * their leading eigenpairs (Jacobi), dropping at most 1 % of their Frobenius norm. For one deep-water wave and for
 * collinear deep-water pairs the correction vanishes on K+ and restores Longuet-Higgins' -|k_i - k_j| on K-.
 * The diagonal of K- is zero (a uniform wave train keeps the still level). Decompositions are cached by the sea's wave
 * vectors and depth; a copy with other amplitudes (tapered, or changed by the coast) reuses it by folding the ratio of
 * its amplitudes to the ones decomposed into the cosines and sines, which is exact because C is a_i a_j times a
 * kernel.
 */
final class SecondOrderCorrection {
    static final SecondOrderCorrection NONE = new SecondOrderCorrection(new double[0][], new double[0], new double[0][], new double[0], new double[0]);
    private static final double DROPPED_FROBENIUS = 1e-4;
    private static final int CACHE = 64;
    private static final Map<Key, SecondOrderCorrection> CACHED = Collections.synchronizedMap(new LinkedHashMap<>(16, 0.75f, true) {
        @Override protected boolean removeEldestEntry(Map.Entry<Key, SecondOrderCorrection> eldest) { return size() > CACHE; }
    });

    /** Modes (rows, one value per component) and weights, for unit largest amplitude; scale weights by aMax^2. */
    final double[][] cosModes, sinModes;
    final double[] cosWeights, sinWeights;
    /** The amplitudes the decomposition was built with (relative to their largest). */
    final double[] amplitudes;

    private SecondOrderCorrection(double[][] cosModes, double[] cosWeights, double[][] sinModes, double[] sinWeights, double[] amplitudes) {
        this.cosModes = cosModes; this.cosWeights = cosWeights; this.sinModes = sinModes; this.sinWeights = sinWeights; this.amplitudes = amplitudes;
    }

    /** Factors a_i / (the amplitude decomposed) to fold into the phases for a field of these amplitudes. */
    double[] fold(double[] amplitude) {
        double[] f = new double[amplitude.length];
        for (int i = 0; i < f.length && i < amplitudes.length; i++) { f[i] = amplitudes[i] > 0 ? amplitude[i] / amplitudes[i] : 0; }
        return f;
    }

    int rank() { return cosModes.length + sinModes.length; }

    private record Key(long[] values) {
        @Override public boolean equals(Object o) { return o instanceof Key other && Arrays.equals(values, other.values); }
        @Override public int hashCode() { return Arrays.hashCode(values); }
    }

    /** The correction for these components, with amplitudes relative to their largest one (aMax > 0). */
    static SecondOrderCorrection of(double[] kx, double[] kz, double[] relativeAmplitude, double[] depth, double[] coth) {
        int n = kx.length;
        if (n == 0) { return NONE; }
        long[] key = new long[3 * n];
        for (int i = 0; i < n; i++) {
            key[3 * i] = Math.round(kx[i] * 1e9); key[3 * i + 1] = Math.round(kz[i] * 1e9);
            key[3 * i + 2] = Double.isFinite(depth[i]) ? Math.round(depth[i] * 1e6) : Long.MAX_VALUE;
        }
        Key k = new Key(key);
        SecondOrderCorrection cached = CACHED.get(k);
        if (cached != null) { return cached; }
        SecondOrderCorrection built = build(kx, kz, relativeAmplitude, depth, coth);
        CACHED.put(k, built);
        return built;
    }

    private static SecondOrderCorrection build(double[] kx, double[] kz, double[] a, double[] depth, double[] coth) {
        int n = kx.length;
        double[][] p = new double[n][n], q = new double[n][n];
        for (int i = 0; i < n; i++) for (int j = i; j < n; j++) {
            double[] sd = kernels(kx[i], kz[i], kx[j], kz[j], depth[i]);
            double ki = Math.hypot(kx[i], kz[i]), kj = Math.hypot(kx[j], kz[j]);
            double cosDelta = (kx[i] * kx[j] + kz[i] * kz[j]) / (ki * kj);
            double plusModel = cosDelta * (kj * coth[i] + ki * coth[j]), minusModel = i == j ? 0 : -plusModel;
            double cPlus = a[i] * a[j] * (sd[0] - plusModel) / 4, cMinus = a[i] * a[j] * (sd[1] - minusModel) / 4;
            p[i][j] = p[j][i] = cPlus + cMinus;
            q[i][j] = q[j][i] = cMinus - cPlus;
        }
        List<double[]> cosModes = new ArrayList<>(), sinModes = new ArrayList<>();
        List<Double> cosWeights = new ArrayList<>(), sinWeights = new ArrayList<>();
        truncate(p, cosModes, cosWeights);
        truncate(q, sinModes, sinWeights);
        return new SecondOrderCorrection(cosModes.toArray(new double[0][]), cosWeights.stream().mapToDouble(Double::doubleValue).toArray(),
                sinModes.toArray(new double[0][]), sinWeights.stream().mapToDouble(Double::doubleValue).toArray(), a.clone());
    }

    /**
     * Sharma and Dean's kernels {K+, K-} for components with wave vectors (kxi, kzi), (kxj, kzj) in water of the given
     * depth (infinite for deep water), with R = k tanh(kh) = omega^2 / g. K- of a component with itself is 0.
     */
    static double[] kernels(double kxi, double kzi, double kxj, double kzj, double depth) {
        double ki = Math.hypot(kxi, kzi), kj = Math.hypot(kxj, kzj), dot = kxi * kxj + kzi * kzj;
        double ri = ki * tanh(ki, depth), rj = kj * tanh(kj, depth), si = Math.sqrt(ri), sj = Math.sqrt(rj);
        double kp = Math.hypot(kxi + kxj, kzi + kzj), km = Math.hypot(kxi - kxj, kzi - kzj);
        double dp = ((si + sj) * (si * (kj * kj - rj * rj) + sj * (ki * ki - ri * ri)) + 2 * (si + sj) * (si + sj) * (dot - ri * rj))
                / ((si + sj) * (si + sj) - kp * tanh(kp, depth));
        double plus = (dp - (dot - ri * rj)) / Math.sqrt(ri * rj) + ri + rj;
        double minus;
        if (km < 1e-12 * (ki + kj)) { minus = 0; }
        else {
            double numerator = (si - sj) * (sj * (ki * ki - ri * ri) - si * (kj * kj - rj * rj)) + 2 * (si - sj) * (si - sj) * (dot + ri * rj);
            double denominator = (si - sj) * (si - sj) - km * tanh(km, depth);
            double dm = numerator == 0 ? 0 : numerator / denominator;
            minus = (dm - (dot + ri * rj)) / Math.sqrt(ri * rj) + ri + rj;
        }
        return new double[]{plus, minus};
    }

    private static double tanh(double k, double depth) { return Double.isFinite(depth) ? Math.tanh(k * depth) : 1; }

    /** Leading eigenpairs of a symmetric matrix holding all but DROPPED_FROBENIUS of its squared Frobenius norm. */
    private static void truncate(double[][] matrix, List<double[]> modes, List<Double> weights) {
        int n = matrix.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) { a[i] = matrix[i].clone(); }
        double[][] v = new double[n][n];
        for (int i = 0; i < n; i++) { v[i][i] = 1; }
        jacobi(a, v);
        Integer[] order = new Integer[n];
        double total = 0;
        for (int i = 0; i < n; i++) { order[i] = i; total += a[i][i] * a[i][i]; }
        Arrays.sort(order, (x, y) -> Double.compare(Math.abs(a[y][y]), Math.abs(a[x][x])));
        double kept = 0;
        for (int r = 0; r < n && kept < (1 - DROPPED_FROBENIUS) * total; r++) {
            int e = order[r];
            if (a[e][e] == 0) { break; }
            double[] mode = new double[n];
            for (int i = 0; i < n; i++) { mode[i] = v[i][e]; }
            modes.add(mode);
            weights.add(a[e][e]);
            kept += a[e][e] * a[e][e];
        }
    }

    /** Cyclic Jacobi rotations: a becomes diagonal (eigenvalues), the columns of v the eigenvectors. */
    private static void jacobi(double[][] a, double[][] v) {
        int n = a.length;
        for (int sweep = 0; sweep < 60; sweep++) {
            double off = 0, diag = 0;
            for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) { if (i != j) { off += a[i][j] * a[i][j]; } else { diag += a[i][i] * a[i][i]; } }
            if (off <= 1e-24 * (diag + off) || off == 0) { return; }
            for (int p = 0; p < n - 1; p++) for (int q = p + 1; q < n; q++) {
                if (a[p][q] == 0) { continue; }
                double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                double t = Math.signum(theta) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
                if (theta == 0) { t = 1; }
                double c = 1 / Math.sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < n; k++) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < n; k++) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < n; k++) {
                    double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }

    /** Adds the correction and its slopes at phases (offset), with fold factors (fold), into out[0..2]. */
    void add(double[] cos, double[] sin, int offset, double[] fold, double[] kx, double[] kz, double[] out) {
        int n = kx.length;
        for (int r = 0; r < cosModes.length; r++) {
            double[] u = cosModes[r];
            double alpha = 0, ax = 0, az = 0;
            for (int i = 0; i < n; i++) { double uf = u[i] * fold[i], s = sin[offset + i]; alpha += uf * cos[offset + i]; ax -= uf * kx[i] * s; az -= uf * kz[i] * s; }
            double w = cosWeights[r];
            out[0] += w * alpha * alpha; out[1] += 2 * w * alpha * ax; out[2] += 2 * w * alpha * az;
        }
        for (int r = 0; r < sinModes.length; r++) {
            double[] u = sinModes[r];
            double beta = 0, bx = 0, bz = 0;
            for (int i = 0; i < n; i++) { double uf = u[i] * fold[i], c = cos[offset + i]; beta += uf * sin[offset + i]; bx += uf * kx[i] * c; bz += uf * kz[i] * c; }
            double w = sinWeights[r];
            out[0] += w * beta * beta; out[1] += 2 * w * beta * bx; out[2] += 2 * w * beta * bz;
        }
    }

    /** The correction's height alone, with fold factors. */
    double height(double[] cos, double[] sin, int offset, double[] fold) {
        double sum = 0;
        for (int r = 0; r < cosModes.length; r++) {
            double alpha = 0;
            for (int i = 0; i < cosModes[r].length; i++) { alpha += cosModes[r][i] * fold[i] * cos[offset + i]; }
            sum += cosWeights[r] * alpha * alpha;
        }
        for (int r = 0; r < sinModes.length; r++) {
            double beta = 0;
            for (int i = 0; i < sinModes[r].length; i++) { beta += sinModes[r][i] * fold[i] * sin[offset + i]; }
            sum += sinWeights[r] * beta * beta;
        }
        return sum;
    }
}

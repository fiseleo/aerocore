package dev.mcmod.aerocore.api.frame;

import java.util.Arrays;
import java.util.Comparator;

/**
 * A frame's stiffness, assembled and factored (0.33). The nodes are numbered in order of x (ties by index), which keeps a
 * vehicle frame's freedoms near the diagonal; the matrix is factored by banded Cholesky (n b² / 2 multiply-adds for n
 * freedoms and half-bandwidth b), then any number of load vectors are solved against it (2 n b each). Springs to the
 * ground add to the diagonal; fixed freedoms are taken out (row and column cleared, their loads left out).
 * <p>Freedoms are 6 per node by node index: displacement along x, y, z (m), rotation about x, y, z (rad); loads the same
 * (N, N m), in the frame's axes. The frame must be one piece (else FrameDisconnectedException names the first node off
 * the largest piece) and held against every rigid-body motion: a pivot below 1e-11 of its diagonal is taken as a free
 * motion and also named as a node not joined.
 */
public final class FrameSolver {
    private static final double PIVOT = 1e-11;

    private final int size, band;
    private final int[] rank, order;
    private final boolean[] fixed;
    /** The lower band by equation: row i holds columns i - band .. i, (band + 1) per row. */
    private final double[] factor;

    private FrameSolver(int[] rank, int[] order, int band, boolean[] fixed, double[] factor) {
        this.rank = rank; this.order = order; this.band = band; this.fixed = fixed; this.factor = factor;
        this.size = rank.length * 6;
    }

    /**
     * Assembles the frame with springs to the ground (N/m, N m/rad, 6 per node; null: none) and fixed freedoms (6 per node;
     * null: none) and factors it.
     */
    public static FrameSolver factor(Frame frame, double[] springs, boolean[] fixed) {
        int n = frame.nodeCount();
        if (n == 0) { throw new IllegalArgumentException("A frame needs nodes"); }
        if (springs != null && springs.length != 6 * n) { throw new IllegalArgumentException("Springs are 6 per node"); }
        if (fixed != null && fixed.length != 6 * n) { throw new IllegalArgumentException("Fixed freedoms are 6 per node"); }
        requireConnected(frame);
        Integer[] sorted = new Integer[n];
        for (int i = 0; i < n; i++) { sorted[i] = i; }
        Arrays.sort(sorted, Comparator.<Integer>comparingDouble(i -> frame.position(i).x()).thenComparingInt(i -> i));
        int[] rank = new int[n], order = new int[n];
        for (int i = 0; i < n; i++) { order[i] = sorted[i]; rank[sorted[i]] = i; }
        int band = 5;
        for (Frame.Member m : frame.members()) { band = Math.max(band, 6 * Math.abs(rank[m.a()] - rank[m.b()]) + 5); }
        int size = 6 * n, w = band + 1;
        double[] a = new double[size * w];
        int[] eq = new int[12];
        for (int m = 0; m < frame.memberCount(); m++) {
            Frame.Member mem = frame.member(m);
            double[] kg = Beam.toFrame(Beam.local(frame, m), Beam.rotation(frame, m));
            for (int p = 0; p < 6; p++) { eq[p] = rank[mem.a()] * 6 + p; eq[6 + p] = rank[mem.b()] * 6 + p; }
            for (int p = 0; p < 12; p++) {
                for (int q = 0; q < 12; q++) {
                    int i = eq[p], j = eq[q];
                    if (i >= j) { a[i * w + j - i + band] += kg[p * 12 + q]; }
                }
            }
        }
        boolean[] fix = new boolean[size];
        for (int node = 0; node < n; node++) {
            for (int d = 0; d < 6; d++) {
                int e = rank[node] * 6 + d;
                if (springs != null) {
                    double s = springs[node * 6 + d];
                    Require.nonnegative(s, "A spring");
                    a[e * w + band] += s;
                }
                if (fixed != null && fixed[node * 6 + d]) { fix[e] = true; }
            }
        }
        for (int e = 0; e < size; e++) {
            if (!fix[e]) { continue; }
            for (int j = Math.max(0, e - band); j < e; j++) { a[e * w + j - e + band] = 0; }
            for (int i = e + 1; i <= Math.min(size - 1, e + band); i++) { a[i * w + e - i + band] = 0; }
            a[e * w + band] = 1;
        }
        double[] diagonal = new double[size];
        for (int i = 0; i < size; i++) { diagonal[i] = a[i * w + band]; }
        for (int i = 0; i < size; i++) {
            int j0 = Math.max(0, i - band), ri = i * w - i + band;
            for (int j = j0; j <= i; j++) {
                int rj = j * w - j + band, k0 = Math.max(j0, j - band);
                double s = a[ri + j];
                for (int k = k0; k < j; k++) { s -= a[ri + k] * a[rj + k]; }
                if (j < i) {
                    a[ri + j] = s / a[rj + j];
                } else {
                    if (!(s > PIVOT * diagonal[i])) { throw new FrameDisconnectedException(order[i / 6]); }
                    a[ri + i] = Math.sqrt(s);
                }
            }
        }
        return new FrameSolver(rank, order, band, fix, a);
    }

    /** Displacements (6 per node by node index) under loads (6 per node; those on fixed freedoms are left out). */
    public double[] solve(double[] loads) {
        if (loads.length != size) { throw new IllegalArgumentException("Loads are 6 per node"); }
        int w = band + 1, n = size / 6;
        double[] x = new double[size];
        for (int node = 0; node < n; node++) {
            for (int d = 0; d < 6; d++) {
                int e = rank[node] * 6 + d;
                x[e] = fixed[e] ? 0 : loads[node * 6 + d];
            }
        }
        for (int i = 0; i < size; i++) {
            int ri = i * w - i + band;
            double s = x[i];
            for (int k = Math.max(0, i - band); k < i; k++) { s -= factor[ri + k] * x[k]; }
            x[i] = s / factor[ri + i];
        }
        for (int i = size - 1; i >= 0; i--) {
            double s = x[i];
            for (int k = i + 1, end = Math.min(size - 1, i + band); k <= end; k++) { s -= factor[k * w - k + band + i] * x[k]; }
            x[i] = s / factor[i * w + band];
        }
        double[] out = new double[size];
        for (int node = 0; node < n; node++) { System.arraycopy(x, rank[node] * 6, out, node * 6, 6); }
        return out;
    }

    /** Half-bandwidth of the factored matrix (freedoms). */
    public int bandwidth() { return band; }

    /** Freedoms: 6 per node. */
    public int freedoms() { return size; }

    /** The nodes in the order they are numbered (by x). */
    public int[] order() { return order.clone(); }

    /** Throws FrameDisconnectedException with the first node (by index) off the frame's largest piece. */
    public static void requireConnected(Frame frame) {
        int n = frame.nodeCount();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) { parent[i] = i; }
        for (Frame.Member m : frame.members()) {
            int ra = root(parent, m.a()), rb = root(parent, m.b());
            if (ra != rb) { parent[Math.max(ra, rb)] = Math.min(ra, rb); }
        }
        int[] count = new int[n];
        for (int i = 0; i < n; i++) { count[root(parent, i)]++; }
        int main = 0;
        for (int i = 1; i < n; i++) { if (count[i] > count[main]) { main = i; } }
        for (int i = 0; i < n; i++) { if (root(parent, i) != main) { throw new FrameDisconnectedException(i); } }
    }

    private static int root(int[] parent, int i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }
}

package dev.mcmod.aerocore.api.vehicle;

import java.util.List;

/**
 * A twisting frame in a step (0.33). The frame's stations have no inertia of their own: each step their twists theta
 * (each section rolled about +x against the rigid body, which keeps the vehicle's mass and its mean roll) are where the
 * frame's stiffness K balances what the wheels push with. The wheels are probed first (Part.probe): each reports its
 * force on the body F0 along its travel at the step's start twist, its stiffness k within the step (tyre, spring and
 * damper in series, as Suspension.Step's slope over the step) and where it is (x along the frame, z across). Twisting
 * a station by theta lifts a point at z by -z theta, so each wheel then pushes F0 + k z n'(theta - theta_start) (n
 * the stations' hat functions at its x), and the stations balance with the body taking their mean (weights w, the
 * stations' shares of the vehicle's mass, w' theta = 0, its multiplier the roll moment the body takes):
 *
 * <pre>
 * [ K + sum k z^2 n n'   w ] [theta ]   [ sum k z^2 n n' theta_start - sum z F0 n ]
 * [ w'                   0 ] [lambda] = [ 0                                        ]
 * </pre>
 *
 * Then each wheel's loads move its attachment by the twist (PartStep.flexLift, flexRate), so the force it gives at the
 * end of the step is the one the solve assumed. With two stations it is the series spring of the suspensions' roll
 * stiffnesses and the frame's: a warp is shared between them. Implicit in the twist and without inertia, stiff frames
 * (1e5 to 1e7 N m/rad) are stable at any step. The twist is solved from the body's start velocities (twist first, then
 * the body): a first-order splitting in transients. Only twist: the frame's bending stays rigid.
 */
public final class FrameFlex {
    private FrameFlex() {}

    /** A wheel's report: x along the frame and z across (vehicle frame, m), its force on the body (N, up) and stiffness within the step (N/m). */
    public record Corner(double xM, double zM, double forceN, double stiffnessNPerM) {}

    /** The stations' twists at the step's end. */
    public static double[] solve(Vehicle.Flex frame, double[] start, List<Corner> corners) {
        int n = frame.stationX().length;
        double[][] a = new double[n + 1][n + 1];
        double[] b = new double[n + 1];
        for (int i = 0; i < n; i++) { System.arraycopy(frame.twistNmPerRad()[i], 0, a[i], 0, n); }
        for (Corner c : corners) {
            double[] sh = shape(frame.stationX(), c.xM());
            double kz2 = c.stiffnessNPerM() * c.zM() * c.zM(), was = 0;
            for (int i = 0; i < n; i++) { was += sh[i] * start[i]; }
            for (int i = 0; i < n; i++) {
                if (sh[i] == 0) { continue; }
                for (int j = 0; j < n; j++) { a[i][j] += kz2 * sh[i] * sh[j]; }
                b[i] += kz2 * sh[i] * was - c.zM() * c.forceN() * sh[i];
            }
        }
        for (int i = 0; i < n; i++) { a[i][n] = frame.weights()[i]; a[n][i] = frame.weights()[i]; }
        double[] x = solveDense(a, b);
        if (x == null) { return start.clone(); }
        double[] theta = new double[n];
        System.arraycopy(x, 0, theta, 0, n);
        return theta;
    }

    /** The stations' hat functions at x (1 at a station, falling to 0 at its neighbours; the end stations' held beyond them). */
    public static double[] shape(double[] stationX, double x) {
        int n = stationX.length;
        double[] out = new double[n];
        if (x <= stationX[0]) { out[0] = 1; return out; }
        if (x >= stationX[n - 1]) { out[n - 1] = 1; return out; }
        for (int i = 0; i + 1 < n; i++) {
            if (x <= stationX[i + 1]) {
                double t = (x - stationX[i]) / (stationX[i + 1] - stationX[i]);
                out[i] = 1 - t; out[i + 1] = t;
                return out;
            }
        }
        out[n - 1] = 1;
        return out;
    }

    /** A twist at x from the stations' (linear between them, held beyond the ends). */
    public static double at(double[] stationX, double[] theta, double x) {
        double[] sh = shape(stationX, x);
        double t = 0;
        for (int i = 0; i < sh.length; i++) { t += sh[i] * theta[i]; }
        return t;
    }

    /** Gaussian elimination with partial pivoting; null when singular. */
    static double[] solveDense(double[][] a0, double[] b0) {
        int n = b0.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) { a[i] = a0[i].clone(); }
        double[] b = b0.clone();
        double scale = 0;
        for (double[] row : a) { for (double v : row) { scale = Math.max(scale, Math.abs(v)); } }
        if (!(scale > 0)) { return null; }
        for (int c = 0; c < n; c++) {
            int p = c;
            for (int r = c + 1; r < n; r++) { if (Math.abs(a[r][c]) > Math.abs(a[p][c])) { p = r; } }
            if (Math.abs(a[p][c]) <= 1e-14 * scale) { return null; }
            double[] t = a[c]; a[c] = a[p]; a[p] = t;
            double tb = b[c]; b[c] = b[p]; b[p] = tb;
            for (int r = c + 1; r < n; r++) {
                double f = a[r][c] / a[c][c];
                if (f == 0) { continue; }
                for (int k = c; k < n; k++) { a[r][k] -= f * a[c][k]; }
                b[r] -= f * b[c];
            }
        }
        double[] x = new double[n];
        for (int r = n - 1; r >= 0; r--) {
            double s = b[r];
            for (int k = r + 1; k < n; k++) { s -= a[r][k] * x[k]; }
            x[r] = s / a[r][r];
        }
        return x;
    }
}

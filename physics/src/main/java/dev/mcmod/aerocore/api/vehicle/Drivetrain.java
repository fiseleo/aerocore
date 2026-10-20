package dev.mcmod.aerocore.api.vehicle;

import java.util.List;

/**
 * The drivetrain solve of one step (0.24): the shaft trains' end speeds together with the torques of the rows that
 * join them (gears, clutches, brakes, freewheels, differentials) as one bounded linear complementarity problem.
 *
 * <p>Each train t is one degree of freedom with inertia M_t, reflected torque T_t and torque slopes K (dT/domega, a full
 * matrix when parts couple trains smoothly: a viscous coupling, a torque converter). Backward Euler gives
 * A (omega' - omega) = dt (T + J_f^T Lambda), A = diag(M) - dt K with the diagonal slopes taken only where they damp
 * (as the trains alone, VehicleDynamics) and the off-diagonal ones as they are. A row i asks J_i omega' = b_i with its
 * torque Lambda_i in [lo_i, hi_i]: free between the bounds the row holds (a gear, a locked clutch, a brake holding), at a
 * bound it slips and carries that torque (a slipping clutch, a sliding brake, an overrunning freewheel). Its residual
 * w = q + D Lambda with D = dt J A^-1 J_f^T is complementary to the bounds: w = 0 inside, w >= 0 at lo, w <= 0 at hi.
 * A row's bounds may follow another row's torque, -/+(base + per |Lambda_ref|) (a limited-slip differential's friction
 * on its input torque, a Torsen's bias): at its bound such a row is a linear relation to the other row's torque and is
 * solved with it.
 *
 * <p>J is kinematic (rotor speed ratios); J_f, the torques' Jacobian, differs by losses: a row's efficiency on the
 * side that takes power from it, and on the way to a train's root a rotor's path efficiency (driving torques times it,
 * loads divided by it, as for torques on the trains alone). Which side takes power depends on the answer, so the solve
 * is repeated with the factors of the last answer until they stop changing (at most four passes; without losses, one).
 *
 * <p>The complementarity problem is solved by block principal pivoting (Judice and Pires 1994), switching to one pivot
 * at a time by least index when the count of wrong rows stops falling (finite for a P-matrix, which D is for
 * independent rows), and by projected Gauss-Seidel if that fails. Redundant rows (two locks on one train) make D
 * singular; then its diagonal is regularised by 1e-10 of its largest entry, which lets them share their torque at a
 * speed error of that order.
 */
final class Drivetrain {
    private Drivetrain() {}

    /** Passes over the loss factors. */
    static final int PASSES = 4;
    /** Relative regularisation of D's diagonal when the rows are redundant. */
    static final double REGULARISATION = 1e-10;

    /**
     * A row in train space: entries (train, kinematic coefficient = rotor coefficient x rotor ratio, path efficiency of
     * that rotor, 1 for a driver); target speed of the combination (rad/s); torque bounds (N m); efficiency on the side
     * taking power. With boundRow >= 0 the bounds are -/+(boundBase + boundPer |torque of boundRow|) instead.
     */
    record Row(int[] trains, double[] coefficients, double[] pathEfficiency, double target, double low, double high, double efficiency,
               int boundRow, double boundBase, double boundPer) {}

    /** End speeds of the trains, each row's torque (N m), the pivoting steps taken and whether Gauss-Seidel had to finish. */
    record Solution(double[] speeds, double[] torques, int iterations, boolean fallback) {}

    /**
     * One step: inertias M (kg m^2 at each train's root), torques T (N m), slopes K (n x n, dT_t/domega_s), start speeds,
     * rows, dt.
     */
    static Solution solve(double[] mass, double[] torque, double[][] slopes, double[] start, List<Row> rows, double dt) {
        int n = mass.length, m = rows.size();
        double[][] a = new double[n][n];
        for (int t = 0; t < n; t++) {
            for (int s = 0; s < n; s++) { a[t][s] = t == s ? mass[t] - dt * Math.min(slopes[t][t], 0) : -dt * slopes[t][s]; }
        }
        Lu lu = Lu.of(a);
        if (lu == null) {
            // smooth couplings that make A singular: take them explicitly instead
            for (int t = 0; t < n; t++) for (int s = 0; s < n; s++) { if (t != s) { a[t][s] = 0; } }
            lu = Lu.of(a);
        }
        double[] free = lu.solve(torque);
        double[] speeds = new double[n];
        for (int t = 0; t < n; t++) { speeds[t] = start[t] + dt * free[t]; }
        if (m == 0) { return new Solution(speeds, new double[0], 0, false); }

        double[][] j = new double[m][n];
        double[] q = new double[m], low = new double[m], high = new double[m], base = new double[m], per = new double[m];
        int[] ref = new int[m];
        boolean[] empty = new boolean[m];
        double[][] factor = new double[m][];
        for (int i = 0; i < m; i++) {
            Row r = rows.get(i);
            for (int e = 0; e < r.trains().length; e++) { j[i][r.trains()[e]] += r.coefficients()[e]; }
            double norm = 0, scale = 0;
            for (int e = 0; e < r.trains().length; e++) { scale = Math.max(scale, Math.abs(r.coefficients()[e])); }
            for (int t = 0; t < n; t++) { norm = Math.max(norm, Math.abs(j[i][t])); }
            // a row whose rotors are geared together already (one train) holds nothing
            empty[i] = !(norm > 1e-12 * scale);
            q[i] = -r.target();
            for (int t = 0; t < n; t++) { q[i] += j[i][t] * speeds[t]; }
            factor[i] = new double[r.trains().length];
            java.util.Arrays.fill(factor[i], 1);
            ref[i] = empty[i] ? -1 : r.boundRow();
            low[i] = empty[i] ? 0 : r.low();
            high[i] = empty[i] ? 0 : r.high();
            base[i] = Math.max(0, r.boundBase());
            per[i] = r.boundPer();
        }
        double[] lambda = new double[m];
        double[][] x = new double[m][];
        double[][] d = new double[m][m];
        int iterations = 0;
        boolean fallback = false;
        for (int pass = 0; pass < PASSES; pass++) {
            for (int i = 0; i < m; i++) {
                Row r = rows.get(i);
                double[] column = new double[n];
                for (int e = 0; e < r.trains().length; e++) { column[r.trains()[e]] += r.coefficients()[e] * factor[i][e]; }
                x[i] = lu.solve(column);
            }
            double largest = 0;
            for (int i = 0; i < m; i++) {
                for (int k = 0; k < m; k++) {
                    double sum = 0;
                    for (int t = 0; t < n; t++) { sum += j[i][t] * x[k][t]; }
                    d[i][k] = dt * sum;
                }
                largest = Math.max(largest, Math.abs(d[i][i]));
            }
            for (int i = 0; i < m; i++) { if (empty[i]) { d[i][i] = 1; } }
            Lcp lcp = Lcp.solve(d, q, low, high, ref, base, per, REGULARISATION * largest, lambda);
            lambda = lcp.lambda();
            iterations += lcp.iterations();
            fallback |= lcp.fallback();
            for (int t = 0; t < n; t++) {
                double sum = free[t];
                for (int k = 0; k < m; k++) { sum += x[k][t] * lambda[k]; }
                speeds[t] = start[t] + dt * sum;
            }
            // the loss factors this answer implies
            boolean changed = false;
            for (int i = 0; i < m; i++) {
                Row r = rows.get(i);
                for (int e = 0; e < r.trains().length; e++) {
                    if (r.efficiency() == 1 && r.pathEfficiency()[e] == 1) { continue; }
                    double power = r.coefficients()[e] * lambda[i] * speeds[r.trains()[e]];
                    double scale = Math.abs(r.coefficients()[e] * lambda[i]) * (1 + Math.abs(speeds[r.trains()[e]]));
                    if (Math.abs(power) <= 1e-9 * scale) { continue; }
                    double f = power > 0 ? r.pathEfficiency()[e] * r.efficiency() : 1 / r.pathEfficiency()[e];
                    if (f != factor[i][e]) { factor[i][e] = f; changed = true; }
                }
            }
            if (!changed) { break; }
        }
        double[] torques = new double[m];
        for (int i = 0; i < m; i++) { torques[i] = empty[i] ? 0 : lambda[i]; }
        return new Solution(speeds, torques, iterations, fallback);
    }

    /** A bounded linear complementarity problem's answer: the row torques, pivoting steps, whether Gauss-Seidel finished it. */
    record Lcp(double[] lambda, int iterations, boolean fallback) {
        /** Iteration cap of the pivoting before Gauss-Seidel takes over. */
        static final int PIVOTS = 60;
        /** Gauss-Seidel sweeps at most. */
        static final int SWEEPS = 500;

        /**
         * w = q + D lambda with lambda in [low, high] (or, where ref >= 0, in -/+(base + per |lambda_ref|)) and
         * lambda strictly inside -> w = 0, at the low bound -> w >= 0, at the high one -> w <= 0. regularisation is added
         * to D's diagonal only where the free rows' D is singular (redundant rows); guess sets the first active set.
         */
        static Lcp solve(double[][] d, double[] q, double[] low, double[] high, int[] ref, double[] base, double[] per, double regularisation,
                         double[] guess) {
            int m = q.length;
            double qScale = 0;
            for (double v : q) { qScale = Math.max(qScale, Math.abs(v)); }
            double tw = 1e-10 * (1 + qScale);
            double[] lambda = new double[m];
            for (int i = 0; i < m; i++) { lambda[i] = Double.isFinite(guess[i]) ? guess[i] : 0; }
            // state: 0 free, -1 at the low bound, +1 at the high one; a row of constant equal bounds stays at them
            int[] state = new int[m];
            for (int i = 0; i < m; i++) {
                double lo = lower(i, lambda, low, ref, base, per), hi = upper(i, lambda, high, ref, base, per);
                if (ref[i] < 0 && low[i] == high[i]) { state[i] = -1; }
                else if (lambda[i] <= lo && lo > Double.NEGATIVE_INFINITY) { state[i] = -1; }
                else if (lambda[i] >= hi && hi < Double.POSITIVE_INFINITY) { state[i] = 1; }
            }
            int best = Integer.MAX_VALUE, stale = 0;
            for (int iteration = 1; iteration <= PIVOTS; iteration++) {
                double[] sign = new double[m];
                for (int i = 0; i < m; i++) { sign[i] = lambda[i] < 0 ? -1 : 1; }
                if (!solveSet(d, q, low, high, ref, base, per, sign, state, lambda, regularisation)) { break; }
                double[] w = residual(d, q, lambda);
                int wrong = 0, first = -1;
                boolean[] flip = new boolean[m];
                for (int i = 0; i < m; i++) {
                    double lo = lower(i, lambda, low, ref, base, per), hi = upper(i, lambda, high, ref, base, per);
                    double tl = 1e-9 * (1 + Math.max(Double.isFinite(lo) ? Math.abs(lo) : 0, Double.isFinite(hi) ? Math.abs(hi) : 0));
                    boolean bad = state[i] == 0 ? lambda[i] < lo - tl || lambda[i] > hi + tl
                            : !(ref[i] < 0 && low[i] == high[i]) && (state[i] < 0 ? w[i] < -tw : w[i] > tw);
                    if (bad) { flip[i] = true; wrong++; if (first < 0) { first = i; } }
                    // a dependent bound solved with the wrong sign of the torque it follows: solve again with the right one
                    else if (state[i] != 0 && ref[i] >= 0 && Math.abs(lambda[ref[i]]) > tl && (lambda[ref[i]] < 0 ? -1 : 1) != sign[ref[i]]) { wrong++; }
                }
                if (wrong == 0) { clampInside(lambda, low, high, ref, base, per); return new Lcp(lambda, iteration, false); }
                if (wrong < best) { best = wrong; stale = 0; } else { stale++; }
                for (int i = 0; i < m; i++) {
                    if (!flip[i] || (stale >= 3 && i != first)) { continue; }
                    if (state[i] == 0) { state[i] = lambda[i] < lower(i, lambda, low, ref, base, per) ? -1 : 1; } else { state[i] = 0; }
                }
            }
            // Gauss-Seidel from the best guess there is
            double[] next = lambda;
            for (int i = 0; i < m; i++) { if (!Double.isFinite(next[i])) { next[i] = 0; } }
            clampInside(next, low, high, ref, base, per);
            for (int sweep = 0; sweep < SWEEPS; sweep++) {
                double change = 0, size = 0;
                for (int i = 0; i < m; i++) {
                    double w = q[i];
                    for (int k = 0; k < m; k++) { w += d[i][k] * next[k]; }
                    double value = Math.clamp(next[i] - w / (d[i][i] + regularisation), lower(i, next, low, ref, base, per), upper(i, next, high, ref, base, per));
                    change = Math.max(change, Math.abs(value - next[i]));
                    size = Math.max(size, Math.abs(value));
                    next[i] = value;
                }
                if (change <= 1e-12 * (1 + size)) { break; }
            }
            return new Lcp(next, PIVOTS, true);
        }

        private static double lower(int i, double[] lambda, double[] low, int[] ref, double[] base, double[] per) {
            return ref[i] < 0 ? low[i] : -(base[i] + per[i] * Math.abs(lambda[ref[i]]));
        }

        private static double upper(int i, double[] lambda, double[] high, int[] ref, double[] base, double[] per) {
            return ref[i] < 0 ? high[i] : base[i] + per[i] * Math.abs(lambda[ref[i]]);
        }

        private static void clampInside(double[] lambda, double[] low, double[] high, int[] ref, double[] base, double[] per) {
            for (int i = 0; i < lambda.length; i++) {
                if (ref[i] < 0) { lambda[i] = Math.clamp(lambda[i], low[i], high[i]); }
            }
            for (int i = 0; i < lambda.length; i++) {
                if (ref[i] >= 0) { lambda[i] = Math.clamp(lambda[i], lower(i, lambda, low, ref, base, per), upper(i, lambda, high, ref, base, per)); }
            }
        }

        private static double[] residual(double[][] d, double[] q, double[] lambda) {
            double[] w = q.clone();
            for (int i = 0; i < q.length; i++) for (int k = 0; k < q.length; k++) { w[i] += d[i][k] * lambda[k]; }
            return w;
        }

        /**
         * The torques for an active set: free rows hold (w = 0), rows at a constant bound carry it, rows at a bound that
         * follows another row's torque carry base + per x sign x lambda_ref (sign the reference's present sign), solved
         * together. False when that system is singular even regularised.
         */
        private static boolean solveSet(double[][] d, double[] q, double[] low, double[] high, int[] ref, double[] base, double[] per,
                                        double[] sign, int[] state, double[] lambda, double regularisation) {
            int m = q.length, u = 0;
            int[] index = new int[m], position = new int[m];
            java.util.Arrays.fill(position, -1);
            for (int i = 0; i < m; i++) {
                // an unbounded side cannot be held at its bound
                if (ref[i] < 0 && state[i] != 0 && !Double.isFinite(state[i] < 0 ? low[i] : high[i])) { state[i] = 0; }
                if (state[i] == 0 || ref[i] >= 0) { position[i] = u; index[u++] = i; }
                else { lambda[i] = state[i] < 0 ? low[i] : high[i]; }
            }
            if (u == 0) { return true; }
            for (int attempt = 0; attempt < 2; attempt++) {
                double[][] a = new double[u][u];
                double[] b = new double[u];
                for (int r = 0; r < u; r++) {
                    int i = index[r];
                    if (state[i] == 0) {
                        b[r] = -q[i];
                        for (int k = 0; k < m; k++) {
                            if (position[k] >= 0) { a[r][position[k]] += d[i][k]; } else { b[r] -= d[i][k] * lambda[k]; }
                        }
                        if (attempt == 1) { a[r][r] += regularisation; }
                    } else {
                        // lambda_i = state (base + per sign_ref lambda_ref)
                        int k = ref[i];
                        a[r][r] = 1;
                        b[r] = state[i] * base[i];
                        if (position[k] >= 0) { a[r][position[k]] -= state[i] * per[i] * sign[k]; } else { b[r] += state[i] * per[i] * Math.abs(lambda[k]); }
                    }
                }
                Lu lu = Lu.of(a);
                if (lu == null) { continue; }
                double[] solved = lu.solve(b);
                for (int r = 0; r < u; r++) { lambda[index[r]] = solved[r]; }
                return true;
            }
            return false;
        }
    }

    /** Dense LU with partial pivoting; of() returns null for a singular or non-finite matrix. */
    static final class Lu {
        private final double[][] lu;
        private final int[] pivot;

        private Lu(double[][] lu, int[] pivot) { this.lu = lu; this.pivot = pivot; }

        static Lu of(double[][] matrix) {
            int n = matrix.length;
            double[][] lu = new double[n][];
            double scale = 0;
            for (int i = 0; i < n; i++) {
                lu[i] = matrix[i].clone();
                for (double v : lu[i]) {
                    if (!Double.isFinite(v)) { return null; }
                    scale = Math.max(scale, Math.abs(v));
                }
            }
            int[] pivot = new int[n];
            for (int c = 0; c < n; c++) {
                int p = c;
                for (int r = c + 1; r < n; r++) { if (Math.abs(lu[r][c]) > Math.abs(lu[p][c])) { p = r; } }
                if (!(Math.abs(lu[p][c]) > 1e-12 * scale)) { return null; }
                pivot[c] = p;
                if (p != c) { double[] t = lu[p]; lu[p] = lu[c]; lu[c] = t; }
                for (int r = c + 1; r < n; r++) {
                    double f = lu[r][c] / lu[c][c];
                    lu[r][c] = f;
                    if (f != 0) { for (int k = c + 1; k < n; k++) { lu[r][k] -= f * lu[c][k]; } }
                }
            }
            return new Lu(lu, pivot);
        }

        double[] solve(double[] b) {
            int n = b.length;
            double[] x = b.clone();
            // the row exchanges in the order they were made (whole rows, multipliers too), then L and U
            for (int c = 0; c < n; c++) { if (pivot[c] != c) { double t = x[c]; x[c] = x[pivot[c]]; x[pivot[c]] = t; } }
            for (int c = 0; c < n; c++) {
                for (int r = c + 1; r < n; r++) { x[r] -= lu[r][c] * x[c]; }
            }
            for (int r = n - 1; r >= 0; r--) {
                for (int k = r + 1; k < n; k++) { x[r] -= lu[r][k] * x[k]; }
                x[r] /= lu[r][r];
            }
            return x;
        }
    }
}

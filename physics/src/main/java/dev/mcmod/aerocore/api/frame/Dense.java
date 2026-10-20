package dev.mcmod.aerocore.api.frame;

/** Small dense symmetric matrices: Cholesky inverse and Jacobi eigenvalues. */
final class Dense {
    private Dense() {}

    /** The inverse of a symmetric positive definite matrix; null when it is not positive definite. */
    static double[][] inverse(double[][] a) {
        int n = a.length;
        double[][] l = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double s = a[i][j];
                for (int k = 0; k < j; k++) { s -= l[i][k] * l[j][k]; }
                if (j < i) {
                    l[i][j] = s / l[j][j];
                } else {
                    if (!(s > 1e-12 * Math.abs(a[i][i]))) { return null; }
                    l[i][i] = Math.sqrt(s);
                }
            }
        }
        double[][] inv = new double[n][n];
        double[] y = new double[n];
        for (int c = 0; c < n; c++) {
            for (int i = 0; i < n; i++) {
                double s = i == c ? 1 : 0;
                for (int k = 0; k < i; k++) { s -= l[i][k] * y[k]; }
                y[i] = s / l[i][i];
            }
            for (int i = n - 1; i >= 0; i--) {
                double s = y[i];
                for (int k = i + 1; k < n; k++) { s -= l[k][i] * inv[k][c]; }
                inv[i][c] = s / l[i][i];
            }
        }
        return inv;
    }

    /**
     * Eigenvalues of a symmetric matrix by cyclic Jacobi rotations; vectors (n x n, or null) receives the eigenvectors as
     * columns.
     */
    static double[] eigen(double[][] matrix, double[][] vectors) {
        int n = matrix.length;
        double[][] a = new double[n][];
        for (int i = 0; i < n; i++) { a[i] = matrix[i].clone(); }
        double[][] v = vectors != null ? vectors : new double[n][n];
        for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) { v[i][j] = i == j ? 1 : 0; } }
        double norm = 0;
        for (double[] row : a) { for (double x : row) { norm += x * x; } }
        for (int sweep = 0; sweep < 100; sweep++) {
            double off = 0;
            for (int p = 0; p < n; p++) { for (int q = p + 1; q < n; q++) { off += a[p][q] * a[p][q]; } }
            if (!(off > 1e-32 * norm)) { break; }
            for (int p = 0; p < n; p++) {
                for (int q = p + 1; q < n; q++) {
                    if (a[p][q] == 0) { continue; }
                    double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                    double t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
                    double c = 1 / Math.sqrt(t * t + 1), s = t * c;
                    for (int k = 0; k < n; k++) {
                        double kp = a[k][p], kq = a[k][q];
                        a[k][p] = c * kp - s * kq; a[k][q] = s * kp + c * kq;
                    }
                    for (int k = 0; k < n; k++) {
                        double pk = a[p][k], qk = a[q][k];
                        a[p][k] = c * pk - s * qk; a[q][k] = s * pk + c * qk;
                    }
                    for (int k = 0; k < n; k++) {
                        double kp = v[k][p], kq = v[k][q];
                        v[k][p] = c * kp - s * kq; v[k][q] = s * kp + c * kq;
                    }
                }
            }
        }
        double[] values = new double[n];
        for (int i = 0; i < n; i++) { values[i] = a[i][i]; }
        return values;
    }
}

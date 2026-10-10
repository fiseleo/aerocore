package dev.mcmod.aerocore.api;

/**
 * In-place radix-2 complex FFT on an n x n grid stored row-major as separate real and imaginary arrays
 * (index = row * n + column). forward computes X(k) = sum x(j) e^{-2 pi i jk/n} along both axes; inverse divides by n^2.
 * Not thread-safe per instance (it reuses scratch rows).
 */
public final class Fft2D {
    private final int n, log;
    private final double[] cos, sin, columnRe, columnIm;
    private final int[] reversed;

    public Fft2D(int n) {
        if (n < 2 || Integer.bitCount(n) != 1) { throw new IllegalArgumentException("Grid size must be a power of two"); }
        this.n = n;
        log = Integer.numberOfTrailingZeros(n);
        cos = new double[n / 2]; sin = new double[n / 2];
        for (int i = 0; i < n / 2; i++) { cos[i] = Math.cos(2 * Math.PI * i / n); sin[i] = Math.sin(2 * Math.PI * i / n); }
        reversed = new int[n];
        for (int i = 0; i < n; i++) { reversed[i] = Integer.reverse(i) >>> (32 - log); }
        columnRe = new double[n]; columnIm = new double[n];
    }

    public int size() { return n; }

    public void forward(double[] re, double[] im) { transform(re, im, false); }

    public void inverse(double[] re, double[] im) {
        transform(re, im, true);
        double scale = 1.0 / ((double) n * n);
        for (int i = 0; i < re.length; i++) { re[i] *= scale; im[i] *= scale; }
    }

    private void transform(double[] re, double[] im, boolean inverse) {
        for (int row = 0; row < n; row++) { line(re, im, row * n, 1, inverse); }
        for (int column = 0; column < n; column++) {
            for (int i = 0; i < n; i++) { columnRe[i] = re[i * n + column]; columnIm[i] = im[i * n + column]; }
            line(columnRe, columnIm, 0, 1, inverse);
            for (int i = 0; i < n; i++) { re[i * n + column] = columnRe[i]; im[i * n + column] = columnIm[i]; }
        }
    }

    /** One contiguous line of n samples starting at offset. */
    private void line(double[] re, double[] im, int offset, int stride, boolean inverse) {
        for (int i = 0; i < n; i++) {
            int j = reversed[i];
            if (j > i) {
                int a = offset + i, b = offset + j;
                double t = re[a]; re[a] = re[b]; re[b] = t;
                t = im[a]; im[a] = im[b]; im[b] = t;
            }
        }
        double sign = inverse ? 1 : -1;
        for (int size = 2; size <= n; size <<= 1) {
            int half = size >> 1, step = n / size;
            for (int start = 0; start < n; start += size) {
                for (int k = 0; k < half; k++) {
                    double wr = cos[k * step], wi = sign * sin[k * step];
                    int a = offset + start + k, b = a + half;
                    double xr = re[b] * wr - im[b] * wi, xi = re[b] * wi + im[b] * wr;
                    re[b] = re[a] - xr; im[b] = im[a] - xi;
                    re[a] += xr; im[a] += xi;
                }
            }
        }
    }
}

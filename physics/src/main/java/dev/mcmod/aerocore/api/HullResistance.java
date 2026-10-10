package dev.mcmod.aerocore.api;

import java.util.function.DoubleBinaryOperator;

/**
 * Calm-water resistance components of displacement hulls, R = 1/2 rho U^2 S [(1 + k) C_F + C_W]:
 * ITTC-1957 friction line (ITTC Recommended Procedure 7.5-02-02-01), Holtrop and Mennen (1982) form factor for
 * surface ships, Hoerner (1965) form factor for streamlined bodies of revolution, and Michell's (1898) thin-ship
 * wave resistance in the form of Tuck (1989). Displacement-hull theory: no planing lift, sinkage, trim or spray.
 */
public final class HullResistance {
    /** Below this Reynolds number the ITTC line is held constant (it is singular at Re = 100). */
    public static final double MIN_REYNOLDS = 1e5;
    private static final double[] GAUSS_X = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
            0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363};
    private static final double[] GAUSS_W = {0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
            0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};
    private HullResistance() {}

    /** ITTC-1957 model-ship correlation line C_F = 0.075 / (log10 Re - 2)^2, Re held at >= 1e5. */
    public static double ittc1957(double reynolds) {
        Checks.nonnegative(reynolds, "Reynolds number");
        double log = Math.log10(Math.max(reynolds, MIN_REYNOLDS)) - 2;
        return 0.075 / (log * log);
    }

    /** Holtrop and Mennen (1982) run length L_R / L = 1 - C_P + 0.06 C_P lcb / (4 C_P - 1); lcb in % of L forward of midships. */
    public static double runLengthRatio(double prismatic, double lcbPercent) {
        return 1 - prismatic + 0.06 * prismatic * lcbPercent / (4 * prismatic - 1);
    }

    /**
     * Holtrop and Mennen (1982) form factor 1 + k1 = c13 {0.93 + c12 (B/L_R)^0.92497 (0.95 - C_P)^-0.521448
     * (1 - C_P + 0.0225 lcb)^0.6906}, c13 = 1 + 0.003 C_stern (-10 V-shaped, 0 normal, +10 U-shaped sections).
     * Regression range about 3.9 < L/B < 15, 2.1 < B/T < 4, 0.55 < C_P < 0.85; C_P is held below 0.9 here.
     */
    public static double holtropFormFactor(double length, double beam, double draft, double prismatic, double lcbPercent, double sternShape) {
        Checks.positive(length, "Length"); Checks.positive(beam, "Beam"); Checks.positive(draft, "Draft");
        double cp = Math.clamp(prismatic, 0.3, 0.9), ratio = draft / length;
        double c12 = ratio > 0.05 ? Math.pow(ratio, 0.2228446) : ratio > 0.02 ? 48.20 * Math.pow(ratio - 0.02, 2.078) + 0.479948 : 0.479948;
        double runLength = length * runLengthRatio(cp, lcbPercent);
        double fullness = Math.max(1e-3, 1 - cp + 0.0225 * lcbPercent);
        return (1 + 0.003 * sternShape) * (0.93 + c12 * Math.pow(beam / runLength, 0.92497) * Math.pow(0.95 - cp, -0.521448) * Math.pow(fullness, 0.6906));
    }

    /** Hoerner's form factor of a streamlined body of revolution, 1 + 1.5 (D/L)^1.5 + 7 (D/L)^3, on wetted area. */
    public static double hoernerFormFactor(double diameterToLength) {
        Checks.nonnegative(diameterToLength, "D/L");
        return 1 + 1.5 * Math.pow(diameterToLength, 1.5) + 7 * Math.pow(diameterToLength, 3);
    }

    /**
     * Michell's wave resistance (N) of a thin symmetric hull with half-breadth f(x, depth) on x in [xAft, xForward],
     * depth in [0, draft], at speed U in deep water:
     * R = 4 rho g^2 / (pi U^2) int_1^inf |P|^2 lambda^2 / sqrt(lambda^2 - 1) d lambda,
     * P = int int f_x exp(-k0 lambda^2 depth) exp(i k0 lambda x) dx d depth, k0 = g / U^2 (Tuck 1989, eqs. 7-8).
     * f is sampled on an nx x nz grid and integrated exactly as piecewise linear (by parts in x, so transom ends
     * count); lambda = cosh u removes the end singularity and Gauss-Legendre panels follow the oscillation up to
     * lambdaMax. The theory assumes no sinkage, trim or breaking and over-predicts humps below Fn of about 0.35.
     */
    public static double michell(DoubleBinaryOperator halfBreadth, double xAft, double xForward, double draft,
                                 int nx, int nz, double speed, double density, double gravity, double lambdaMax) {
        Checks.positive(speed, "Speed"); Checks.positive(draft, "Draft"); Checks.positive(gravity, "Gravity");
        if (!(xForward > xAft) || nx < 2 || nz < 1 || !(lambdaMax > 1)) { throw new IllegalArgumentException("Invalid Michell grid"); }
        double[] xs = new double[nx + 1], zs = new double[nz + 1];
        double[][] f = new double[nz + 1][nx + 1];
        for (int j = 0; j <= nx; j++) { xs[j] = xAft + (xForward - xAft) * j / nx; }
        for (int m = 0; m <= nz; m++) {
            zs[m] = draft * m / nz;
            for (int j = 0; j <= nx; j++) { f[m][j] = halfBreadth.applyAsDouble(xs[j], zs[m]); }
        }
        double k0 = gravity / (speed * speed), length = xForward - xAft;
        double uMax = acosh(lambdaMax), sum = 0, u = 0;
        double[] p = new double[2];
        while (u < uMax) {
            // Half an oscillation of cos(k0 lambda L) per panel, at most 0.05 in u.
            double width = Math.min(0.05, Math.min(uMax - u, Math.PI / (k0 * length * Math.sinh(Math.min(uMax, u + 0.05)) + 1e-12)));
            for (int q = 0; q < GAUSS_X.length; q++) {
                double uq = u + 0.5 * width * (1 + GAUSS_X[q]), lambda = Math.cosh(uq);
                integral(f, xs, zs, k0 * lambda, k0 * lambda * lambda, p);
                sum += 0.5 * width * GAUSS_W[q] * (p[0] * p[0] + p[1] * p[1]) * lambda * lambda;
            }
            u += width;
        }
        return 4 * density * gravity * gravity / (Math.PI * speed * speed) * sum;
    }

    /** P = -i k int_depth exp(-K depth) int_x f e^{i k x} dx, with exact weights for piecewise-linear f. */
    private static void integral(double[][] f, double[] xs, double[] zs, double k, double decay, double[] out) {
        int nx = xs.length - 1, nz = zs.length - 1;
        double[] gr = new double[nz + 1], gi = new double[nz + 1];
        for (int m = 0; m <= nz; m++) {
            double re = 0, im = 0;
            for (int j = 0; j < nx; j++) {
                double h = xs[j + 1] - xs[j], f0 = f[m][j], f1 = f[m][j + 1];
                if (f0 == 0 && f1 == 0) { continue; }
                // int_x0^x1 (f0 + s (x - x0)) e^{ikx} dx = e^{ikx0} [f0 A + s B], A = (e^{ikh} - 1)/(ik), B = int_0^h t e^{ikt} dt.
                double kh = k * h, s = (f1 - f0) / h, ar, ai, br, bi;
                if (kh < 1e-3) {
                    ar = h; ai = 0.5 * k * h * h;
                    br = 0.5 * h * h; bi = k * h * h * h / 3;
                } else {
                    double c = Math.cos(kh), sn = Math.sin(kh);
                    ar = sn / k; ai = (1 - c) / k;
                    // B = e^{ikh} (h/(ik) + 1/k^2) - 1/k^2.
                    br = (h * sn) / k + (c - 1) / (k * k);
                    bi = (-h * c) / k + sn / (k * k);
                }
                double tr = f0 * ar + s * br, ti = f0 * ai + s * bi, e0r = Math.cos(k * xs[j]), e0i = Math.sin(k * xs[j]);
                re += e0r * tr - e0i * ti; im += e0r * ti + e0i * tr;
            }
            gr[m] = re; gi[m] = im;
        }
        double sr = 0, si = 0;
        for (int m = 0; m < nz; m++) {
            double h = zs[m + 1] - zs[m], w0, w1, kh = decay * h, e0 = Math.exp(-decay * zs[m]);
            // int_0^h (g0 (1 - t/h) + g1 t/h) e^{-K t} dt, scaled by e^{-K z0}.
            if (kh < 1e-4) { w0 = h / 2; w1 = h / 2; }
            else {
                double e = Math.exp(-kh), first = (1 - e) / decay, second = (1 - e * (1 + kh)) / (decay * decay * h);
                w0 = first - second; w1 = second;
            }
            sr += e0 * (w0 * gr[m] + w1 * gr[m + 1]); si += e0 * (w0 * gi[m] + w1 * gi[m + 1]);
        }
        // Multiply by -i k.
        out[0] = k * si; out[1] = -k * sr;
    }

    private static double acosh(double x) { return Math.log(x + Math.sqrt(x * x - 1)); }
}

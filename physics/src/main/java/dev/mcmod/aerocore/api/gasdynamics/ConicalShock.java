package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Supersonic flow over a circular cone at zero incidence (Taylor and Maccoll, Proc. R. Soc. A 139, 1933; Anderson,
 * Modern Compressible Flow, sec. 10.4): an attached conical shock, then isentropic conical flow to the surface. The
 * Taylor-Maccoll equation in the nondimensional radial velocity V'r(theta) (velocities over the limiting speed)
 * <pre>
 *   (g-1)/2 [1 - V'r^2 - V'r'^2] [2 V'r + V'r' cot(theta) + V'r''] - V'r' [V'r V'r' + V'r' V'r''] = 0
 * </pre>
 * is integrated (RK4) from just behind the shock, where an oblique shock of the same wave angle gives the state,
 * inward until the polar velocity V'theta = V'r' vanishes: that ray is the cone surface. The wave angle for a given
 * cone is found by bisection on the weak branch. Calorically perfect gas; angles in radians; ratios over free stream.
 */
public record ConicalShock(double machAhead, double coneHalfAngle, double waveAngle, double surfaceMach, double surfacePressureRatio,
                           double surfaceTemperatureRatio, double totalPressureRatio) {

    private static final int STEPS = 1000;

    /** The flow over a cone of half-angle delta at Mach M; throws if the shock detaches. */
    public static ConicalShock of(double mach, double coneHalfAngle, double gamma) {
        if (!(mach > 1) || !Double.isFinite(mach) || !(gamma > 1)) { throw new IllegalArgumentException("Conical flow needs M > 1 and gamma > 1"); }
        if (!(coneHalfAngle > 0)) { throw new IllegalArgumentException("Cone half-angle must be positive"); }
        double mu = Math.asin(1 / mach);
        // cone angle rises with the wave angle up to a largest cone, past which the shock detaches
        double lo = mu * (1 + 1e-9), hi = Math.PI / 2 * (1 - 1e-9);
        double r = 0.5 * (Math.sqrt(5) - 1), a = hi - r * (hi - lo), b = lo + r * (hi - lo);
        double fa = coneAngle(mach, a, gamma), fb = coneAngle(mach, b, gamma);
        double l = lo, h = hi;
        for (int i = 0; i < 60 && h - l > 1e-8; i++) {
            if (fa < fb) { l = a; a = b; fa = fb; b = l + r * (h - l); fb = coneAngle(mach, b, gamma); }
            else { h = b; b = a; fb = fa; a = h - r * (h - l); fa = coneAngle(mach, a, gamma); }
        }
        double betaMax = 0.5 * (l + h);
        if (coneHalfAngle > coneAngle(mach, betaMax, gamma)) { throw new IllegalArgumentException("The cone detaches the shock at Mach " + mach); }
        hi = betaMax;
        for (int i = 0; i < 60 && hi - lo > 1e-11; i++) {
            double beta = 0.5 * (lo + hi);
            if (coneAngle(mach, beta, gamma) < coneHalfAngle) { lo = beta; } else { hi = beta; }
        }
        return ofWaveAngle(mach, 0.5 * (lo + hi), gamma);
    }

    /** The cone whose shock stands at wave angle beta, and its surface state. */
    public static ConicalShock ofWaveAngle(double mach, double waveAngle, double gamma) {
        double[] s = integrate(mach, waveAngle, gamma);
        double theta = s[0], v = s[1];
        ObliqueShock shock = ObliqueShock.ofWaveAngle(mach, waveAngle, gamma);
        double machSurface = Math.sqrt(2 / (gamma - 1) * v * v / (1 - v * v));
        // isentropic from just behind the shock (its total pressure) to the surface
        double p2 = shock.pressureRatio(), m2 = shock.machBehind();
        double pSurface = p2 * PerfectGas.totalToStaticPressure(m2, gamma) / PerfectGas.totalToStaticPressure(machSurface, gamma);
        double tSurface = PerfectGas.totalToStaticTemperature(mach, gamma) / PerfectGas.totalToStaticTemperature(machSurface, gamma);
        return new ConicalShock(mach, theta, waveAngle, machSurface, pSurface, tSurface, shock.totalPressureRatio());
    }

    /** The cone half-angle that holds a shock at wave angle beta. */
    static double coneAngle(double mach, double waveAngle, double gamma) { return integrate(mach, waveAngle, gamma)[0]; }

    /** {cone half-angle, V' at the surface}: Taylor-Maccoll from the shock inward until V'theta = 0. */
    private static double[] integrate(double mach, double beta, double gamma) {
        ObliqueShock shock = ObliqueShock.ofWaveAngle(mach, beta, gamma);
        double m2 = shock.machBehind(), delta = shock.deflection();
        double v = 1 / Math.sqrt(2 / ((gamma - 1) * m2 * m2) + 1);
        double vr = v * Math.cos(beta - delta), vt = -v * Math.sin(beta - delta);
        if (vt >= 0) { return new double[]{0, v}; }
        double theta = beta, h = -beta / STEPS;
        for (int i = 0; i < STEPS; i++) {
            double[] y = {vr, vt};
            double[] k1 = rhs(theta, y, gamma);
            double[] k2 = rhs(theta + h / 2, new double[]{y[0] + h / 2 * k1[0], y[1] + h / 2 * k1[1]}, gamma);
            double[] k3 = rhs(theta + h / 2, new double[]{y[0] + h / 2 * k2[0], y[1] + h / 2 * k2[1]}, gamma);
            double[] k4 = rhs(theta + h, new double[]{y[0] + h * k3[0], y[1] + h * k3[1]}, gamma);
            double nvr = vr + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
            double nvt = vt + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
            if (nvt >= 0) {
                // the surface lies inside this step: interpolate V'theta to zero (cubic Hermite in theta)
                double f = vt / (vt - nvt);
                double surf = theta + f * h;
                // refine with a step from theta of length f h
                double hs = f * h;
                double[] q1 = rhs(theta, y, gamma);
                double[] q2 = rhs(theta + hs / 2, new double[]{y[0] + hs / 2 * q1[0], y[1] + hs / 2 * q1[1]}, gamma);
                double[] q3 = rhs(theta + hs / 2, new double[]{y[0] + hs / 2 * q2[0], y[1] + hs / 2 * q2[1]}, gamma);
                double[] q4 = rhs(theta + hs, new double[]{y[0] + hs * q3[0], y[1] + hs * q3[1]}, gamma);
                double svr = vr + hs / 6 * (q1[0] + 2 * q2[0] + 2 * q3[0] + q4[0]);
                double svt = vt + hs / 6 * (q1[1] + 2 * q2[1] + 2 * q3[1] + q4[1]);
                // one Newton correction of the surface ray: dV'theta/dtheta at the surface
                double slope = rhs(surf, new double[]{svr, svt}, gamma)[1];
                double corr = slope != 0 ? -svt / slope : 0;
                return new double[]{surf + corr, Math.hypot(svr + corr * svt, 0)};
            }
            vr = nvr;
            vt = nvt;
            theta += h;
        }
        return new double[]{0, Math.hypot(vr, vt)};
    }

    /** d/dtheta of {V'r, V'theta}, V'theta = dV'r/dtheta. */
    private static double[] rhs(double theta, double[] y, double gamma) {
        double vr = y[0], vt = y[1];
        double a = 0.5 * (gamma - 1) * (1 - vr * vr - vt * vt);
        double vrr = (vt * vt * vr - a * (2 * vr + vt / Math.tan(theta))) / (a - vt * vt);
        return new double[]{vt, vrr};
    }
}

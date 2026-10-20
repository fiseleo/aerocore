package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Pressure on a body's surface in supersonic and hypersonic flow by local-inclination methods (0.28), as pressure
 * coefficients on the free stream's dynamic pressure, air (gamma 1.4):
 * <ul>
 * <li>Tangent cone: a surface inclined theta to the stream carries the surface pressure of a cone of that half-angle
 *   (Taylor-Maccoll, ConicalShock), exact for cones and good on ogives and other bodies of revolution at moderate to
 *   high Mach numbers (Anderson, Hypersonic and High-Temperature Gas Dynamics, sec. 3.4). Built once from a sweep of
 *   wave angles at each tabulated Mach number (one Taylor-Maccoll integration each), interpolated.</li>
 * <li>Modified Newtonian (Lees): Cp_max sin^2 theta, Cp_max the stagnation pressure behind a normal shock (Rayleigh's
 *   pitot formula): for slopes the cone detaches at and for faces square to the flow.</li>
 * <li>Expansion: a surface turning away by theta from the free stream, by Prandtl-Meyer (no lower than a vacuum).</li>
 * <li>Base: the dead air behind a blunt base at -1/M^2 (about seven tenths of a vacuum; projectile and missile base
 *   pressures at Mach 1.5-4, Hoerner Fluid-Dynamic Drag ch. 16), at most -0.6.</li>
 * </ul>
 */
public final class SurfacePressure {
    private SurfacePressure() {}

    static final double GAMMA = 1.4;
    /** Tabulated Mach numbers of the cone table. */
    static final double[] MACHS = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.2, 2.5, 2.8, 3.2, 3.6, 4.0, 4.5, 5, 6, 7, 8, 10, 12, 15, 20};
    /** Wave angles swept per Mach number, cubically spaced from the Mach angle (small cones, where Cp grows as theta^2, close together). */
    private static final int WAVES = 120;

    /** Pressure coefficient of a pressure ratio p/p_inf at a Mach number. */
    public static double cp(double pressureRatio, double mach) { return (pressureRatio - 1) / (0.5 * GAMMA * mach * mach); }

    /** Rayleigh's pitot stagnation pressure coefficient: the largest a surface sees (modified Newtonian's Cp_max); isentropic below Mach 1. */
    public static double stagnation(double mach) {
        if (!(mach > 1)) { return mach < 1e-3 ? 1 : cp(PerfectGas.totalToStaticPressure(mach, GAMMA), mach); }
        double g = GAMMA, m2 = mach * mach;
        double pitot = Math.pow((g + 1) * (g + 1) * m2 / (4 * g * m2 - 2 * (g - 1)), g / (g - 1)) * (1 - g + 2 * g * m2) / (g + 1);
        return cp(pitot, mach);
    }

    /** Modified Newtonian: Cp_max sin^2 theta (theta the surface's inclination to the stream, rad; 0 in its shadow). */
    public static double newtonian(double mach, double theta) {
        double s = Math.sin(Math.clamp(theta, 0, Math.PI / 2));
        return stagnation(mach) * s * s;
    }

    /** A surface turning away from the stream by theta (rad): Prandtl-Meyer from the free stream, no lower than a vacuum. */
    public static double expansion(double mach, double theta) {
        double vacuum = -2 / (GAMMA * mach * mach);
        if (!(mach > 1)) { return Math.max(vacuum, -2 * theta); }
        double nu = PerfectGas.prandtlMeyer(mach, GAMMA) + Math.max(theta, 0);
        if (nu >= PerfectGas.maxPrandtlMeyer(GAMMA)) { return vacuum; }
        double m2 = PerfectGas.machAtPrandtlMeyer(nu, GAMMA);
        double ratio = PerfectGas.totalToStaticPressure(mach, GAMMA) / PerfectGas.totalToStaticPressure(m2, GAMMA);
        return Math.max(vacuum, cp(ratio, mach));
    }

    /**
     * A body of revolution's surface turning away from the stream by theta: the tangent cone's mirror image (slender-body
     * theory makes a boat-tail's suction the cone's pressure with the sign turned, the axisymmetric relief that a
     * two-dimensional expansion lacks), no stronger than Prandtl-Meyer's.
     */
    public static double boattail(double mach, double theta) { return Math.max(expansion(mach, theta), -tangentCone(mach, theta)); }

    /** Base pressure coefficient behind a blunt base (negative). */
    public static double base(double mach) { return -Math.min(0.6, 1 / (mach * mach)); }

    /**
     * Tangent cone: the surface pressure coefficient of a cone of half-angle theta (rad) at Mach M (the table's ends
     * held), modified Newtonian where the shock would detach (and from there on, the larger of the two near it).
     */
    public static double tangentCone(double mach, double theta) {
        if (!(theta > 0)) { return 0; }
        double[] m = MACHS;
        double x = Math.clamp(mach, m[0], m[m.length - 1]);
        int i = 0;
        while (i < m.length - 2 && x > m[i + 1]) { i++; }
        double f = (x - m[i]) / (m[i + 1] - m[i]);
        return (1 - f) * Table.TABLE.at(i, theta, m[i]) + f * Table.TABLE.at(i + 1, theta, m[i + 1]);
    }

    /** The largest cone half-angle with an attached shock at a tabulated Mach number's row, rad (for tests). */
    static double detachment(int row) { return Table.TABLE.cone[row][Table.TABLE.cone[row].length - 1]; }

    /** Per tabulated Mach number: cone angles (rising) and their surface pressure coefficients. Built on first use (about 0.1 s). */
    static final class Table {
        static final Table TABLE = new Table();
        final double[][] cone = new double[MACHS.length][], cps = new double[MACHS.length][];

        private Table() {
            for (int r = 0; r < MACHS.length; r++) {
                double mach = MACHS[r], mu = Math.asin(1 / mach);
                double[] th = new double[WAVES + 1], c = new double[WAVES + 1];
                int k = 1;
                th[0] = 0; c[0] = 0;
                for (int w = 1; w <= WAVES; w++) {
                    double u = (double) w / WAVES, beta = mu + (Math.PI / 2 - mu) * u * u * u * 0.999;
                    ConicalShock s;
                    try { s = ConicalShock.ofWaveAngle(mach, beta, GAMMA); } catch (RuntimeException e) { break; }
                    if (!(s.coneHalfAngle() > th[k - 1])) { break; }   // past the largest cone: the strong branch
                    th[k] = s.coneHalfAngle();
                    c[k] = cp(s.surfacePressureRatio(), mach);
                    k++;
                }
                cone[r] = java.util.Arrays.copyOf(th, k);
                cps[r] = java.util.Arrays.copyOf(c, k);
            }
        }

        double at(int row, double theta, double mach) {
            double[] th = cone[row], c = cps[row];
            int n = th.length;
            if (theta >= th[n - 1]) { return Math.max(c[n - 1], newtonian(mach, theta)); }
            int lo = 0, hi = n - 1;
            while (hi - lo > 1) { int mid = (lo + hi) >>> 1; if (th[mid] <= theta) { lo = mid; } else { hi = mid; } }
            double f = (theta - th[lo]) / (th[hi] - th[lo]);
            return c[lo] + f * (c[hi] - c[lo]);
        }
    }
}

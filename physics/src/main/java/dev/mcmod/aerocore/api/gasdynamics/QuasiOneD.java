package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Generalized steady quasi-one-dimensional flow (Shapiro, The Dynamics and Thermodynamics of Compressible Fluid Flow,
 * ch. 8, "influence coefficients"): area change, heating, wall friction and mass injection acting together along a
 * duct, for a thermally perfect gas. The five differential relations
 * <pre>
 *   continuity  d rho/rho + dV/V              = dw/w - dA/A
 *   momentum    dp/p + gamma M^2 dV/V         = -gamma M^2 (4 f dx / D) / 2 - (1 - y) gamma M^2 dw/w
 *   state       dp/p - d rho/rho - dT/T       = 0
 *   energy      cp T dT/T + V^2 dV/V          = dh0
 * </pre>
 * (y the injectant's axial velocity over the stream's) give dV/V = (dh0 / (cp T) - (c2 - c1)) / (1 - M^2), singular at
 * Mach 1: a duct that drives the flow through Mach 1 anywhere but at a throat is choked. Integrated with RK4 in x on
 * (ln p, ln T, ln V); the stream's total enthalpy rises by the given heat (the caller accounts mixing).
 */
public final class QuasiOneD {
    private QuasiOneD() {}

    /**
     * A duct piece: area linear from in to out, heat added uniformly (J per kg of the inflowing stream), wall friction
     * (Fanning f over hydraulic diameter D; NaN D means a circle of the local area), mass added uniformly (a fraction of
     * the inflow) with an axial velocity of y times the stream's.
     */
    public record Segment(double lengthM, double areaInM2, double areaOutM2, double heatJPerKg, double frictionFactor, double hydraulicDiameterM,
                          double massAddedFraction, double injectantVelocityRatio) {
        public Segment {
            if (!(lengthM > 0) || !(areaInM2 > 0) || !(areaOutM2 > 0) || !Double.isFinite(heatJPerKg) || !(frictionFactor >= 0)
                    || !(massAddedFraction >= 0) || !Double.isFinite(injectantVelocityRatio)) {
                throw new IllegalArgumentException("A duct segment needs positive length and areas, finite heat, nonnegative friction and injection");
            }
        }
        /** Area change only. */
        public static Segment nozzle(double lengthM, double areaInM2, double areaOutM2) { return new Segment(lengthM, areaInM2, areaOutM2, 0, 0, Double.NaN, 0, 0); }
        /** Heat only, constant area. */
        public static Segment heated(double lengthM, double areaM2, double heatJPerKg) { return new Segment(lengthM, areaM2, areaM2, heatJPerKg, 0, Double.NaN, 0, 0); }
        /** Friction only, constant area. */
        public static Segment rough(double lengthM, double areaM2, double frictionFactor, double hydraulicDiameterM) {
            return new Segment(lengthM, areaM2, areaM2, 0, frictionFactor, hydraulicDiameterM, 0, 0);
        }
        double area(double x) { return areaInM2 + (areaOutM2 - areaInM2) * x / lengthM; }
        double diameter(double x) { return Double.isNaN(hydraulicDiameterM) ? Math.sqrt(4 * area(x) / Math.PI) : hydraulicDiameterM; }
    }

    /** The exit state (or where the flow choked), the mass flow over the inflow, and whether it choked before the end (at x). */
    public record Result(GasState exit, double massFlowRatio, boolean choked, double chokedAtM) {}

    /** Marches the stream entering at the given state through the segment in a number of steps. */
    public static Result march(Thermo gas, Segment seg, GasState inlet, int steps) {
        if (steps < 1) { throw new IllegalArgumentException("At least one step"); }
        double h = seg.lengthM() / steps;
        double[] y = {Math.log(inlet.pressurePa()), Math.log(inlet.temperatureK()), Math.log(Math.max(1e-9, inlet.speedMps()))};
        double x = 0;
        boolean supersonic = inlet.mach(gas) > 1;
        for (int i = 0; i < steps; i++) {
            double[] k1 = rhs(gas, seg, x, y), k2 = rhs(gas, seg, x + h / 2, add(y, k1, h / 2)), k3 = rhs(gas, seg, x + h / 2, add(y, k2, h / 2)),
                    k4 = rhs(gas, seg, x + h, add(y, k3, h));
            double[] next = new double[3];
            boolean bad = false;
            for (int j = 0; j < 3; j++) {
                next[j] = y[j] + h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
                // a log past exp's range (the step jumped over the sonic singularity) is as bad as no number
                bad |= !Double.isFinite(next[j]) || Math.abs(next[j]) > 700;
            }
            double machNext = bad ? Double.NaN : Math.exp(next[2]) / gas.speedOfSound(Math.exp(next[1]));
            if (bad || (machNext > 1) != supersonic) {
                // the flow reaches Mach 1 inside this step: bisect the step length on the sign of 1 - M^2
                double lo = 0, hi = h;
                for (int k = 0; k < 50; k++) {
                    double s = 0.5 * (lo + hi);
                    double[] trial = euler(gas, seg, x, y, s);
                    double m = trial == null ? Double.NaN : Math.exp(trial[2]) / gas.speedOfSound(Math.exp(trial[1]));
                    if (trial != null && (m > 1) == supersonic && Double.isFinite(m)) { lo = s; } else { hi = s; }
                }
                double[] at = euler(gas, seg, x, y, lo);
                if (at == null) { at = y; }
                return new Result(state(at), 1 + seg.massAddedFraction() * (x + lo) / seg.lengthM(), true, x + lo);
            }
            y = next;
            x += h;
        }
        return new Result(state(y), 1 + seg.massAddedFraction(), false, Double.NaN);
    }

    private static GasState state(double[] y) { return new GasState(Math.exp(y[1]), Math.exp(y[0]), Math.exp(y[2])); }

    /** RK4 sub-steps of length s from x (for the bisection near choking); null if it fails. */
    private static double[] euler(Thermo gas, Segment seg, double x, double[] y, double s) {
        int n = 8;
        double hs = s / n;
        double[] z = y.clone();
        for (int i = 0; i < n; i++) {
            double[] k1 = rhs(gas, seg, x, z), k2 = rhs(gas, seg, x + hs / 2, add(z, k1, hs / 2)), k3 = rhs(gas, seg, x + hs / 2, add(z, k2, hs / 2)),
                    k4 = rhs(gas, seg, x + hs, add(z, k3, hs));
            for (int j = 0; j < 3; j++) { z[j] += hs / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]); }
            for (double v : z) { if (!Double.isFinite(v) || Math.abs(v) > 700) { return null; } }
            x += hs;
        }
        return z;
    }

    private static double[] add(double[] y, double[] k, double s) { return new double[]{y[0] + s * k[0], y[1] + s * k[1], y[2] + s * k[2]}; }

    /** d/dx of {ln p, ln T, ln V}. */
    private static double[] rhs(Thermo gas, Segment seg, double x, double[] y) {
        double t = Math.exp(y[1]), v = Math.exp(y[2]);
        double cp = gas.cp(t), g = gas.gamma(t), r = gas.gasConstant();
        double m2 = v * v / (g * r * t);
        double a = seg.area(x), dlnA = (seg.areaOutM2() - seg.areaInM2()) / seg.lengthM() / a;
        double w = 1 + seg.massAddedFraction() * x / seg.lengthM(), dlnW = seg.massAddedFraction() / seg.lengthM() / w;
        double friction = 4 * seg.frictionFactor() / seg.diameter(x);
        double dh0 = seg.heatJPerKg() / seg.lengthM() / w;   // heat per kg of the stream now (per metre)
        double c1 = dlnW - dlnA;
        double c2 = -g * m2 * 0.5 * friction - (1 - seg.injectantVelocityRatio()) * g * m2 * dlnW;
        double dlnV = (dh0 / (cp * t) - (c2 - c1)) / (1 - m2);
        double dlnT = c2 - c1 - (g * m2 - 1) * dlnV;
        double dlnP = c2 - g * m2 * dlnV;
        return new double[]{dlnP, dlnT, dlnV};
    }
}

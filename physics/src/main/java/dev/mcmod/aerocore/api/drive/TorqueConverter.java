package dev.mcmod.aerocore.api.drive;

/**
 * A hydrodynamic torque converter's characteristic (0.24): pump torque T_P = (omega_P / K(nu))^2 and turbine torque
 * mu(nu) T_P at the speed ratio nu = omega_T / omega_P (SAE J643's quantities).
 *
 * <p>The shape is MathWorks' default converter table (passenger-car-like; nu 0, 0.5, 0.6, 0.7, 0.8, 0.87, 0.92, 0.94,
 * 0.96, 0.97: torque ratio 2.232 to 0.9983, K 12.29 to 50 rad/s per sqrt(N m)), stretched to a design's stall torque ratio
 * (mu - 1 scaled; passenger cars 1.8-2.2, Schaeffler US 8,403,118) and stall K (K scaled; the table's is 117 rpm per
 * sqrt(N m)); K is nearly flat to nu 0.6 and the torque ratio meets 1 near nu 0.94 (GM US 9,879,769). Above 0.97 the
 * torque falls linearly to none at nu = 1. Turbine faster than pump (coasting): a fluid coupling the other way, the
 * turbine driving the pump with the coupling end of the table (the stator freewheels: no multiplication). Turbine
 * turning backward against the pump: stalled harder, the stall values held.
 *
 * <p>Converter capacity scales as rho omega^2 D^5 (Naunheimer et al.), so K scales as D^(-5/2): K_stall(D) =
 * K_ref (D_ref / D)^2.5 with the table taken for a 0.26 m converter (rule: the size of a passenger car's).
 */
public final class TorqueConverter {
    static final double[] NU = {0, 0.5, 0.6, 0.7, 0.8, 0.87, 0.92, 0.94, 0.96, 0.97};
    static final double[] RATIO = {2.232, 1.5462, 1.4058, 1.2746, 1.1528, 1.0732, 1.0192, 0.9983, 0.9983, 0.9983};
    static final double[] K = {12.29, 12.86, 13.15, 13.63, 14.62, 16.27, 19.35, 22.10, 30.00, 50};
    /** The table's converter diameter (rule), m. */
    public static final double REFERENCE_DIAMETER = 0.26;

    private final double stallRatio, stallK;

    /** A converter of a stall torque ratio (above 1) and stall capacity factor K (rad/s per sqrt(N m)). */
    public TorqueConverter(double stallTorqueRatio, double stallK) {
        if (!(stallTorqueRatio >= 1 && stallTorqueRatio < 5) || !(stallK > 0)) { throw new IllegalArgumentException("Stall ratio 1-5 and K positive"); }
        this.stallRatio = stallTorqueRatio; this.stallK = stallK;
    }

    /** The stall K of a converter of a diameter (m) with the table's shape. */
    public static double stallK(double diameterM) { return K[0] * Math.pow(REFERENCE_DIAMETER / diameterM, 2.5); }

    public double stallTorqueRatio() { return stallRatio; }
    public double stallK() { return stallK; }

    /** Torque ratio at a speed ratio 0..1. */
    public double torqueRatio(double nu) {
        double base = interpolate(RATIO, nu), last = RATIO[RATIO.length - 1];
        return last + (stallRatio - last) * (base - last) / (RATIO[0] - last);
    }

    /** Capacity factor at a speed ratio 0..0.97, rad/s per sqrt(N m). */
    public double capacity(double nu) { return stallK * interpolate(K, nu) / K[0]; }

    /** Pump torque absorbed at pump speed and speed ratio (forward flow), N m (positive opposing the pump). */
    public double pumpTorque(double pumpRadPerSecond, double nu) {
        double n = Math.clamp(nu, 0, 1), w = pumpRadPerSecond;
        double last = NU[NU.length - 1];
        double t = Math.pow(w / capacity(Math.min(n, last)), 2);
        return n <= last ? t : t * (1 - n) / (1 - last);
    }

    /**
     * Torques {on the pump, on the turbine} (N m, about their common sense of rotation) at pump and turbine speeds
     * (rad/s). Forward flow: the pump is loaded (negative), the turbine driven (positive); coasting the other way.
     */
    public double[] torques(double pump, double turbine) {
        double sign = pump != 0 ? Math.signum(pump) : Math.signum(turbine);
        if (sign == 0) { return new double[]{0, 0}; }
        double p = pump * sign, t = turbine * sign;
        if (p >= t) {
            if (p <= 0) { return new double[]{0, 0}; }
            double nu = t / p, load = pumpTorque(p, Math.max(nu, 0));
            return new double[]{-sign * load, sign * torqueRatio(Math.max(nu, 0)) * load};
        }
        // the turbine overruns: a coupling driving the pump, no torque multiplication
        if (t <= 0) { return new double[]{0, 0}; }
        double nu = Math.max(p, 0) / t, load = pumpTorque(t, nu);
        return new double[]{sign * load, -sign * load};
    }

    private static double interpolate(double[] values, double nu) {
        double n = Math.clamp(nu, 0, NU[NU.length - 1]);
        for (int i = 1; i < NU.length; i++) {
            if (n <= NU[i]) {
                double f = (n - NU[i - 1]) / (NU[i] - NU[i - 1]);
                return values[i - 1] + f * (values[i] - values[i - 1]);
            }
        }
        return values[values.length - 1];
    }
}

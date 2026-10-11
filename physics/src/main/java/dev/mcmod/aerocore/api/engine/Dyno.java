package dev.mcmod.aerocore.api.engine;

import java.util.ArrayList;
import java.util.List;

/**
 * Operating points of a cylinder spec the way an engine calibration finds them: at each speed and manifold pressure
 * the spark is set for best torque (MBT, golden-section search on net IMEP), then, with knock control, retarded until
 * the end-gas knock integral at 90 % burned stays below the margin. Without knock control the spark stays at MBT
 * (capped by the ignition system's maximum advance) and the point is reported as knocking.
 */
public final class Dyno {
    private Dyno() {}

    /** Cycles each spark trial runs from the point's settled state (a changed spark moves residual and exhaust temperature a little). */
    static final int TRIAL_CYCLES = 2;

    /** Spark strategy: knock control (sensor and retard), the largest advance the ignition can give, the knock integral allowed. */
    public record SparkControl(boolean knockControl, double maxAdvanceDeg, double knockMargin) {
        public SparkControl {
            if (!(maxAdvanceDeg > 0) || maxAdvanceDeg > 80) { throw new IllegalArgumentException("Maximum advance must be in (0, 80] degrees"); }
            if (!(knockMargin > 0)) { throw new IllegalArgumentException("Knock margin must be positive"); }
        }
        public static final SparkControl STANDARD = new SparkControl(true, 50, 1);
    }

    /** One calibrated point: the spark chosen, whether knock limited it, and the cycle there. */
    public record Point(double rpm, double manifoldPressurePa, double sparkAdvanceDeg, double mbtAdvanceDeg, boolean knockLimited,
                        boolean knocking, EngineCycle.Result cycle) {}

    /** Calibrates one point; exhaust back pressure from the exhaust system (Pa above ambient) and ambient pressure. */
    public static Point operate(EngineCycle.Spec spec, double rpm, double manifoldPressurePa, double intakeTemperatureK,
                                double exhaustPressurePa, SparkControl control) {
        EngineCycle.Conditions base = new EngineCycle.Conditions(rpm, manifoldPressurePa, intakeTemperatureK, exhaustPressurePa, 20);
        EngineCycle.Run warm = EngineCycle.run(spec, base, null, 3);
        java.util.function.DoubleFunction<EngineCycle.Result> at = adv ->
                EngineCycle.run(spec, new EngineCycle.Conditions(rpm, manifoldPressurePa, intakeTemperatureK, exhaustPressurePa, adv), warm.warm(), TRIAL_CYCLES).result();
        // golden-section search for MBT on [0, max]
        double lo = 0, hi = control.maxAdvanceDeg(), g = (Math.sqrt(5) - 1) / 2;
        double a = hi - g * (hi - lo), b = lo + g * (hi - lo);
        EngineCycle.Result ra = at.apply(a), rb = at.apply(b);
        while (hi - lo > 1) {
            if (ra.netImep() > rb.netImep()) { hi = b; b = a; rb = ra; a = hi - g * (hi - lo); ra = at.apply(a); }
            else { lo = a; a = b; ra = rb; b = lo + g * (hi - lo); rb = at.apply(b); }
        }
        double mbt = ra.netImep() > rb.netImep() ? a : b;
        double advance = mbt;
        boolean limited = false;
        if (control.knockControl() && at.apply(mbt).knockMargin() >= control.knockMargin()) {
            limited = true;
            double safe = -10, unsafe = mbt;
            if (at.apply(safe).knockMargin() >= control.knockMargin()) { advance = safe; }
            else {
                while (unsafe - safe > 0.5) {
                    double mid = 0.5 * (safe + unsafe);
                    if (at.apply(mid).knockMargin() >= control.knockMargin()) { unsafe = mid; } else { safe = mid; }
                }
                advance = safe;
            }
        }
        EngineCycle.Result r = EngineCycle.run(spec, new EngineCycle.Conditions(rpm, manifoldPressurePa, intakeTemperatureK, exhaustPressurePa, advance),
                warm.warm(), 2).result();
        return new Point(rpm, manifoldPressurePa, advance, mbt, limited, r.knockMargin() >= control.knockMargin(), r);
    }

    /** Points at the given speeds for one manifold pressure (a full-load curve when it is the ambient pressure). */
    public static List<Point> curve(EngineCycle.Spec spec, double[] rpms, double manifoldPressurePa, double intakeTemperatureK,
                                    java.util.function.DoubleUnaryOperator exhaustPressureAtRpm, SparkControl control) {
        List<Point> points = new ArrayList<>();
        for (double rpm : rpms) { points.add(operate(spec, rpm, manifoldPressurePa, intakeTemperatureK, exhaustPressureAtRpm.applyAsDouble(rpm), control)); }
        return points;
    }
}

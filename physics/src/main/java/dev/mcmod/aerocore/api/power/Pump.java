package dev.mcmod.aerocore.api.power;

import java.util.function.DoubleUnaryOperator;

/**
 * A centrifugal pump (0.26): its head falls with the flow as H0 (1 - (Q / Qmax)^2), the usual parabolic fit of a
 * pump curve; its hydraulic power peaks at Qmax / sqrt 3, taken as the best efficiency point, the efficiency falling
 * as a parabola either side; the duty point is where the curve meets the system's head (Karassik, Pump Handbook ch. 2;
 * Gülich, Centrifugal Pumps ch. 4). Heads in metres of the pumped liquid.
 */
public record Pump(double shutoffHeadM, double runoutFlowM3PerS, double bestEfficiency) {
    public static final double G = 9.80665;

    public Pump {
        if (!(shutoffHeadM > 0) || !(runoutFlowM3PerS > 0) || !(bestEfficiency > 0 && bestEfficiency <= 1)) { throw new IllegalArgumentException("Pump: positive head and flow, efficiency in (0, 1]"); }
    }

    /** A pump whose best efficiency point is a flow (m^3/s) at a head (m). */
    public static Pump forDuty(double flowM3PerS, double headM, double efficiency) { return new Pump(1.5 * headM, Math.sqrt(3) * flowM3PerS, efficiency); }

    /** Head at a flow, m (none past the runout). */
    public double headM(double flowM3PerS) {
        double x = Math.max(flowM3PerS, 0) / runoutFlowM3PerS;
        return Math.max(0, shutoffHeadM * (1 - x * x));
    }

    /** The best efficiency point's flow, m^3/s. */
    public double bestFlowM3PerS() { return runoutFlowM3PerS / Math.sqrt(3); }

    /** Efficiency at a flow (at least 5 % while it pumps). */
    public double efficiency(double flowM3PerS) {
        double x = Math.max(flowM3PerS, 0) / bestFlowM3PerS() - 1;
        return Math.max(0.05, bestEfficiency * (1 - x * x));
    }

    /** Shaft power at a flow of a liquid of a density, W. */
    public double shaftPowerW(double flowM3PerS, double density) {
        return Math.max(flowM3PerS, 0) <= 0 ? 0 : density * G * flowM3PerS * headM(flowM3PerS) / efficiency(flowM3PerS);
    }

    /** The flow where the pump's head meets the system's head curve (m at a flow), m^3/s; 0 when the system needs more than its shutoff head. */
    public double dutyFlowM3PerS(DoubleUnaryOperator systemHeadM) {
        if (systemHeadM.applyAsDouble(0) >= shutoffHeadM) { return 0; }
        double lo = 0, hi = runoutFlowM3PerS;
        for (int i = 0; i < 80; i++) {
            double mid = 0.5 * (lo + hi);
            if (headM(mid) > systemHeadM.applyAsDouble(mid)) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }
}

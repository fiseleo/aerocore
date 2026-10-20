package dev.mcmod.aerocore.api.engine;

import java.util.function.DoubleUnaryOperator;

/**
 * A turbocharger (0.26): a centrifugal compressor driven by an exhaust turbine on one shaft, a wastegate that bypasses
 * the turbine to hold the boost, an intercooler (Heywood 1988, chapter 6.8; Watson and Janota, Turbocharging the
 * Internal Combustion Engine, 1982).
 * <ul>
 * <li>Compressor: polytropic compression (Turbomachinery.compressorWork); the most pressure ratio its impeller gives at
 *     a speed is the one whose work is the slip-limited Euler work sigma U^2 (Stanitz's slip factor).</li>
 * <li>Turbine: an orifice of an effective area for the gas (ValveFlow, exhaust R 287, gamma 1.33) and an expansion of
 *     an isentropic efficiency (cp 1150 J/kg K). With the wastegate shut all the exhaust goes through it, which sets the
 *     pressure in front of it; the wastegate opens to give the compressor just the power it needs, at a lower pressure
 *     and with the rest of the gas let past.</li>
 * <li>Intercooler: the charge cooled toward the coolant by an effectiveness.</li>
 * </ul>
 */
public record Turbocharger(double compressorTipM, int compressorBlades, double compressorPolytropicEfficiency, double turbineAreaM2,
                           double turbineEfficiency, double mechanicalEfficiency, double maxPressureRatio, double intercoolerEffectiveness,
                           double rotorInertiaKgM2) {
    public Turbocharger {
        if (!(compressorTipM > 0) || compressorBlades < 3 || !(turbineAreaM2 > 0) || !(maxPressureRatio > 1) || !(rotorInertiaKgM2 > 0)) {
            throw new IllegalArgumentException("Turbocharger: positive sizes, at least 3 blades, a pressure ratio above 1");
        }
        if (!(compressorPolytropicEfficiency > 0 && compressorPolytropicEfficiency <= 1) || !(turbineEfficiency > 0 && turbineEfficiency <= 1)
                || !(mechanicalEfficiency > 0 && mechanicalEfficiency <= 1) || !(intercoolerEffectiveness >= 0 && intercoolerEffectiveness < 1)) {
            throw new IllegalArgumentException("Turbocharger efficiencies in (0, 1]");
        }
    }

    public static final double EXHAUST_CP = 1150, EXHAUST_GAMMA = 1.33, EXHAUST_R = 287;

    /** Work to compress a kg of air by a ratio from an inlet temperature, J/kg. */
    public double compressorWorkJPerKg(double ratio, double inletK) {
        return ratio <= 1 ? 0 : Turbomachinery.compressorWork(ratio, compressorPolytropicEfficiency, inletK);
    }

    /** Compressor power for an air flow (kg/s) at a ratio, W. */
    public double compressorPowerW(double airKgPerS, double ratio, double inletK) { return Math.max(airKgPerS, 0) * compressorWorkJPerKg(ratio, inletK); }

    /** Charge temperature after the compressor and intercooler, K. */
    public double chargeK(double ratio, double inletK, double coolantK) {
        double t2 = ratio <= 1 ? inletK : Turbomachinery.compressorExitTemperature(ratio, compressorPolytropicEfficiency, inletK);
        return t2 - intercoolerEffectiveness * Math.max(0, t2 - coolantK);
    }

    /** The most pressure ratio the impeller gives at a speed (rpm): the ratio whose work is sigma U^2, capped by the wastegate setting. */
    public double pressureRatioAt(double rpm, double inletK) {
        double u = Turbomachinery.tipSpeed(compressorTipM, rpm), work = Turbomachinery.slipFactor(compressorBlades) * u * u;
        double lo = 1, hi = maxPressureRatio;
        if (compressorWorkJPerKg(hi, inletK) <= work) { return hi; }
        for (int i = 0; i < 50; i++) {
            double mid = 0.5 * (lo + hi);
            if (compressorWorkJPerKg(mid, inletK) > work) { hi = mid; } else { lo = mid; }
        }
        return 0.5 * (lo + hi);
    }

    /** Turbine work per kg expanding from an inlet temperature by an expansion ratio, J/kg. */
    public double turbineWorkJPerKg(double inletK, double expansionRatio) {
        if (expansionRatio <= 1) { return 0; }
        return EXHAUST_CP * inletK * turbineEfficiency * (1 - Math.pow(expansionRatio, -(EXHAUST_GAMMA - 1) / EXHAUST_GAMMA));
    }

    /** Gas the turbine passes from an inlet pressure and temperature to an outlet pressure, kg/s. */
    public double turbineFlowKgPerS(double inletPa, double inletK, double outletPa) {
        return ValveFlow.massFlow(turbineAreaM2, inletPa, inletK, outletPa, EXHAUST_R, EXHAUST_GAMMA);
    }

    /** Pressure in front of the turbine that passes a flow (kg/s) with the wastegate shut, Pa. */
    public double fullFlowInletPa(double exhaustKgPerS, double inletK, double outletPa) {
        if (exhaustKgPerS <= 0) { return outletPa; }
        double lo = outletPa, hi = outletPa * 2;
        while (turbineFlowKgPerS(hi, inletK, outletPa) < exhaustKgPerS && hi < 1e8) { hi *= 2; }
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (turbineFlowKgPerS(mid, inletK, outletPa) < exhaustKgPerS) { lo = mid; } else { hi = mid; }
        }
        return 0.5 * (lo + hi);
    }

    /** How the turbine meets a compressor's power: the pressure in front of it, the share of the exhaust it takes, and whether it could. */
    public record Match(double turbineInletPa, double turbineShare, boolean reached) {}

    /** The turbine giving a compressor power (W) from an exhaust flow (kg/s) and temperature (K) into an outlet pressure. */
    public Match match(double compressorW, double exhaustKgPerS, double exhaustK, double outletPa) {
        double full = fullFlowInletPa(exhaustKgPerS, exhaustK, outletPa);
        double available = mechanicalEfficiency * exhaustKgPerS * turbineWorkJPerKg(exhaustK, full / outletPa);
        if (available <= compressorW) { return new Match(full, 1, available >= compressorW - 1e-9); }
        // the wastegate opens: a lower pressure in front, the turbine taking what passes its area at that pressure
        double lo = outletPa, hi = full;
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            double power = mechanicalEfficiency * turbineFlowKgPerS(mid, exhaustK, outletPa) * turbineWorkJPerKg(exhaustK, mid / outletPa);
            if (power < compressorW) { lo = mid; } else { hi = mid; }
        }
        double p = 0.5 * (lo + hi);
        return new Match(p, Math.min(1, turbineFlowKgPerS(p, exhaustK, outletPa) / exhaustKgPerS), true);
    }

    /**
     * The most boost (pressure ratio) the turbine sustains with the wastegate shut, for an engine whose air flow (kg/s)
     * and exhaust temperature (K) at a boost ratio are given, burning a fuel-to-air ratio, between the ambient and
     * the wastegate setting.
     */
    public double maxBoost(DoubleUnaryOperator airAtRatio, DoubleUnaryOperator exhaustKAtRatio, double fuelPerAir, double ambientPa, double ambientK) {
        java.util.function.DoublePredicate holds = ratio -> {
            double air = airAtRatio.applyAsDouble(ratio);
            return match(compressorPowerW(air, ratio, ambientK), air * (1 + fuelPerAir), exhaustKAtRatio.applyAsDouble(ratio), ambientPa).reached();
        };
        if (holds.test(maxPressureRatio)) { return maxPressureRatio; }
        double lo = 1, hi = maxPressureRatio;
        for (int i = 0; i < 40; i++) {
            double mid = 0.5 * (lo + hi);
            if (holds.test(mid)) { lo = mid; } else { hi = mid; }
        }
        return lo;
    }
}

package dev.mcmod.aerocore.api.engine;

import java.util.stream.IntStream;

/**
 * A diesel's performance map (0.26) over speed (rpm), boost (intake pressure over ambient; 1 only for a naturally
 * aspirated engine) and load (fuel-air equivalence ratio phi = 1 / lambda; a diesel is unthrottled, its fuel is its
 * load), per cylinder per cycle: every point calibrated by Dyno (injection at best torque, retarded to the peak
 * pressure limit), its exhaust back pressure from the engine's turbocharger matched to the point's flows (one
 * correction). Points as EngineMap's: pressureRatio is the boost, sparkAdvanceDeg the start of injection, knockLimited
 * limited by peak pressure.
 */
public record DieselMap(double[] rpms, double[] boosts, double[] loads, EngineMap.Point[][][] points) {

    /** What the map needs from a diesel. */
    public interface Engine {
        /** One cylinder's cycle at a lambda (Spec with compressionIgnition). */
        EngineCycle.Spec spec(double lambda);
        /** Charge temperature in the manifold at a boost ratio, K. */
        double intakeTemperature(double ambientK, double boostRatio);
        /** Exhaust port pressure at a speed for an intake pressure, the engine's air flow (kg/s, 0 for a first guess) and exhaust temperature, Pa. */
        double exhaustPressure(double ambientPa, double intakePa, double rpm, double airKgPerS, double exhaustK);
        /** Most fuel the injectors give per cylinder per cycle at a speed, kg; infinite without a limit. */
        double fuelLimitKg(double rpm);
    }

    /** The grid, the latest injection advance tried, the ambient the map is computed at and the cylinder count (for flows). */
    public record Calibration(double[] rpms, double[] boosts, double[] loads, double maxAdvanceDeg, double ambientPa, double ambientK, int cylinders) {
        public Calibration {
            rpms = rpms.clone(); boosts = boosts.clone(); loads = loads.clone();
            if (rpms.length < 2 || loads.length < 2 || boosts.length < 1) { throw new IllegalArgumentException("A diesel map needs two speeds, two loads and a boost"); }
            for (double l : loads) { if (!(l > 0 && l <= 1)) { throw new IllegalArgumentException("Loads are equivalence ratios in (0, 1]"); } }
        }
    }

    public EngineMap.Point at(int i, int j, int k) { return points[i][j][k]; }

    /** Trilinear interpolation of one quantity at a speed, boost and load (clamped to the map). */
    public double value(double rpm, double boost, double load, java.util.function.ToDoubleFunction<EngineMap.Point> q) {
        int i = index(rpms, rpm), j = index(boosts, boost), k = index(loads, load);
        double fi = fraction(rpms, i, rpm), fj = fraction(boosts, j, boost), fk = fraction(loads, k, load);
        int j1 = Math.min(j + 1, boosts.length - 1), i1 = i + 1, k1 = k + 1;
        double v = 0;
        for (int a = 0; a < 2; a++) {
            for (int b = 0; b < 2; b++) {
                for (int c = 0; c < 2; c++) {
                    double w = (a == 0 ? 1 - fi : fi) * (b == 0 ? 1 - fj : fj) * (c == 0 ? 1 - fk : fk);
                    if (w != 0) { v += w * q.applyAsDouble(points[a == 0 ? i : i1][b == 0 ? j : j1][c == 0 ? k : k1]); }
                }
            }
        }
        return v;
    }

    private static int index(double[] grid, double x) {
        if (grid.length == 1) { return 0; }
        int i = 0;
        while (i < grid.length - 2 && x > grid[i + 1]) { i++; }
        return i;
    }

    private static double fraction(double[] grid, int i, double x) {
        if (grid.length == 1) { return 0; }
        return Math.clamp((x - grid[i]) / (grid[i + 1] - grid[i]), 0, 1);
    }

    /** Builds the map (seconds to tens of seconds; the speeds in parallel). */
    public static DieselMap build(Engine engine, Calibration c) {
        double[] rpms = c.rpms(), boosts = c.boosts(), loads = c.loads();
        EngineMap.Point[][][] points = new EngineMap.Point[rpms.length][boosts.length][loads.length];
        IntStream.range(0, rpms.length).parallel().forEach(i -> {
            for (int j = 0; j < boosts.length; j++) {
                for (int k = 0; k < loads.length; k++) { points[i][j][k] = point(engine, c, rpms[i], boosts[j], loads[k]); }
            }
        });
        return new DieselMap(rpms, boosts, loads, points);
    }

    private static EngineMap.Point point(Engine engine, Calibration c, double rpm, double boost, double load) {
        double pIn = boost * c.ambientPa(), tIn = engine.intakeTemperature(c.ambientK(), boost);
        double lambda = 1 / load;
        Dyno.SparkControl control = new Dyno.SparkControl(false, c.maxAdvanceDeg(), 1);
        double pEx = engine.exhaustPressure(c.ambientPa(), pIn, rpm, 0, 0);
        Dyno.Point p = Dyno.operate(engine.spec(lambda), rpm, pIn, tIn, pEx, control);
        double limit = engine.fuelLimitKg(rpm);
        if (p.cycle().fuelKg() > limit) {
            lambda = Math.min(12, lambda * p.cycle().fuelKg() / limit);
            p = Dyno.operate(engine.spec(lambda), rpm, pIn, tIn, pEx, control);
        }
        // the turbocharger's back pressure for this point's flows, then once more
        double air = p.cycle().inductedAirKg() * c.cylinders() * rpm / 120;
        double corrected = engine.exhaustPressure(c.ambientPa(), pIn, rpm, air, p.cycle().exhaustTemperatureK());
        if (Math.abs(corrected - pEx) > 0.01 * pEx) { p = Dyno.operate(engine.spec(lambda), rpm, pIn, tIn, corrected, control); }
        EngineCycle.Result r = p.cycle();
        return new EngineMap.Point(rpm, boost, r.netImep(), r.pumpingMep(), r.inductedAirKg(), r.fuelKg(), r.heatToWallsJ(), r.exhaustTemperatureK(),
                p.sparkAdvanceDeg(), p.knockLimited(), p.knocking(), r.peakPressurePa(), lambda, 0, 0);
    }
}

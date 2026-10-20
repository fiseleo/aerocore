package dev.mcmod.aerocore.api.engine;

import java.util.stream.IntStream;

/**
 * A spark-ignition engine's performance map over speed (rpm) and manifold pressure ratio (manifold over ambient),
 * per cylinder per cycle: every point calibrated by Dyno (spark at best torque or the knock limit), at the engine's
 * target lambda, leaner where its injectors run out; cam phases chosen for best full-load torque at each speed (kept at
 * part load). Immutable; rows are speeds, columns pressure ratios.
 */
public record EngineMap(double[] rpms, double[] pressures, Point[][] points) {

    /** One map point, per cylinder per cycle. */
    public record Point(double rpm, double pressureRatio, double netImepPa, double pumpingMepPa, double airKg, double fuelKg, double wallHeatJ,
                        double exhaustTemperatureK, double sparkAdvanceDeg, boolean knockLimited, boolean knocking, double peakPressurePa,
                        double lambda, double intakePhaseDeg, double exhaustPhaseDeg) {}

    /** What the map needs from an engine. */
    public interface Engine {
        /** One cylinder's cycle at a lambda and cam phases (crank degrees later). */
        EngineCycle.Spec spec(double lambda, double intakePhaseDeg, double exhaustPhaseDeg);
        /** Exhaust port pressure for an ambient and manifold pressure at a speed (rpm), Pa absolute. */
        double exhaustPressure(double ambientPa, double manifoldPa, double rpm);
        /** Most fuel the injectors give per cylinder per cycle at a speed and manifold pressure, kg; infinite without a limit. */
        double fuelLimitKg(double rpm, double manifoldPa);
        /** Target lambda at a manifold pressure ratio. */
        double lambda(double pressureRatio);
        /** Charge temperature in the manifold at a pressure ratio (0.26: a turbocharger's intercooled charge), K; the calibration's by default. */
        default double intakeTemperature(double calibrationK, double pressureRatio) { return calibrationK; }
    }

    /**
     * How to calibrate: the grid, spark control, the ambient pressure and intake temperature the map is computed at,
     * the cam phaser ranges (crank degrees either way; 0 for none) and the leanest lambda the injector limit may force.
     */
    public record Calibration(double[] rpms, double[] pressures, Dyno.SparkControl spark, double ambientPa, double intakeK,
                              double intakePhaseRangeDeg, double exhaustPhaseRangeDeg, double maxLambda) {
        public Calibration {
            rpms = rpms.clone(); pressures = pressures.clone();
            if (rpms.length < 2 || pressures.length < 2) { throw new IllegalArgumentException("A map needs at least two speeds and two pressures"); }
        }
    }

    public Point at(int i, int j) { return points[i][j]; }

    /** Bilinear interpolation of one quantity at a speed and pressure ratio (clamped to the map). */
    public double value(double rpm, double ratio, java.util.function.ToDoubleFunction<Point> q) {
        int i = index(rpms, rpm), j = index(pressures, ratio);
        double fi = Math.clamp((rpm - rpms[i]) / (rpms[i + 1] - rpms[i]), 0, 1), fj = Math.clamp((ratio - pressures[j]) / (pressures[j + 1] - pressures[j]), 0, 1);
        double a = q.applyAsDouble(points[i][j]), b = q.applyAsDouble(points[i + 1][j]), c = q.applyAsDouble(points[i][j + 1]), d = q.applyAsDouble(points[i + 1][j + 1]);
        return (1 - fi) * ((1 - fj) * a + fj * c) + fi * ((1 - fj) * b + fj * d);
    }
    private static int index(double[] grid, double x) {
        int i = 0;
        while (i < grid.length - 2 && x > grid[i + 1]) { i++; }
        return i;
    }

    /** Builds the map (seconds of work; the speeds in parallel). */
    public static EngineMap build(Engine engine, Calibration c) {
        double[] rpms = c.rpms(), pressures = c.pressures();
        Point[][] points = new Point[rpms.length][pressures.length];
        IntStream.range(0, rpms.length).parallel().forEach(i -> {
            double rpm = rpms[i];
            double[] phases = bestPhases(engine, c, rpm);
            for (int j = 0; j < pressures.length; j++) { points[i][j] = point(engine, c, rpm, pressures[j], phases[0], phases[1]); }
        });
        return new EngineMap(rpms, pressures, points);
    }

    private static double[] bestPhases(Engine engine, Calibration c, double rpm) {
        double ri = c.intakePhaseRangeDeg(), re = c.exhaustPhaseRangeDeg();
        double bestI = 0, bestE = 0;
        if (ri > 0) {
            double best = Double.NEGATIVE_INFINITY;
            for (double p : new double[]{-ri, -ri / 2, 0, ri / 2, ri}) {
                double imep = point(engine, c, rpm, 1, p, 0).netImepPa();
                if (imep > best) { best = imep; bestI = p; }
            }
        }
        if (re > 0) {
            double best = Double.NEGATIVE_INFINITY;
            for (double p : new double[]{-re, -re / 2, 0, re / 2, re}) {
                double imep = point(engine, c, rpm, 1, bestI, p).netImepPa();
                if (imep > best) { best = imep; bestE = p; }
            }
        }
        return new double[]{bestI, bestE};
    }

    private static Point point(Engine engine, Calibration c, double rpm, double ratio, double phaseI, double phaseE) {
        double pIm = ratio * c.ambientPa();
        double pEx = engine.exhaustPressure(c.ambientPa(), pIm, rpm);
        double lambda = engine.lambda(ratio);
        double tIm = engine.intakeTemperature(c.intakeK(), ratio);
        Dyno.Point p = Dyno.operate(engine.spec(lambda, phaseI, phaseE), rpm, pIm, tIm, pEx, c.spark());
        double limit = engine.fuelLimitKg(rpm, pIm);
        if (p.cycle().fuelKg() > limit) {
            // The injectors run out: the same air with the fuel they can give, a leaner mixture.
            double leaner = Math.min(c.maxLambda(), lambda * p.cycle().fuelKg() / limit);
            p = Dyno.operate(engine.spec(leaner, phaseI, phaseE), rpm, pIm, tIm, pEx, c.spark());
            lambda = leaner;
        }
        EngineCycle.Result r = p.cycle();
        return new Point(rpm, ratio, r.netImep(), r.pumpingMep(), r.inductedAirKg(), r.fuelKg(), r.heatToWallsJ(), r.exhaustTemperatureK(),
                p.sparkAdvanceDeg(), p.knockLimited(), p.knocking(), r.peakPressurePa(), lambda, phaseI, phaseE);
    }
}

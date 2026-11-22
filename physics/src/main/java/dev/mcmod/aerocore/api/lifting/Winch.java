package dev.mcmod.aerocore.api.lifting;

/**
 * A rope drum driven by a motor through a gear: the pitch diameter of each rope layer, the rope a layer holds, and the
 * line pull and rope speed at the layer in use. Formulas only; the drum, motor and gear are the caller's.
 *
 * <p>Layers: the first layer's rope centres lie on the pitch diameter D + d (D the barrel). Each further layer lies in
 * the grooves the layer below forms, its centres on equilateral triangles of side d with two ropes below, so it rises
 * d sqrt(3) / 2 and the pitch diameter grows by sqrt(3) d per layer (NESTED_RISE; the ideal of parallel-grooved,
 * Lebus-type multilayer drums: at the crossovers the rope rides higher). A rise of d (1) is rope stacked straight on
 * rope.
 */
public final class Winch {
    private Winch() {}

    /** Layer rise per rope diameter of a rope nested in the grooves of the layer below: sqrt(3) / 2. */
    public static final double NESTED_RISE = Math.sqrt(3) / 2;
    /** More layers than this are taken as a mistake (layerFor). */
    private static final int MAX_LAYERS = 10_000;

    /** Pitch diameter (m) of a layer (1 the first) of nested rope: D + d + (layer - 1) sqrt(3) d. */
    public static double layerDiameterM(double drumDiameterM, double ropeDiameterM, int layer) {
        return layerDiameterM(drumDiameterM, ropeDiameterM, layer, NESTED_RISE);
    }

    /** Pitch diameter (m) of a layer with the caller's rise per rope diameter (NESTED_RISE to 1): D + d + 2 (layer - 1) rise d. */
    public static double layerDiameterM(double drumDiameterM, double ropeDiameterM, int layer, double rise) {
        Check.positive(drumDiameterM, "Drum diameter"); Check.positive(ropeDiameterM, "Rope diameter");
        Check.atLeastOne(layer, "Layer"); Check.positive(rise, "Layer rise");
        return drumDiameterM + ropeDiameterM + 2.0 * (layer - 1) * rise * ropeDiameterM;
    }

    /** Turns in a layer on a drum length (m, between the flanges) at a groove pitch (m, at least the rope diameter): floor(L / p). */
    public static int turnsPerLayer(double drumLengthM, double pitchM) {
        Check.positive(drumLengthM, "Drum length"); Check.positive(pitchM, "Pitch");
        double turns = Math.floor(drumLengthM / pitchM);
        if (turns < 1) { throw new IllegalArgumentException("The drum is shorter than one pitch"); }
        if (turns > Integer.MAX_VALUE) { throw new IllegalArgumentException("Too many turns"); }
        return (int) turns;
    }

    /** Rope (m) one layer holds: turns x pi x its pitch diameter. */
    public static double layerLengthM(double drumDiameterM, double ropeDiameterM, int turns, int layer) {
        return Check.atLeastOne(turns, "Turns") * Math.PI * layerDiameterM(drumDiameterM, ropeDiameterM, layer);
    }

    /** Rope (m) a drum holds in its first n layers: the sum of layerLengthM. */
    public static double capacityM(double drumDiameterM, double ropeDiameterM, int turns, int layers) {
        Check.atLeastOne(layers, "Layers");
        double sum = 0;
        for (int i = 1; i <= layers; i++) { sum += layerLengthM(drumDiameterM, ropeDiameterM, turns, i); }
        return sum;
    }

    /**
     * The layer (1 the first) holding the outermost wrap when a length of rope (m, not negative; dead wraps included)
     * is wound on: the first layer whose cumulative capacity reaches it. The line pull and rope speed act at that
     * layer's pitch diameter.
     */
    public static int layerFor(double drumDiameterM, double ropeDiameterM, int turns, double storedM) {
        Check.nonNegative(storedM, "Stored rope");
        double sum = 0;
        for (int i = 1; i <= MAX_LAYERS; i++) {
            sum += layerLengthM(drumDiameterM, ropeDiameterM, turns, i);
            if (storedM <= sum) { return i; }
        }
        throw new IllegalArgumentException("More than " + MAX_LAYERS + " layers of rope");
    }

    /** Line pull (N) at a layer of pitch diameter D_i (m) from a motor torque (N m) through a gear of ratio i (motor turns per drum turn) and efficiency: T i eta / (D_i / 2). */
    public static double linePullN(double motorTorqueNm, double gearRatio, double gearEfficiency, double layerDiameterM) {
        Check.finite(motorTorqueNm, "Motor torque"); Check.positive(gearRatio, "Gear ratio");
        Check.share(gearEfficiency, "Gear efficiency"); Check.positive(layerDiameterM, "Layer diameter");
        return motorTorqueNm * gearRatio * gearEfficiency / (layerDiameterM / 2);
    }

    /** Motor torque (N m) for a line pull (N) at a layer: S (D_i / 2) / (i eta), the inverse of linePullN when hoisting. */
    public static double motorTorqueNm(double linePullN, double gearRatio, double gearEfficiency, double layerDiameterM) {
        Check.finite(linePullN, "Line pull"); Check.positive(gearRatio, "Gear ratio");
        Check.share(gearEfficiency, "Gear efficiency"); Check.positive(layerDiameterM, "Layer diameter");
        return linePullN * (layerDiameterM / 2) / (gearRatio * gearEfficiency);
    }

    /** Rope speed (m/s) at a layer of pitch diameter D_i (m) for a motor speed (rad/s) through a gear of ratio i: (omega / i) (D_i / 2). */
    public static double ropeSpeedMps(double motorRadPerS, double gearRatio, double layerDiameterM) {
        Check.finite(motorRadPerS, "Motor speed"); Check.positive(gearRatio, "Gear ratio"); Check.positive(layerDiameterM, "Layer diameter");
        return motorRadPerS / gearRatio * (layerDiameterM / 2);
    }
}

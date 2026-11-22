package dev.mcmod.aerocore.api.earthmoving;

/**
 * Buckets and scraper bowls: rated capacity, how far a cut fills them, and what loading them costs.
 *
 * <ul>
 * <li>Capacity (SAE J296, ISO 7451 for hydraulic excavator buckets): struck capacity V_s = S W, the side area inside the
 *     strike plane times the mean inner width; heaped capacity V_h = V_s + the heap on the strike plane, a hip roof
 *     with faces at a slope k (rise over run) over the opening b x W (b the shorter side): k b^2 W / 4 - k b^3 / 12;
 *     SAE / ISO heap at 1:1 (b^2 W / 4 - b^3 / 12), the older CECE rating at 1:2 (b^2 W / 8 - b^3 / 24).</li>
 * <li>Fill factor: the share of the heaped capacity a bucket really carries (Caterpillar Performance Handbook,
 *     hydraulic excavators): moist loam or sandy clay 1.00 to 1.10, sand and gravel 0.95 to 1.10, hard tough clay 0.80
 *     to 0.90, well blasted rock 0.60 to 0.75, poorly blasted rock 0.40 to 0.50 (mid values below).</li>
 * <li>Loading: the edge's cut ({@link Digging#edge}) plus lifting the chip into the container, rho g d w h per metre of
 *     travel (the work of raising the cut, d x w of bank material, by h). The friction of the incoming chip against
 *     the load already in a bowl is not modelled (it grows as the bowl fills).</li>
 * <li>Breakout: the tangential force a curling bucket needs at its tip, the cut's resistance plus the load's moment
 *     about the bucket pivot over the tip radius (SAE J1179 rates breakout as that tip force).</li>
 * </ul>
 */
public final class Bucket {
    private Bucket() {}

    /** Heap slope of SAE J296 and ISO 7451: 1:1. */
    public static final double SAE_HEAP_SLOPE = 1.0;

    /** Heap slope of the CECE rating: 1:2. */
    public static final double CECE_HEAP_SLOPE = 0.5;

    /** Fill factor of moist loam or sandy clay (Caterpillar: 1.00 to 1.10). */
    public static final double FILL_LOAM = 1.05;

    /** Fill factor of sand and gravel (Caterpillar: 0.95 to 1.10). */
    public static final double FILL_SAND_GRAVEL = 1.0;

    /** Fill factor of hard, tough clay (Caterpillar: 0.80 to 0.90). */
    public static final double FILL_HARD_CLAY = 0.85;

    /** Fill factor of well blasted rock (Caterpillar: 0.60 to 0.75). */
    public static final double FILL_BLASTED_ROCK = 0.675;

    /** Fill factor of poorly blasted rock (Caterpillar: 0.40 to 0.50). */
    public static final double FILL_POOR_BLASTED_ROCK = 0.45;

    /** Struck capacity (m^3): side area inside the strike plane (m^2) x mean inner width (m). */
    public static double struckM3(double sideAreaM2, double meanWidthM) {
        if (!(sideAreaM2 >= 0) || !(meanWidthM >= 0) || !Double.isFinite(sideAreaM2) || !Double.isFinite(meanWidthM)) {
            throw new IllegalArgumentException("Area and width finite and not negative");
        }
        return sideAreaM2 * meanWidthM;
    }

    /**
     * The heap (m^3) on an opening of length and width (m) with faces at a slope (rise over run, 1 for SAE / ISO): a hip
     * roof, k a^2 c / 4 - k a^3 / 12 with a the shorter and c the longer side.
     */
    public static double heapM3(double openingLengthM, double openingWidthM, double slope) {
        if (!(openingLengthM >= 0) || !(openingWidthM >= 0) || !(slope >= 0) || !Double.isFinite(openingLengthM) || !Double.isFinite(openingWidthM)
                || !Double.isFinite(slope)) {
            throw new IllegalArgumentException("Sizes and slope finite and not negative");
        }
        double a = Math.min(openingLengthM, openingWidthM), c = Math.max(openingLengthM, openingWidthM);
        return slope * a * a * c / 4 - slope * a * a * a / 12;
    }

    /** Heaped capacity (m^3): struck capacity plus the heap of {@link #heapM3}. */
    public static double heapedM3(double struckM3, double openingLengthM, double openingWidthM, double slope) {
        if (!(struckM3 >= 0) || !Double.isFinite(struckM3)) { throw new IllegalArgumentException("Struck capacity finite and not negative"); }
        return struckM3 + heapM3(openingLengthM, openingWidthM, slope);
    }

    /**
     * Length of cut (m) that fills a container of a heaped capacity (m^3) to a fill factor, cutting a depth and width
     * (m) of a material: capacity x fill / (d w swell). Infinite for a cut of no cross-section.
     */
    public static double cutToFillM(Material m, double capacityM3, double fillFactor, double depthM, double widthM) {
        if (!(capacityM3 >= 0) || !(fillFactor >= 0) || !(depthM >= 0) || !(widthM >= 0) || !Double.isFinite(capacityM3) || !Double.isFinite(fillFactor)
                || !Double.isFinite(depthM) || !Double.isFinite(widthM)) {
            throw new IllegalArgumentException("Capacity, fill factor, depth and width finite and not negative");
        }
        double need = capacityM3 * fillFactor, loosePerM = depthM * widthM * m.swell();
        if (need == 0) { return 0; }
        return loosePerM > 0 ? need / loosePerM : Double.POSITIVE_INFINITY;
    }

    /** Force (N) of lifting the cut chip (depth x width, m, of bank material) by a height (m) as the container advances: rho_bank g d w h. */
    public static double liftN(Material m, double depthM, double widthM, double liftM) {
        if (!(depthM >= 0) || !(widthM >= 0) || !(liftM >= 0) || !Double.isFinite(depthM) || !Double.isFinite(widthM) || !Double.isFinite(liftM)) {
            throw new IllegalArgumentException("Depth, width and lift finite and not negative");
        }
        return m.bankKgM3() * Material.G * depthM * widthM * liftM;
    }

    /**
     * Horizontal resistance (N) of a scraper bowl loading: its cutting edge (rake rad, depth and width m, speed m/s)
     * by {@link Digging#edge} plus lifting the chip by liftM (m) into the bowl, {@link #liftN}.
     */
    public static double scraperLoadingN(Material m, double rakeRad, double depthM, double widthM, double speedMps, double liftM) {
        return Digging.edge(m, rakeRad, depthM, widthM, speedMps, 0).horizontalN() + liftN(m, depthM, widthM, liftM);
    }

    /**
     * Tip force (N) a curling bucket needs: the cut's resistance along the tip's path (N) plus a load (kg) at an arm
     * about the pivot (m, horizontal distance of its centre of mass) over the tip radius (m).
     */
    public static double breakoutN(double cutN, double loadKg, double loadArmM, double tipRadiusM) {
        if (!(cutN >= 0) || !(loadKg >= 0) || !(tipRadiusM > 0) || !Double.isFinite(cutN) || !Double.isFinite(loadKg) || !Double.isFinite(loadArmM)
                || !Double.isFinite(tipRadiusM)) {
            throw new IllegalArgumentException("Force and load finite and not negative, tip radius positive");
        }
        return cutN + loadKg * Material.G * loadArmM / tipRadiusM;
    }
}

package dev.mcmod.aerocore.api.earthmoving;

/**
 * Dozer and grader blades: the loose soil prism carried ahead of the blade, what pushing it costs, and what spills.
 *
 * <ul>
 * <li>Prism: loose soil heaped against a blade of width W and height H stands at its angle of repose, a triangular
 *     prism of volume 0.5 W H^2 / tan(repose). SAE J1265 (capacity rating of dozer blades) rates a straight blade at
 *     0.8 W H^2, which is this prism at a repose of 32 deg (tan 32 deg = 0.625); Caterpillar's handbook quotes blade
 *     capacities by that rule (a D6-class straight blade 3.4 m x 1.3 m: 4.6 m^3 loose).</li>
 * <li>Pushing: the prism slides over the ground on loose soil, resistance its weight times tan(phi) of the loose
 *     material (soil on soil). The cut under the blade is a separate {@link Digging#edge} with the prism as surcharge.</li>
 * <li>Spill: loose material above the blade's capacity runs off over the top and the ends.</li>
 * </ul>
 */
public final class Blade {
    private Blade() {}

    /** SAE J1265's capacity factor of a straight blade: capacity 0.8 W H^2 (m^3). */
    public static final double SAE_J1265_FACTOR = 0.8;

    /** Loose prism (m^3) a blade of width and height (m) holds at an angle of repose (rad, 0..90 deg exclusive): 0.5 W H^2 / tan(repose). */
    public static double prismM3(double widthM, double heightM, double reposeRad) {
        if (!(widthM >= 0) || !(heightM >= 0) || !(reposeRad > 0 && reposeRad < Math.PI / 2) || !Double.isFinite(widthM) || !Double.isFinite(heightM)) {
            throw new IllegalArgumentException("Sizes finite and not negative, repose within (0, 90) deg");
        }
        return 0.5 * widthM * heightM * heightM / Math.tan(reposeRad);
    }

    /** SAE J1265 capacity (m^3) of a straight blade of width and height (m): 0.8 W H^2. */
    public static double saeJ1265M3(double widthM, double heightM) {
        if (!(widthM >= 0) || !(heightM >= 0) || !Double.isFinite(widthM) || !Double.isFinite(heightM)) { throw new IllegalArgumentException("Sizes finite and not negative"); }
        return SAE_J1265_FACTOR * widthM * heightM * heightM;
    }

    /** Force (N) to slide a loose prism (m^3) of a material over the ground: loose mass x g x tan(phi) of the dumped material (its angle of repose). */
    public static double pushN(Material m, double prismLooseM3) {
        if (!(prismLooseM3 >= 0) || !Double.isFinite(prismLooseM3)) { throw new IllegalArgumentException("Volume finite and not negative"); }
        return prismLooseM3 * m.looseKgM3() * Material.G * Math.tan(m.reposeRad());
    }

    /** Loose volume (m^3) that spills from a blade holding a loose volume when its capacity (m^3) is less: max(0, held - capacity). */
    public static double spilledM3(double heldLooseM3, double capacityM3) {
        if (!(heldLooseM3 >= 0) || !(capacityM3 >= 0) || !Double.isFinite(heldLooseM3) || !Double.isFinite(capacityM3)) {
            throw new IllegalArgumentException("Volumes finite and not negative");
        }
        return Math.max(0, heldLooseM3 - capacityM3);
    }

    /**
     * Mean surcharge (Pa) a prism of a height (m) at the blade puts on the ground it covers: half its height of loose
     * material (a triangular prism's mean), for {@link Digging#edge}.
     */
    public static double surchargePa(Material m, double prismHeightM) {
        if (!(prismHeightM >= 0) || !Double.isFinite(prismHeightM)) { throw new IllegalArgumentException("Height finite and not negative"); }
        return 0.5 * prismHeightM * m.looseKgM3() * Material.G;
    }

    /** Height (m) at the blade of a loose prism (m^3) on a blade of a width (m), at the material's angle of repose: sqrt(2 V tan(repose) / W). */
    public static double prismHeightM(Material m, double prismLooseM3, double widthM) {
        if (!(prismLooseM3 >= 0) || !(widthM > 0) || !Double.isFinite(prismLooseM3) || !Double.isFinite(widthM)) {
            throw new IllegalArgumentException("Volume finite and not negative, width positive");
        }
        return Math.sqrt(2 * prismLooseM3 * Math.tan(m.reposeRad()) / widthM);
    }
}

package dev.mcmod.aerocore.api.lifting;

import dev.mcmod.aerocore.api.Atmosphere;
import dev.mcmod.aerocore.api.frame.FrameMaterial;
import dev.mcmod.aerocore.api.frame.Section;

import java.util.Objects;

/**
 * Rated loads of a tower crane with a trolley on a horizontal jib: the hook load is capped by the hoist near the mast
 * (the maximum load) and by the load moment farther out, load x radius &lt;= the maximum load moment, out to the jib's
 * end. Formulas only; the maximum load and moment (set by whichever of the jib, tower, slewing ring and foundation is
 * weakest) are the caller's, and momentCapacityNm gives one member's elastic capacity.
 *
 * <p>A simplification: real charts fall a little faster than 1 / r toward the jib's end (the jib's own bending and the
 * trolley and hook block counted in the load), and the counter-jib's ballast, which balances part of the load moment,
 * is the caller's.
 */
public final class TowerCrane {
    private TowerCrane() {}

    private static final double G = Atmosphere.GRAVITY_MPS2;

    /**
     * Rated load (kg) at a trolley radius (m): min(maximum load, maximum load moment / (g r)) between the least radius
     * the trolley reaches and the jib's length (m), 0 outside. The maximum load moment is in N m.
     */
    public static double ratedKg(double radiusM, double minRadiusM, double jibLengthM, double maxLoadKg, double maxLoadMomentNm) {
        Check.finite(radiusM, "Radius"); Check.nonNegative(minRadiusM, "Least radius"); Check.positive(jibLengthM, "Jib length");
        Check.positive(maxLoadKg, "Maximum load"); Check.positive(maxLoadMomentNm, "Maximum load moment");
        if (!(minRadiusM < jibLengthM)) { throw new IllegalArgumentException("The least radius must be inside the jib"); }
        if (radiusM < minRadiusM || radiusM > jibLengthM) { return 0; }
        if (radiusM == 0) { return maxLoadKg; }
        return Math.min(maxLoadKg, maxLoadMomentNm / (G * radiusM));
    }

    /** The radius (m) out to which the maximum load may be lifted: maximum load moment / (g x maximum load). */
    public static double maxLoadRadiusM(double maxLoadKg, double maxLoadMomentNm) {
        return Check.positive(maxLoadMomentNm, "Maximum load moment") / (G * Check.positive(maxLoadKg, "Maximum load"));
    }

    /** A chart of ratedKg at points (at least 2) evenly spaced from the least radius to the jib's end (m). */
    public static LoadChart.Table chart(double minRadiusM, double jibLengthM, double maxLoadKg, double maxLoadMomentNm, int points) {
        if (points < 2) { throw new IllegalArgumentException("A chart has at least two points"); }
        double[] radii = new double[points], rated = new double[points];
        for (int i = 0; i < points; i++) {
            radii[i] = i == points - 1 ? jibLengthM : minRadiusM + (jibLengthM - minRadiusM) * i / (points - 1);
            rated[i] = ratedKg(radii[i], minRadiusM, jibLengthM, maxLoadKg, maxLoadMomentNm);
        }
        return new LoadChart.Table(radii, rated);
    }

    /**
     * Elastic moment capacity (N m) of a member's section about its z axis (bending in its up plane): yield / safety x
     * I_z / y_max (safety at least 1; LoadChart.STRUCTURE_SAFETY). For a jib or tower taken as one beam; the lattice
     * members' own buckling is the caller's.
     */
    public static double momentCapacityNm(Section section, FrameMaterial material, double safety) {
        Objects.requireNonNull(section, "section"); Objects.requireNonNull(material, "material");
        if (!(safety >= 1) || !Double.isFinite(safety)) { throw new IllegalArgumentException("Safety factor at least 1"); }
        return material.yieldPa() / safety * section.izM4() / section.outerYM();
    }
}

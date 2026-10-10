package dev.mcmod.aerocore.api;

import java.util.ArrayList;
import java.util.List;

/**
 * Hard-chine planing hull: V-bottom sections of constant deadrise beta from the keel to the chines, vertical sides up
 * to the deck. Model-local axes as SlenderHull (x forward, y up, z starboard); the transom is at x = 0 and the
 * keel of the prismatic part at y = 0. The bow follows Fridsma's (1969) series models: aft of one beam from the stem
 * the hull is prismatic; over the forward beam the chine half-beam follows the elliptical planform
 * (x'/B)^2 + (b/b_c)^2 = 1 and the keel rises on the ellipse (x'/B)^2 + ((D - y)/D)^2 = 1 to the deck at the stem
 * (x' measured forward from the start of the bow, B the chine beam, D the depth).
 */
public final class PlaningHull {
    private final SlenderHull hull;
    private final double length, chineBeam, deadrise, depth;
    private final double[] keelY, chineHalfBeam;

    private PlaningHull(SlenderHull hull, double length, double chineBeam, double deadrise, double depth, double[] keelY, double[] chineHalfBeam) {
        this.hull = hull; this.length = length; this.chineBeam = chineBeam; this.deadrise = deadrise; this.depth = depth;
        this.keelY = keelY; this.chineHalfBeam = chineHalfBeam;
    }

    /**
     * Fridsma-type hull of overall length L, chine beam B, deadrise (rad) and depth (keel to deck), with the given
     * number of stations (denser over the bow and near the transom).
     */
    public static PlaningHull fridsma(double length, double chineBeam, double deadrise, double depth, int stations) {
        Checks.positive(length, "Length"); Checks.positive(chineBeam, "Beam"); Checks.positive(depth, "Depth");
        if (!(deadrise > 0 && deadrise < Math.toRadians(45))) { throw new IllegalArgumentException("Deadrise must be in (0, 45) deg"); }
        if (!(length > 1.5 * chineBeam)) { throw new IllegalArgumentException("Length must exceed 1.5 beams"); }
        if (stations < 8) { throw new IllegalArgumentException("Use at least 8 stations"); }
        double bowStart = length - chineBeam, half = chineBeam / 2;
        List<HullSection> sections = new ArrayList<>();
        double[] keel = new double[stations], chine = new double[stations];
        for (int i = 0; i < stations; i++) {
            // Cosine spacing in s gives fine strips at both ends; the last station sits just short of the stem.
            double s = 0.5 * (1 - Math.cos(Math.PI * i / (stations - 1))), x = s * length * 0.995;
            double b = half, k = 0;
            if (x > bowStart) {
                double e = (x - bowStart) / chineBeam;
                b = half * Math.sqrt(Math.max(0, 1 - e * e));
                k = depth * (1 - Math.sqrt(Math.max(0, 1 - e * e)));
            }
            b = Math.max(b, 0.01 * half);
            k = Math.min(k, depth - 0.02 * depth);
            double chineY = Math.min(k + b * Math.tan(deadrise), depth - 0.01 * depth);
            keel[i] = k; chine[i] = b;
            List<double[]> v = new ArrayList<>();
            v.add(new double[]{depth, b});
            if (chineY < depth) { v.add(new double[]{chineY, b}); }
            v.add(new double[]{k, 0});
            if (chineY < depth) { v.add(new double[]{chineY, -b}); }
            v.add(new double[]{depth, -b});
            sections.add(new HullSection(x, v));
        }
        return new PlaningHull(new SlenderHull(sections), length, chineBeam, deadrise, depth, keel, chine);
    }

    public SlenderHull hull() { return hull; }
    public double length() { return length; }
    public double chineBeam() { return chineBeam; }
    public double deadrise() { return deadrise; }
    public double depth() { return depth; }
    /** Keel height of station i above the prismatic keel line, m. */
    public double keelY(int i) { return keelY[i]; }
    /** Chine half-beam of station i, m. */
    public double chineHalfBeam(int i) { return chineHalfBeam[i]; }
    /** Strip boundaries: boundary(0) is the transom, boundary(n) the stem. */
    public double boundary(int i) {
        int n = hull.stations();
        if (i <= 0) { return 0; }
        if (i >= n) { return length; }
        return 0.5 * (hull.sections().get(i - 1).xM() + hull.sections().get(i).xM());
    }
}

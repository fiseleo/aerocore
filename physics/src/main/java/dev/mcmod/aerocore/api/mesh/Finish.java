package dev.mcmod.aerocore.api.mesh;

/**
 * A surface finish (0.29): the key a client draws a quad with (its texture, gloss, metalness) and the length in metres
 * its texture repeats over, which the mesh builders use for texture coordinates (circumferences of turned and drawn
 * surfaces are rounded to whole repeats, so a seam does not show). What a finish looks like is the drawing mod's.
 */
public record Finish(String id, double tileM) {
    public Finish {
        if (id == null || id.isBlank()) { throw new IllegalArgumentException("A finish needs an id"); }
        if (!(tileM > 0) || !Double.isFinite(tileM)) { throw new IllegalArgumentException("A finish's tile must be a positive length"); }
    }

    /** The tile used where a quad has no finish. */
    public static final double DEFAULT_TILE_M = 0.5;

    /** A circumference rounded to whole repeats of a tile (at least one). */
    public static double snapped(double circumferenceM, double tileM) {
        return tileM * Math.max(1, Math.round(circumferenceM / tileM));
    }
}

package dev.mcmod.aerocore.api.damage;

/**
 * 0.42: how fast metal wastes where it stands, as a uniform loss of thickness. Typical rates only (orders of magnitude
 * for an unprotected surface); how thick a part's wall is, how well it is painted and how fast a game's year passes are
 * the caller's.
 *
 * <p>Sources: carbon steel in air from the first-year rates of ISO 9223:2012 (corrosivity C1 up to 1.3 um/year, C2 to
 * 25, C3 to 50, C4 to 80, C5 to 200); steel immersed in sea water 0.1 to 0.15 mm/year and two to three times that in
 * the splash zone, in fresh water about half the sea's (Uhlig's Corrosion Handbook, 3rd ed., the chapters on carbon
 * steel in waters). The other metals are typical figures of the same handbooks (ASM Handbook 13B), given as the uniform
 * loss that does the same harm: aluminium, stainless steel and titanium keep an oxide film and fail by pitting where
 * they fail at all; magnesium goes quickly in salt water.
 */
public final class Corrosion {
    private Corrosion() {}

    /** Where the metal stands. */
    public enum Exposure {
        /** Indoors, dry (ISO 9223 C1). */
        DRY,
        /** Outdoors under cover or in dry weather, inland (C2). */
        AIR,
        /** Rained on (C3 to C4). */
        RAIN,
        /** Sea air: salt spray without being wetted by the sea (C5). */
        SEA_AIR,
        /** Under fresh water. */
        FRESH_WATER,
        /** Under sea water. */
        SEA_WATER,
        /** Wetted by the sea and drying again: the splash zone. */
        SEA_SPLASH
    }

    /** What the metal is. */
    public enum Metal {
        CARBON_STEEL(0.0010, 0.013, 0.050, 0.14, 0.05, 0.12, 0.30),
        /** Weathering and low-alloy steels: a third of carbon steel's in air once the patina forms, the same under water. */
        LOW_ALLOY_STEEL(0.0005, 0.005, 0.017, 0.07, 0.05, 0.12, 0.30),
        STAINLESS_STEEL(0, 0.00003, 0.0001, 0.001, 0.0002, 0.002, 0.004),
        ALUMINIUM(0, 0.0002, 0.0005, 0.002, 0.002, 0.010, 0.020),
        MAGNESIUM(0.0005, 0.010, 0.030, 0.20, 0.10, 1.5, 2.0),
        TITANIUM(0, 0, 0, 0, 0, 0.00001, 0.00001),
        COPPER_ALLOY(0, 0.001, 0.002, 0.004, 0.010, 0.030, 0.050),
        /** Nothing that rusts: composites, plastics, rubber, glass. */
        NONE(0, 0, 0, 0, 0, 0, 0);

        private final double[] mmPerYear;
        Metal(double... mmPerYear) { this.mmPerYear = mmPerYear; }
    }

    /** Seconds of a year (365.25 days). */
    public static final double YEAR_S = 365.25 * 86400;

    /** The loss of thickness of a bare surface, mm per year. */
    public static double rateMmPerYear(Metal metal, Exposure exposure) { return metal.mmPerYear[exposure.ordinal()]; }

    /**
     * The rate under a coating: a sound paint system lets through the share holidays of the bare rate (pinholes and
     * edges; a few per cent for a good system), a coating that is gone nothing less than all of it. intact is 0 (bare)
     * to 1 (sound).
     */
    public static double coated(double bareMmPerYear, double intact, double holidays) {
        if (!(bareMmPerYear >= 0) || !(holidays >= 0 && holidays <= 1)) { throw new IllegalArgumentException("Rate not negative, holidays within 0..1"); }
        double sound = Math.clamp(intact, 0.0, 1.0);
        return bareMmPerYear * (1 - sound * (1 - holidays));
    }

    /**
     * The share of a wall used up in a time (s) at a rate (mm/year): the loss over the share of the thickness (mm) the
     * part can lose before it is no longer fit (allowable, 0..1: about 0.2 to 0.3 for thin stressed skin, more for
     * heavy castings).
     */
    public static double used(double mmPerYear, double seconds, double thicknessMm, double allowable) {
        if (!(mmPerYear >= 0) || !(seconds >= 0) || !(thicknessMm > 0) || !(allowable > 0 && allowable <= 1)) {
            throw new IllegalArgumentException("Rate and time not negative, thickness positive, allowable within (0, 1]");
        }
        return mmPerYear * seconds / YEAR_S / (allowable * thicknessMm);
    }
}

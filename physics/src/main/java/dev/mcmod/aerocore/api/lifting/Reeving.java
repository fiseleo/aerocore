package dev.mcmod.aerocore.api.lifting;

/**
 * A rope tackle: n parts of line between the boom head (or trolley, or crab) and the hook block. Formulas only; how
 * many parts a crane reeves is the caller's.
 *
 * <p>Each sheave the rope runs over while hoisting costs a share 1 - eta of the tension (bearing friction and the
 * rope's bending stiffness). With the fast line (to the drum) carrying S, the n parts carry S, S eta, ..., S eta^(n-1),
 * so the hook load is W = S (1 - eta^n) / (1 - eta) = n S eta_F with the tackle efficiency
 * eta_F = (1 - eta^n) / (n (1 - eta)) (DIN 15020-1; FEM 1.001 booklet 4). Fixed guide sheaves between the drum and
 * the tackle cost eta each on top. Checked against a maker's figures: a two-axle 40 t all-terrain crane with 34 kN
 * line pull rates its hook blocks of 3, 7 and 11 parts at 10.1, 22.8 and 34.9 t (Liebherr LTM 1040-2.1 data sheet);
 * eta_F with eta = 0.98 gives 10.2, 22.9 and 34.5 t.
 */
public final class Reeving {
    private Reeving() {}

    /** Efficiency of a sheave on rolling bearings (DIN 15020-1, as commonly quoted; the maker's hook blocks in the class comment agree). */
    public static final double SHEAVE_ROLLING = 0.98;
    /** Efficiency of a sheave on plain bearings (DIN 15020-1, as commonly quoted). */
    public static final double SHEAVE_PLAIN = 0.96;
    /**
     * Full wraps that must stay on a drum with the hook at its lowest (the "dead wraps" that take the load off the rope
     * anchor): two (OSHA 29 CFR 1910.180(h)(3)(x), from ANSI B30.5).
     */
    public static final int DEAD_WRAPS = 2;

    /** Tackle efficiency eta_F = (1 - eta^n) / (n (1 - eta)) for n parts over sheaves of efficiency eta (1 at eta = 1). */
    public static double efficiency(int parts, double sheaveEfficiency) {
        Check.atLeastOne(parts, "Parts of line");
        Check.share(sheaveEfficiency, "Sheave efficiency");
        if (sheaveEfficiency == 1) { return 1; }
        return (1 - Math.pow(sheaveEfficiency, parts)) / (parts * (1 - sheaveEfficiency));
    }

    /**
     * Hook load (N, block included) a line pull at the drum (N) lifts: n S eta_F eta^k, k the fixed guide sheaves
     * between the drum and the tackle.
     */
    public static double hookLoadN(int parts, double linePullN, double sheaveEfficiency, int guideSheaves) {
        Check.nonNegative(linePullN, "Line pull");
        if (guideSheaves < 0) { throw new IllegalArgumentException("Guide sheaves must not be negative"); }
        return parts * linePullN * efficiency(parts, sheaveEfficiency) * Math.pow(sheaveEfficiency, guideSheaves);
    }

    /** Line pull at the drum (N) to hoist a hook load (N, block included): W / (n eta_F eta^k). */
    public static double linePullN(int parts, double hookLoadN, double sheaveEfficiency, int guideSheaves) {
        Check.nonNegative(hookLoadN, "Hook load");
        if (guideSheaves < 0) { throw new IllegalArgumentException("Guide sheaves must not be negative"); }
        return hookLoadN / (parts * efficiency(parts, sheaveEfficiency) * Math.pow(sheaveEfficiency, guideSheaves));
    }

    /** Hook speed (m/s) for a rope speed at the drum (m/s): v / n. */
    public static double hookSpeedMps(double ropeSpeedMps, int parts) {
        Check.finite(ropeSpeedMps, "Rope speed");
        return ropeSpeedMps / Check.atLeastOne(parts, "Parts of line");
    }

    /** Rope (m) the drum takes in to lift the hook by a height (m): n h. */
    public static double storedRopeM(double liftM, int parts) {
        Check.nonNegative(liftM, "Lift");
        return Check.atLeastOne(parts, "Parts of line") * liftM;
    }

    /**
     * Rope length (m) to reeve: n parts of a fall (m, head to block at the lowest hook position), the lead from the
     * drum to the tackle (m) and DEAD_WRAPS turns on a drum of pitch diameter D (m): n fall + lead + 2 pi D.
     */
    public static double reevedLengthM(int parts, double fallM, double leadM, double drumDiameterM) {
        Check.nonNegative(fallM, "Fall"); Check.nonNegative(leadM, "Lead"); Check.positive(drumDiameterM, "Drum diameter");
        return Check.atLeastOne(parts, "Parts of line") * fallM + leadM + DEAD_WRAPS * Math.PI * drumDiameterM;
    }
}

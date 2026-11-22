package dev.mcmod.aerocore.api.lifting;

import java.util.Objects;

/**
 * Steel wire rope for hoisting: strength, mass and stiffness of a rope, the coefficient of utilisation and the drum and
 * sheave diameters a mechanism group asks for, and the bending life over sheaves. Formulas and sourced typical values
 * only; which rope a crane reeves and how it is classified are the caller's.
 *
 * <p>SI throughout (m, N, Pa, kg). The rope grade is the nominal tensile strength R_r of the wires (EN 12385-4,
 * ISO 2408: 1770, 1960 or 2160 N/mm^2), the same number ISO 4308-1 calls R_o.
 *
 * <p>Sources: ISO 4308-1:2003 (Cranes and lifting appliances, selection of wire ropes, Tables 1 to 4); EN 12385-4
 * (stranded ropes for general lifting, minimum breaking force F_min = K d^2 R_r / 1000 kN with d in mm and R_r in
 * N/mm^2); maker's data to EN 12385-4 for the two constructions below; Feyrer, Wire Ropes, 2nd ed. (Springer 2015)
 * and his 2011 correction sheet (UPDATE-Rope.pdf, IFT Universitaet Stuttgart) for the bending life.
 */
public final class WireRope {
    private WireRope() {}

    /** Rope grade 1770 N/mm^2 (Pa). */
    public static final double GRADE_1770_PA = 1770e6;
    /** Rope grade 1960 N/mm^2 (Pa), the usual grade of crane hoist ropes. */
    public static final double GRADE_1960_PA = 1960e6;
    /** Rope grade 2160 N/mm^2 (Pa). */
    public static final double GRADE_2160_PA = 2160e6;

    /**
     * Rope modulus over the metallic area of a six-strand rope with a steel core at working loads, about 100 kN/mm^2
     * (Pa). A rope has no single Young's modulus: its apparent modulus, measured between two loads, rises with load
     * and with the share of steel in the section (Feyrer, Wire Ropes, on the rope modulus). An order of magnitude for
     * stretch and EA / L, not a catalogue value; the caller's measured value is better.
     */
    public static final double MODULUS_PA = 100e9;

    /** Smallest rope diameter (m) Feyrer's bending-life regression is given for: 6 mm. */
    public static final double FEYRER_MIN_DIAMETER_M = 0.006;

    /**
     * Constants of Feyrer's bending-life regression (Wire Ropes, 2nd ed., eq. 3.55 and Table 3.14 as corrected in
     * 2011): lg N = a0 + (a1 + a3 lg(D/d)) (lg(S/d^2) - 0.4 lg(R0/1770)) + a2 lg(D/d) + lg f_d + lg f_l.
     */
    public record Feyrer(double a0, double a1, double a2, double a3) {
        public Feyrer {
            Check.finite(a0, "a0"); Check.finite(a1, "a1"); Check.finite(a2, "a2"); Check.finite(a3, "a3");
        }
    }

    /**
     * Rope constructions with their minimum breaking force factor K (EN 12385-4), fill factor f (metallic area over
     * the rope circle, pi d^2 / 4), mass factor (kg/m per m^2 of d^2: mass per metre = factor x d^2), ISO 4308-1
     * rope type factor t and Feyrer's constants for the mean number of bending cycles to discard.
     */
    public enum Construction {
        /**
         * Six strands of 36 wires (6x36 Warrington-Seale class), independent wire rope core, regular lay, not
         * compacted. K = 0.356 (EN 12385-4; ISO 4308-1 Table 1 uses the same K' for 6x36WS-IWRC); fill factor 0.586
         * and 1.64 kg/m at 20 mm (Pfeifer data sheet 6x36WS-WC to EN 12385-4, which lists 252 / 279 / 308 kN at 20 mm
         * in grades 1770 / 1960 / 2160); t = 1 (6 to 10 outer strands). Bending life: Feyrer's 8x36 Warrington-Seale
         * IWRC regular-lay row (the nearest class: his table has no six-strand 36-wire row).
         */
        SIX_STRAND_IWRC(0.356, 0.586, 4100, 1.0, new Feyrer(0.584, 0.377, 6.232, -1.750)),
        /**
         * Rotation-resistant 35(W)x7 with a steel centre, not compacted. K = 0.360 and 1.82 kg/m at 20 mm (Katradis
         * 35(W)x7 data to EN 12385: 282 kN at 20 mm and 70.6 kN at 10 mm in grade 1960); fill factor 0.61 (Mennens 35x7
         * data: 38.9 mm^2 of steel at 9 mm); t = 1 (rotation-resistant, 10 and more outer strands). Bending life:
         * Feyrer's 34x7 spiral round strand row.
         */
        ROTATION_RESISTANT_35X7(0.360, 0.61, 4550, 1.0, new Feyrer(-1.432, 1.619, 7.559, -2.622));

        private final double breakingFactor, fillFactor, massFactorKgM3, ropeTypeFactor;
        private final Feyrer discard;

        Construction(double breakingFactor, double fillFactor, double massFactorKgM3, double ropeTypeFactor, Feyrer discard) {
            this.breakingFactor = breakingFactor;
            this.fillFactor = fillFactor;
            this.massFactorKgM3 = massFactorKgM3;
            this.ropeTypeFactor = ropeTypeFactor;
            this.discard = discard;
        }

        /** Minimum breaking force factor K (EN 12385-4). */
        public double breakingFactor() { return breakingFactor; }
        /** Fill factor f: metallic area over pi d^2 / 4. */
        public double fillFactor() { return fillFactor; }
        /** Mass per metre over d^2 (kg/m^3). */
        public double massFactorKgM3() { return massFactorKgM3; }
        /** ISO 4308-1 Table 3 rope type factor t, multiplying the drum and sheave ratios. */
        public double ropeTypeFactor() { return ropeTypeFactor; }
        /** Feyrer's constants for the mean number of bending cycles to discard, regular lay. */
        public Feyrer discard() { return discard; }
    }

    /**
     * ISO 4308-1:2003 mechanism groups (classification by ISO 4301-1): the minimum practical coefficient of utilisation
     * Z_p of running ropes (Table 1) and of stationary ropes such as pendants (Table 4), and the drum and sheave
     * selection factors h1 and h2 (pitch diameter over rope diameter, Table 2).
     */
    public enum Group {
        M1(3.15, 2.5, 11.2, 12.5), M2(3.35, 2.5, 12.5, 14.0), M3(3.55, 3.0, 14.0, 16.0), M4(4.0, 3.5, 16.0, 18.0),
        M5(4.5, 4.0, 18.0, 20.0), M6(5.6, 4.5, 20.0, 22.4), M7(7.1, 5.0, 22.4, 25.0), M8(9.0, 5.0, 25.0, 28.0);

        private final double zp, stationaryZp, drumRatio, sheaveRatio;

        Group(double zp, double stationaryZp, double drumRatio, double sheaveRatio) {
            this.zp = zp;
            this.stationaryZp = stationaryZp;
            this.drumRatio = drumRatio;
            this.sheaveRatio = sheaveRatio;
        }

        /** Z_p of a running rope (ISO 4308-1 Table 1): minimum breaking force over the greatest rope tension. */
        public double zp() { return zp; }
        /** Z_p of a stationary rope (ISO 4308-1 Table 4). */
        public double stationaryZp() { return stationaryZp; }
        /** h1: least drum pitch diameter over rope diameter (ISO 4308-1 Table 2). */
        public double drumRatio() { return drumRatio; }
        /** h2: least sheave pitch diameter over rope diameter (ISO 4308-1 Table 2). */
        public double sheaveRatio() { return sheaveRatio; }
    }

    /**
     * Minimum breaking force (N) of a rope of diameter d (m) in grade R_r (Pa): F_min = K d^2 R_r (EN 12385-4 Annex A,
     * which writes it K d^2 R_r / 1000 in kN with d in mm and R_r in N/mm^2; in SI the factor 1000 cancels).
     */
    public static double minBreakingForceN(double breakingFactor, double diameterM, double gradePa) {
        Check.positive(breakingFactor, "Breaking force factor"); Check.positive(diameterM, "Rope diameter"); Check.positive(gradePa, "Rope grade");
        return breakingFactor * diameterM * diameterM * gradePa;
    }

    /** Minimum breaking force (N) of a construction (its K) at diameter d (m) and grade R_r (Pa). */
    public static double minBreakingForceN(Construction construction, double diameterM, double gradePa) {
        return minBreakingForceN(Objects.requireNonNull(construction, "construction").breakingFactor, diameterM, gradePa);
    }

    /** Metallic cross-section (m^2): A_m = f pi d^2 / 4, f the construction's fill factor. */
    public static double metalAreaM2(Construction construction, double diameterM) {
        Check.positive(diameterM, "Rope diameter");
        return Objects.requireNonNull(construction, "construction").fillFactor * Math.PI * diameterM * diameterM / 4;
    }

    /** Mass per metre (kg/m): the construction's mass factor x d^2 (from the makers' tables named on each construction). */
    public static double kgPerM(Construction construction, double diameterM) {
        Check.positive(diameterM, "Rope diameter");
        return Objects.requireNonNull(construction, "construction").massFactorKgM3 * diameterM * diameterM;
    }

    /**
     * Spinning loss factor k = K / (f pi / 4): the minimum breaking force over the summed strength of the wires, A_m R_r
     * (the wires are laid helically and pressed on each other; Feyrer, ch. 1). About 0.77 for the six-strand rope.
     */
    public static double spinningLossFactor(Construction construction) {
        Objects.requireNonNull(construction, "construction");
        return construction.breakingFactor / (construction.fillFactor * Math.PI / 4);
    }

    /** Axial stiffness (N/m) of a free rope length L (m): E A_m / L, E the rope modulus over the metallic area (Pa; MODULUS_PA). */
    public static double stiffnessNPerM(Construction construction, double diameterM, double modulusPa, double lengthM) {
        Check.positive(modulusPa, "Rope modulus"); Check.positive(lengthM, "Rope length");
        return modulusPa * metalAreaM2(construction, diameterM) / lengthM;
    }

    /** Elastic stretch (m) of a free length L (m) under a tension S (N, not negative): S L / (E A_m). */
    public static double stretchM(Construction construction, double diameterM, double modulusPa, double lengthM, double tensionN) {
        Check.nonNegative(tensionN, "Tension");
        return tensionN / stiffnessNPerM(construction, diameterM, modulusPa, lengthM);
    }

    /** Working load limit (N): the greatest rope tension a minimum breaking force (N) allows at a coefficient of utilisation Z_p (at least 1), F_min / Z_p (ISO 4308-1 eq. 3). */
    public static double workingLoadLimitN(double minBreakingForceN, double zp) {
        Check.positive(minBreakingForceN, "Minimum breaking force");
        if (!(zp >= 1) || !Double.isFinite(zp)) { throw new IllegalArgumentException("The coefficient of utilisation must be at least 1"); }
        return minBreakingForceN / zp;
    }

    /** Working load limit (N) of a running rope of a construction, diameter (m) and grade (Pa) in a mechanism group (its Z_p). */
    public static double workingLoadLimitN(Construction construction, double diameterM, double gradePa, Group group) {
        return workingLoadLimitN(minBreakingForceN(construction, diameterM, gradePa), Objects.requireNonNull(group, "group").zp);
    }

    /**
     * Least rope diameter (m) for a greatest rope tension S (N) at a coefficient of utilisation Z_p: d = C sqrt(S) with
     * C = sqrt(Z_p / (K R_r)) (ISO 4308-1 eqs. 1 and 2; in mm with R in N/mm^2 the same C reads 0.085 for M5 and
     * 6x36WS-IWRC 1770). S includes the hook block, the reeving efficiency and the rope's own weight where it counts;
     * ISO 4308-1 lets the nominal diameter be up to 1.25 times this.
     */
    public static double requiredDiameterM(Construction construction, double gradePa, double ropeTensionN, double zp) {
        Objects.requireNonNull(construction, "construction");
        Check.positive(gradePa, "Rope grade"); Check.positive(ropeTensionN, "Rope tension");
        if (!(zp >= 1) || !Double.isFinite(zp)) { throw new IllegalArgumentException("The coefficient of utilisation must be at least 1"); }
        return Math.sqrt(zp * ropeTensionN / (construction.breakingFactor * gradePa));
    }

    /** Least diameter (m) of a running rope in a mechanism group (its Z_p) for a greatest rope tension (N). */
    public static double requiredDiameterM(Construction construction, double gradePa, double ropeTensionN, Group group) {
        return requiredDiameterM(construction, gradePa, ropeTensionN, Objects.requireNonNull(group, "group").zp);
    }

    /** Least drum pitch diameter (m) for a rope diameter (m) in a mechanism group: h1 t d (ISO 4308-1 eq. 4). */
    public static double minDrumDiameterM(Group group, Construction construction, double ropeDiameterM) {
        Check.positive(ropeDiameterM, "Rope diameter");
        return Objects.requireNonNull(group, "group").drumRatio * Objects.requireNonNull(construction, "construction").ropeTypeFactor * ropeDiameterM;
    }

    /** Least sheave pitch diameter (m) for a rope diameter (m) in a mechanism group: h2 t d (ISO 4308-1 eq. 5). */
    public static double minSheaveDiameterM(Group group, Construction construction, double ropeDiameterM) {
        Check.positive(ropeDiameterM, "Rope diameter");
        return Objects.requireNonNull(group, "group").sheaveRatio * Objects.requireNonNull(construction, "construction").ropeTypeFactor * ropeDiameterM;
    }

    /** Mean number of bending cycles to discard (Feyrer) of a construction; see the other overload. */
    public static double bendsToDiscard(Construction construction, double ropeDiameterM, double sheaveDiameterM, double tensionN,
                                        double gradePa, double bendingLengthM) {
        return bendsToDiscard(Objects.requireNonNull(construction, "construction").discard, ropeDiameterM, sheaveDiameterM, tensionN, gradePa, bendingLengthM);
    }

    /**
     * Number of bending cycles over a sheave (one cycle: straight, bent, straight again) at which a rope reaches its
     * discard state, by Feyrer's regression of single-bending tests (Wire Ropes, 2nd ed., eq. 3.55 and Table 3.14 as
     * corrected in 2011):
     * lg N = a0 + (a1 + a3 lg(D/d)) (lg(S/d^2) - 0.4 lg(R0/1770)) + a2 lg(D/d) + lg f_d + lg f_l, with d and D in mm,
     * S in N, R0 in N/mm^2, the diameter factor f_d = 0.52 / (-0.48 + (d/16)^0.3) and the bending length factor
     * f_l = 1.54 / (2.54 - ((l/d - 2.5) / 5.5)^-0.14).
     *
     * <p>D is the sheave's pitch diameter (groove bottom plus d), l the length of rope that is bent in each cycle
     * (m, at least 10 d). Given for d of 6 mm and more, steel sheaves with grooves of radius 0.53 d and well lubricated
     * ropes. Reverse bends, plastic sheaves, fleet angles and poor lubrication change the life a great deal and are
     * left to the caller. With the discard constants of a Construction this is the mean (half the ropes are discarded
     * sooner). Zero tension: never (positive infinity). Throws where the regression's tension term would not shorten
     * the life (D/d below about 1.6 for the six-strand constants, 4.1 for the 34x7).
     */
    public static double bendsToDiscard(Feyrer constants, double ropeDiameterM, double sheaveDiameterM, double tensionN,
                                        double gradePa, double bendingLengthM) {
        Objects.requireNonNull(constants, "constants");
        Check.positive(ropeDiameterM, "Rope diameter"); Check.positive(sheaveDiameterM, "Sheave diameter");
        Check.nonNegative(tensionN, "Tension"); Check.positive(gradePa, "Rope grade"); Check.positive(bendingLengthM, "Bending length");
        if (ropeDiameterM < FEYRER_MIN_DIAMETER_M) { throw new IllegalArgumentException("Feyrer's regression is for ropes of 6 mm and more"); }
        if (bendingLengthM < 10 * ropeDiameterM) { throw new IllegalArgumentException("The bending length must be at least 10 rope diameters"); }
        double dMm = ropeDiameterM * 1000, ratio = sheaveDiameterM / ropeDiameterM, lgRatio = Math.log10(ratio);
        double tensionTerm = constants.a1() + constants.a3() * lgRatio;
        if (!(tensionTerm < 0)) { throw new IllegalArgumentException("D/d " + ratio + " is outside Feyrer's regression"); }
        if (tensionN == 0) { return Double.POSITIVE_INFINITY; }
        double fd = 0.52 / (-0.48 + Math.pow(dMm / 16, 0.3));
        double fl = 1.54 / (2.54 - Math.pow((bendingLengthM / ropeDiameterM - 2.5) / 5.5, -0.14));
        double lgN = constants.a0()
                + tensionTerm * (Math.log10(tensionN / (dMm * dMm)) - 0.4 * Math.log10(gradePa / 1770e6))
                + constants.a2() * lgRatio + Math.log10(fd) + Math.log10(fl);
        return Math.pow(10, lgN);
    }
}

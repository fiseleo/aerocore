package dev.mcmod.aerocore.api;

/**
 * Four-quadrant model of a propeller of constant-chord blades from its geometry.
 * <ul>
 * <li>Inside the Wageningen B-series range (2 to 7 blades, expanded area ratio 0.30 to 1.05, pitch ratio 0.5 to 1.4;
 *     Oosterveld and van Oossanen 1975): the B-series regression ahead, and astern and windmilling MARIN's measured
 *     four-quadrant data for the three tested screws (B3-65 P/D 1.0, B4-70 P/D 1.0 and 1.4) or blade-element momentum
 *     theory of the same geometry otherwise, joined by 15 degree ramps (WageningenB.fourQuadrant).</li>
 * <li>Outside it: blade-element momentum theory in all quadrants (BladeElementPropeller) with the marine section fit
 *     of TestPlatform.PROPELLER_SECTION. This is also the model for air propellers far from B-series shapes.</li>
 * </ul>
 */
public final class PropellerModels {
    private PropellerModels() {}

    /** Constant blade chord over diameter giving an expanded area ratio: (A_E/A_0) pi / (2 Z (1 - h)). */
    public static double chordRatio(int blades, double areaRatio, double hubRatio) { return areaRatio * Math.PI / (2 * blades * (1 - hubRatio)); }

    public static boolean inBSeriesRange(int blades, double areaRatio, double pitchRatio) {
        return blades >= 2 && blades <= 7 && areaRatio >= 0.30 && areaRatio <= 1.05 && pitchRatio >= 0.5 && pitchRatio <= 1.4;
    }

    /** The four-quadrant model, tabulated at 1 degree (built in a fraction of a second; cache it). */
    public static PropellerModel fourQuadrant(int blades, double areaRatio, double pitchRatio, double hubRatio) {
        BladeElementPropeller bemt = new BladeElementPropeller(blades, hubRatio, chordRatio(blades, areaRatio, hubRatio), pitchRatio,
                TestPlatform.PROPELLER_SECTION, 20);
        if (!inBSeriesRange(blades, areaRatio, pitchRatio)) { return bemt.tabulate(361); }
        WageningenB series = new WageningenB(blades, areaRatio, pitchRatio);
        PropellerModel elsewhere = bemt;
        if (blades == 3 && areaRatio == 0.65 && pitchRatio == 1.0) { elsewhere = BSeriesFourQuadrant.B3_65_PITCH_1_0; }
        else if (blades == 4 && areaRatio == 0.70 && pitchRatio == 1.0) { elsewhere = BSeriesFourQuadrant.B4_70_PITCH_1_0; }
        else if (blades == 4 && areaRatio == 0.70 && pitchRatio == 1.4) { elsewhere = BSeriesFourQuadrant.B4_70_PITCH_1_4; }
        return series.fourQuadrant(elsewhere, Math.toRadians(15), 361);
    }
}

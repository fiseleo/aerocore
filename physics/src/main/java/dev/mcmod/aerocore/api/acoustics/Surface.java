package dev.mcmod.aerocore.api.acoustics;

import dev.mcmod.aerocore.api.ContactMaterial;

/**
 * What a surface does to sound (0.37): its random-incidence absorption coefficients in the octave bands 125 Hz to
 * 4 kHz, and, for impacts on it, a density, a loss factor and how it rings. Absorption from the usual tables
 * (Everest and Pohlmann, Master Handbook of Acoustics, 6th ed., appendix; Kuttruff, Room Acoustics, table 6.2):
 * concrete or stone 0.01-0.02, steel plate, wood panelling on battens, glass 6 mm, heavy fabric and carpet, grass and
 * soil, sand, gravel, fresh snow, ice, water. OPEN is sky: everything leaves.
 */
public enum Surface {
    ROCK(new double[]{0.01, 0.01, 0.015, 0.02, 0.02, 0.02}, 2400, 0.01, true),
    STEEL(new double[]{0.01, 0.01, 0.01, 0.02, 0.02, 0.02}, 7850, 0.001, true),
    WOOD(new double[]{0.15, 0.11, 0.10, 0.07, 0.06, 0.07}, 600, 0.03, true),
    GLASS(new double[]{0.18, 0.06, 0.04, 0.03, 0.02, 0.02}, 2500, 0.002, true),
    WOOL(new double[]{0.08, 0.24, 0.57, 0.69, 0.71, 0.73}, 300, 0.3, false),
    SOIL(new double[]{0.15, 0.25, 0.40, 0.55, 0.60, 0.60}, 1500, 0.3, false),
    SAND(new double[]{0.15, 0.35, 0.40, 0.50, 0.55, 0.80}, 1600, 0.4, false),
    GRAVEL(new double[]{0.25, 0.60, 0.65, 0.70, 0.75, 0.80}, 1800, 0.3, false),
    SNOW(new double[]{0.45, 0.75, 0.90, 0.95, 0.95, 0.95}, 300, 0.5, false),
    ICE(new double[]{0.01, 0.01, 0.01, 0.02, 0.02, 0.02}, 917, 0.01, true),
    MUD(new double[]{0.05, 0.08, 0.10, 0.15, 0.20, 0.25}, 1700, 0.5, false),
    WATER(new double[]{0.008, 0.008, 0.013, 0.015, 0.02, 0.025}, 1000, 0.5, false),
    LEAVES(new double[]{0.03, 0.06, 0.11, 0.17, 0.27, 0.31}, 200, 0.4, false),
    OPEN(new double[]{1, 1, 1, 1, 1, 1}, 0, 1, false);

    /** Octave band centres of the coefficients, Hz. */
    public static final double[] BANDS = {125, 250, 500, 1000, 2000, 4000};

    private final double[] absorption;
    private final double densityKgM3, lossFactor;
    private final boolean rings;

    Surface(double[] absorption, double densityKgM3, double lossFactor, boolean rings) {
        this.absorption = absorption;
        this.densityKgM3 = densityKgM3;
        this.lossFactor = lossFactor;
        this.rings = rings;
    }

    /** Absorption in octave band i (BANDS). */
    public double absorption(int band) { return absorption[Math.max(0, Math.min(absorption.length - 1, band))]; }
    /** Mean absorption over 500 Hz-1 kHz (the usual single number). */
    public double absorption() { return 0.5 * (absorption[2] + absorption[3]); }
    public double densityKgM3() { return densityKgM3; }
    /** Structural loss factor (a struck plate of it rings with a 60 dB decay of 2.2 / (f eta) s). */
    public double lossFactor() { return lossFactor; }
    /** Whether a blow on it rings (hard), or thuds (soft, granular). */
    public boolean rings() { return rings; }

    /** The surface of a block contact material (AeroCore's block classes); ROCK for anything else. */
    public static Surface of(ContactMaterial m) {
        if (m == null) { return ROCK; }
        if (m.equals(ContactMaterial.STEEL_ON_STEEL)) { return STEEL; }
        if (m.equals(ContactMaterial.STEEL_ON_WOOD)) { return WOOD; }
        if (m.equals(ContactMaterial.STEEL_ON_ICE)) { return ICE; }
        if (m.equals(ContactMaterial.STEEL_ON_SAND)) { return SAND; }
        if (m.equals(ContactMaterial.STEEL_ON_GRAVEL)) { return GRAVEL; }
        if (m.equals(ContactMaterial.STEEL_ON_SOIL)) { return SOIL; }
        if (m.equals(ContactMaterial.STEEL_ON_MUD)) { return MUD; }
        if (m.equals(ContactMaterial.STEEL_ON_SNOW)) { return SNOW; }
        if (m.equals(ContactMaterial.STEEL_ON_WOOL)) { return WOOL; }
        if (m.equals(ContactMaterial.STEEL_ON_GLASS)) { return GLASS; }
        return ROCK;
    }
}

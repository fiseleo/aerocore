package dev.mcmod.aerocore.api.petroleum;

import java.util.EnumMap;
import java.util.Map;

/**
 * A crude oil's assay (0.26): its API gravity and the volume of each true-boiling-point cut. Cuts (TBP, C): gas (C1-C4,
 * below 30), light naphtha 30-90, heavy naphtha 90-180, kerosene 180-240, diesel (atmospheric gas oil) 240-350, vacuum
 * gas oil 350-550 and vacuum residue above, each with a typical density at 15 C (Gary, Handwerk and Kaiser, Petroleum
 * Refining, 5th ed., ch. 3-4). Mass yields are volume times the cut's density, normalized (the cut densities are
 * typical, not the crude's own).
 * <ul>
 * <li>Light sweet (Brent-like, API 38, 0.4 % sulfur): 3, 8, 16, 12, 22, 28, 11 vol %.</li>
 * <li>Heavy sour (Maya-like, API 22, 3.3 % sulfur): 1.5, 4, 9, 9, 15, 24, 37.5 vol %.</li>
 * </ul>
 */
public record CrudeAssay(String name, double api, double sulfurWt, Map<Cut, Double> volume) {
    /** A cut: its boiling range (C) and typical density (kg/m^3) in a light and a heavy crude. */
    public enum Cut {
        GAS(-160, 30, 550, 550), LIGHT_NAPHTHA(30, 90, 670, 680), HEAVY_NAPHTHA(90, 180, 750, 760), KEROSENE(180, 240, 800, 810),
        DIESEL(240, 350, 850, 860), VACUUM_GAS_OIL(350, 550, 920, 940), RESIDUE(550, 900, 1010, 1040);

        public final double fromC, toC, lightDensity, heavyDensity;

        Cut(double fromC, double toC, double lightDensity, double heavyDensity) {
            this.fromC = fromC; this.toC = toC; this.lightDensity = lightDensity; this.heavyDensity = heavyDensity;
        }
    }

    public CrudeAssay {
        volume = Map.copyOf(volume);
        double sum = 0;
        for (Cut c : Cut.values()) { sum += volume.getOrDefault(c, 0.0); }
        if (Math.abs(sum - 1) > 1e-6) { throw new IllegalArgumentException("Cut volumes must add up to 1, not " + sum); }
    }

    private static Map<Cut, Double> cuts(double... v) {
        Map<Cut, Double> m = new EnumMap<>(Cut.class);
        for (int i = 0; i < v.length; i++) { m.put(Cut.values()[i], v[i]); }
        return m;
    }

    public static final CrudeAssay LIGHT_SWEET = new CrudeAssay("light_sweet", 38, 0.4, cuts(0.03, 0.08, 0.16, 0.12, 0.22, 0.28, 0.11));
    public static final CrudeAssay HEAVY_SOUR = new CrudeAssay("heavy_sour", 22, 3.3, cuts(0.015, 0.04, 0.09, 0.09, 0.15, 0.24, 0.375));

    /** Specific gravity (60/60 F) from API gravity: 141.5 / (API + 131.5). */
    public static double specificGravity(double api) { return 141.5 / (api + 131.5); }

    /** The crude's density at 15 C, kg/m^3 (water 999 kg/m^3). */
    public double densityKgM3() { return specificGravity(api) * 999.0; }

    /** A cut's typical density in this crude (light below API 30 grades to heavy), kg/m^3. */
    public double cutDensity(Cut c) {
        double t = Math.clamp((38 - api) / (38 - 22), 0, 1);
        return c.lightDensity + t * (c.heavyDensity - c.lightDensity);
    }

    public double volumeYield(Cut c) { return volume.getOrDefault(c, 0.0); }

    /** A cut's share of the crude's mass. */
    public double massYield(Cut c) {
        double total = 0;
        for (Cut k : Cut.values()) { total += volumeYield(k) * cutDensity(k); }
        return volumeYield(c) * cutDensity(c) / total;
    }

    /** The mass share boiling below a temperature (C) at the cut points, linear within a cut. */
    public double massBoiledBelow(double celsius) {
        double share = 0;
        for (Cut c : Cut.values()) {
            share += massYield(c) * Math.clamp((celsius - c.fromC) / (c.toC - c.fromC), 0, 1);
        }
        return share;
    }
}

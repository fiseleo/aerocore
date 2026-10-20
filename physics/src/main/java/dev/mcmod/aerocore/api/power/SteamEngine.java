package dev.mcmod.aerocore.api.power;

/**
 * A simple reciprocating steam engine (0.26): saturated steam from a boiler at a pressure expands to the atmosphere
 * (non-condensing); its ideal (Rankine) work per kg is the enthalpy drop of an isentropic expansion less the feed pump's
 * work; the engine gets a diagram factor of it (the indicated over the ideal: cut-off, wire drawing, clearance; 0.5-0.7
 * for simple engines), loses a little more to friction, and the boiler passes on a share of its fuel's heat to the steam.
 * Saturated steam properties: IAPWS-IF97 (as tabulated in Cengel and Boles, Thermodynamics, table A-5), 100 kPa to
 * 2 MPa, interpolated linearly in pressure. Simple engines of the line-shaft era turned 5-10 % of their coal into work.
 */
public record SteamEngine(double boilerPa, double diagramFactor, double mechanicalEfficiency, double boilerEfficiency) {
    public SteamEngine {
        if (!(boilerPa > 110e3 && boilerPa <= 2e6)) { throw new IllegalArgumentException("Boiler pressure 110 kPa to 2 MPa"); }
        if (!(diagramFactor > 0 && diagramFactor <= 1) || !(mechanicalEfficiency > 0 && mechanicalEfficiency <= 1)
                || !(boilerEfficiency > 0 && boilerEfficiency <= 1)) {
            throw new IllegalArgumentException("Factors in (0, 1]");
        }
    }

    /** A small mill engine: 1 MPa (10 bar) saturated, diagram factor 0.6, 85 % mechanical, a 70 % boiler. */
    public static final SteamEngine MILL = new SteamEngine(1e6, 0.6, 0.85, 0.7);

    /** Saturated water and steam at a pressure: temperature (C), hf, hg (J/kg), sf, sg (J/kg K), vf (m^3/kg). */
    public record Saturation(double pressurePa, double temperatureC, double hf, double hg, double sf, double sg, double vf) {
        public double hfg() { return hg - hf; }
        public double sfg() { return sg - sf; }
    }

    // kPa, T C, hf, hg (kJ/kg), sf, sg (kJ/kg K), vf (m^3/kg)
    private static final double[][] TABLE = {
            {100, 99.61, 417.51, 2675.0, 1.3028, 7.3589, 0.001043},
            {101.325, 99.97, 419.06, 2675.6, 1.3069, 7.3545, 0.001043},
            {125, 105.97, 444.36, 2684.9, 1.3741, 7.2841, 0.001048},
            {150, 111.35, 467.13, 2693.1, 1.4337, 7.2231, 0.001053},
            {200, 120.21, 504.70, 2706.3, 1.5302, 7.1270, 0.001061},
            {300, 133.52, 561.43, 2724.9, 1.6717, 6.9917, 0.001073},
            {400, 143.61, 604.66, 2738.1, 1.7765, 6.8955, 0.001084},
            {500, 151.83, 640.09, 2748.1, 1.8604, 6.8207, 0.001093},
            {600, 158.83, 670.38, 2756.2, 1.9308, 6.7593, 0.001101},
            {800, 170.41, 720.87, 2768.3, 2.0457, 6.6616, 0.001115},
            {1000, 179.88, 762.51, 2777.1, 2.1381, 6.5850, 0.001127},
            {1200, 187.96, 798.33, 2783.8, 2.2159, 6.5217, 0.001138},
            {1500, 198.29, 844.55, 2791.0, 2.3143, 6.4430, 0.001154},
            {2000, 212.38, 908.47, 2798.3, 2.4467, 6.3390, 0.001177}};

    /** Saturated properties at a pressure (Pa, 100 kPa to 2 MPa). */
    public static Saturation saturation(double pressurePa) {
        double kpa = Math.clamp(pressurePa / 1000, TABLE[0][0], TABLE[TABLE.length - 1][0]);
        int i = 1;
        while (i < TABLE.length - 1 && TABLE[i][0] < kpa) { i++; }
        double[] a = TABLE[i - 1], b = TABLE[i];
        double t = (kpa - a[0]) / (b[0] - a[0]);
        double[] v = new double[7];
        for (int k = 0; k < 7; k++) { v[k] = a[k] + t * (b[k] - a[k]); }
        return new Saturation(v[0] * 1000, v[1], v[2] * 1000, v[3] * 1000, v[4] * 1000, v[5] * 1000, v[6]);
    }

    public static final double ATMOSPHERE_PA = 101325;

    /** Ideal (Rankine) work per kg of steam, expansion to the atmosphere, less the feed pump's, J/kg. */
    public double rankineWorkJPerKg() {
        Saturation hot = saturation(boilerPa), cold = saturation(ATMOSPHERE_PA);
        double x = (hot.sg() - cold.sf()) / cold.sfg();
        double h2 = cold.hf() + x * cold.hfg();
        return hot.hg() - h2 - cold.vf() * (boilerPa - ATMOSPHERE_PA);
    }

    /** Heat the boiler gives each kg of steam (feed water at the exhaust's saturation temperature), J/kg. */
    public double heatJPerKg() {
        Saturation cold = saturation(ATMOSPHERE_PA);
        return saturation(boilerPa).hg() - (cold.hf() + cold.vf() * (boilerPa - ATMOSPHERE_PA));
    }

    /** The Rankine cycle's efficiency. */
    public double rankineEfficiency() { return rankineWorkJPerKg() / heatJPerKg(); }

    /** Shaft work per kg of steam, J/kg. */
    public double shaftJPerKg() { return rankineWorkJPerKg() * diagramFactor * mechanicalEfficiency; }

    /** Shaft work over the fuel's heat. */
    public double overallEfficiency() { return shaftJPerKg() / heatJPerKg() * boilerEfficiency; }

    /** Steam for a shaft power (W), kg/s. */
    public double steamKgPerS(double shaftW) { return Math.max(shaftW, 0) / shaftJPerKg(); }

    /** Fuel of a heating value (J/kg) for a shaft power (W), kg/s. */
    public double fuelKgPerS(double shaftW, double heatingValueJPerKg) { return Math.max(shaftW, 0) / (overallEfficiency() * heatingValueJPerKg); }
}

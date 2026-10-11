package dev.mcmod.aerocore.api.engine;

/**
 * Ideal-gas species of air and combustion products with NASA 7-coefficient polynomials from GRI-Mech 3.0 (Smith et
 * al., gri30 as distributed with Cantera): cp/R, h/(R T) (including the enthalpy of formation, a6) and s/R. The fits
 * cover 200 (N2: 300) to 3500 K (N2: 5000 K, Ar: 5000 K); above the top of a fit cp is held at its value there,
 * below the bottom the low fit is extrapolated (a few kelvin below 300 K for N2 changes cp by less than 0.1 %). Dry
 * air is 20.95 % O2, 78.08 % N2 and 0.93 % Ar by moles (CO2 and traces left out). Dissociation is not modelled.
 */
public enum Species {
    O2(0.0319988, 1000, 3500,
            new double[]{3.78245636, -2.99673416e-03, 9.84730201e-06, -9.68129509e-09, 3.24372837e-12, -1063.94356, 3.65767573},
            new double[]{3.28253784, 1.48308754e-03, -7.57966669e-07, 2.09470555e-10, -2.16717794e-14, -1088.45772, 5.45323129}),
    N2(0.0280134, 1000, 5000,
            new double[]{3.298677, 1.4082404e-03, -3.963222e-06, 5.641515e-09, -2.444854e-12, -1020.8999, 3.950372},
            new double[]{2.92664, 1.4879768e-03, -5.68476e-07, 1.0097038e-10, -6.753351e-15, -922.7977, 5.980528}),
    H2O(0.01801528, 1000, 3500,
            new double[]{4.19864056, -2.0364341e-03, 6.52040211e-06, -5.48797062e-09, 1.77197817e-12, -3.02937267e+04, -0.849032208},
            new double[]{3.03399249, 2.17691804e-03, -1.64072518e-07, -9.7041987e-11, 1.68200992e-14, -3.00042971e+04, 4.9667701}),
    CO2(0.0440095, 1000, 3500,
            new double[]{2.35677352, 8.98459677e-03, -7.12356269e-06, 2.45919022e-09, -1.43699548e-13, -4.83719697e+04, 9.90105222},
            new double[]{3.85746029, 4.41437026e-03, -2.21481404e-06, 5.23490188e-10, -4.72084164e-14, -4.8759166e+04, 2.27163806}),
    CO(0.0280101, 1000, 3500,
            new double[]{3.57953347, -6.1035368e-04, 1.01681433e-06, 9.07005884e-10, -9.04424499e-13, -1.4344086e+04, 3.50840928},
            new double[]{2.71518561, 2.06252743e-03, -9.98825771e-07, 2.30053008e-10, -2.03647716e-14, -1.41518724e+04, 7.81868772}),
    H2(0.00201588, 1000, 3500,
            new double[]{2.34433112, 7.98052075e-03, -1.9478151e-05, 2.01572094e-08, -7.37611761e-12, -917.935173, 0.683010238},
            new double[]{3.3372792, -4.94024731e-05, 4.99456778e-07, -1.79566394e-10, 2.00255376e-14, -950.158922, -3.20502331}),
    AR(0.039948, 1000, 5000,
            new double[]{2.5, 0, 0, 0, 0, -745.375, 4.366},
            new double[]{2.5, 0, 0, 0, 0, -745.375, 4.366});

    /** Universal gas constant, J/(mol K) (CODATA 2018, exact). */
    public static final double R_UNIVERSAL = 8.314462618;
    /** Moles of N2 and of Ar per mole of O2 in dry air. */
    public static final double AIR_N2_PER_O2 = 78.08 / 20.95, AIR_AR_PER_O2 = 0.93 / 20.95;
    /** Mass of air per mole of O2 in it, kg/mol. */
    public static double airMassPerMoleO2() { return O2.molarMass + AIR_N2_PER_O2 * N2.molarMass + AIR_AR_PER_O2 * AR.molarMass; }

    private final double molarMass, middle, top;
    private final double[] low, high;

    Species(double molarMassKgPerMol, double middleK, double topK, double[] low, double[] high) {
        this.molarMass = molarMassKgPerMol; this.middle = middleK; this.top = topK; this.low = low; this.high = high;
    }

    /** kg/mol. */
    public double molarMass() { return molarMass; }

    /** Molar heat capacity at constant pressure, J/(mol K). */
    public double cp(double temperatureK) {
        double t = Math.min(temperatureK, top);
        double[] a = t < middle ? low : high;
        return R_UNIVERSAL * (a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * a[4]))));
    }

    /** Molar enthalpy including formation (zero for O2, N2, H2 at 298.15 K), J/mol. */
    public double enthalpy(double temperatureK) {
        if (temperatureK > top) { return enthalpy(top) + cp(top) * (temperatureK - top); }
        double t = temperatureK;
        double[] a = t < middle ? low : high;
        return R_UNIVERSAL * (t * (a[0] + t * (a[1] / 2 + t * (a[2] / 3 + t * (a[3] / 4 + t * a[4] / 5)))) + a[5]);
    }
}

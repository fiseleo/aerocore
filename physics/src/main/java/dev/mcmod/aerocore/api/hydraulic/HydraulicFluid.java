package dev.mcmod.aerocore.api.hydraulic;

/**
 * A hydraulic fluid: density (kg/m^3), bulk modulus (Pa) and kinematic viscosity (m^2/s), all positive and finite;
 * with the orifice equation and the effective bulk modulus of an aerated oil.
 */
public record HydraulicFluid(double densityKgPerM3, double bulkModulusPa, double kinematicViscosityM2PerS) {
    /** Checks that all three are finite and positive. */
    public HydraulicFluid {
        Hydraulics.positive(densityKgPerM3, "Density");
        Hydraulics.positive(bulkModulusPa, "Bulk modulus");
        Hydraulics.positive(kinematicViscosityM2PerS, "Viscosity");
    }

    /**
     * ISO VG 46 mineral hydraulic oil (the usual grade of mobile machinery) at 50 C, the oil alone.
     * Viscosity: 46 mm^2/s at 40 C by definition of the grade (ISO 3448), about 6.8 at 100 C for a viscosity index
     * near 100; the ASTM D341 (Walther) line through the two gives 30 mm^2/s at 50 C. Density: 870 to 880 kg/m^3 at
     * 15 C, expanding about 6.5e-4 per K, about 860 at 50 C. Bulk modulus: the tangent modulus of mineral oil at
     * moderate pressure is of the order of 1.4 to 1.7 GPa (Totten (ed.), Handbook of Hydraulic Fluid Technology, 2000;
     * Merritt, Hydraulic Control Systems, 1967); 1.5 GPa taken.
     */
    public static final HydraulicFluid ISO_VG46_50C = new HydraulicFluid(860, 1.5e9, 30e-6);

    /**
     * The same oil as a working circuit feels it: some free air and the give of steel lines bring the effective bulk
     * modulus down to about 1.2 GPa (about 0.24 % free air at 10 MPa by effectiveBulkModulusPa). Rubber hoses and more
     * air lower it further: Merritt (1967) advises 0.7 GPa when the air content is not known.
     */
    public static final HydraulicFluid ISO_VG46_50C_IN_CIRCUIT = new HydraulicFluid(860, 1.2e9, 30e-6);

    /**
     * Discharge coefficient of a sharp-edged orifice at high Reynolds number: von Mises' ideal jet contraction
     * pi / (pi + 2) = 0.611 (Merritt 1967); spool valve ports run 0.6 to 0.7.
     */
    public static final double SHARP_EDGED_CD = Math.PI / (Math.PI + 2);

    /**
     * Reynolds number (on the hydraulic diameter) around which orifice flow turns from laminar to turbulent: 12, the
     * default of the orifice blocks in MathWorks Simscape Fluids (SimHydraulics); Merritt (1967) puts laminar orifice
     * flow below a Reynolds number of about 10.
     */
    public static final double ORIFICE_LAMINAR_REYNOLDS = 12;

    /** Heat capacity ratio of air, for the adiabatic compression of entrained air. */
    public static final double AIR_HEAT_RATIO = 1.4;

    /** Dynamic viscosity, Pa s: mu = rho nu. */
    public double dynamicViscosityPaS() { return densityKgPerM3 * kinematicViscosityM2PerS; }

    /**
     * The pressure drop (Pa) at which an orifice's flow turns laminar: where the turbulent jet's Reynolds number on
     * the hydraulic diameter of a round hole, D_h = sqrt(4 A / pi), falls to ORIFICE_LAMINAR_REYNOLDS:
     * p_t = rho / 2 (Re nu / (Cd D_h))^2. Of the order of 10 Pa for a few-mm hole in ISO VG 46 oil.
     */
    public double orificeTransitionPa(double cd, double areaM2) {
        Hydraulics.positive(cd, "Discharge coefficient");
        Hydraulics.positive(areaM2, "Area");
        double dh = Math.sqrt(4 * areaM2 / Math.PI);
        double v = ORIFICE_LAMINAR_REYNOLDS * kinematicViscosityM2PerS / (cd * dh);
        return 0.5 * densityKgPerM3 * v * v;
    }

    /**
     * Flow through an orifice, m^3/s, signed with the pressure drop (Pa) across it: the orifice equation
     * Q = Cd A sqrt(2 dp / rho) (Merritt 1967), regularised as Q = Cd A sqrt(2 / rho) dp / (dp^2 + p_t^2)^(1/4) with
     * the laminar transition p_t of orificeTransitionPa (the form of MathWorks Simscape Fluids' orifice blocks). Smooth
     * and increasing, linear (laminar) for |dp| well below p_t with the finite slope Cd A sqrt(2 / (rho p_t)), within
     * 0.3 % of the square-root law above 10 p_t. A closed orifice (area or Cd zero) passes nothing.
     */
    public double orificeFlowM3PerS(double cd, double areaM2, double deltaPPa) {
        Hydraulics.nonNegative(cd, "Discharge coefficient");
        Hydraulics.nonNegative(areaM2, "Area");
        Hydraulics.finite(deltaPPa, "Pressure drop");
        if (cd == 0 || areaM2 == 0) { return 0; }
        return flow(cd, areaM2, deltaPPa, orificeTransitionPa(cd, areaM2));
    }

    /** The same with the caller's laminar transition pressure (Pa, positive) instead of the Reynolds number's. */
    public double orificeFlowM3PerS(double cd, double areaM2, double deltaPPa, double transitionPa) {
        Hydraulics.nonNegative(cd, "Discharge coefficient");
        Hydraulics.nonNegative(areaM2, "Area");
        Hydraulics.finite(deltaPPa, "Pressure drop");
        Hydraulics.positive(transitionPa, "Transition pressure");
        return flow(cd, areaM2, deltaPPa, transitionPa);
    }

    private double flow(double cd, double areaM2, double dp, double pt) {
        return cd * areaM2 * Math.sqrt(2 / densityKgPerM3) * dp / Math.sqrt(Math.sqrt(dp * dp + pt * pt));
    }

    /**
     * The pressure drop (Pa) that drives a flow (m^3/s, signed) through an orifice: the exact inverse of
     * orificeFlowM3PerS, dp = q sqrt((q^2 + sqrt(q^4 + 4 p_t^2)) / 2) with q = Q / (Cd A sqrt(2 / rho)); the square law
     * dp = rho / 2 (Q / (Cd A))^2 at high flow. Cd and area must be positive.
     */
    public double orificePressureDropPa(double cd, double areaM2, double flowM3PerS) {
        double pt = orificeTransitionPa(cd, areaM2);
        double q = Hydraulics.finite(flowM3PerS, "Flow") / (cd * areaM2 * Math.sqrt(2 / densityKgPerM3));
        double q2 = q * q;
        return q * Math.sqrt(0.5 * (q2 + Math.sqrt(q2 * q2 + 4 * pt * pt)));
    }

    /**
     * Effective bulk modulus (Pa) of a liquid carrying free air: 1 / B_e = 1 / B_liquid + x / (gamma p), x the share
     * of the volume that is free air at the absolute pressure p (Pa), compressed adiabatically (gamma 1.4); Merritt
     * (1967), leaving out the give of the lines. A quarter of a per cent of air at 10 MPa takes a fifth off 1.5 GPa.
     */
    public static double effectiveBulkModulusPa(double liquidBulkModulusPa, double airVolumeShare, double absolutePressurePa) {
        Hydraulics.positive(liquidBulkModulusPa, "Liquid bulk modulus");
        if (!(airVolumeShare >= 0 && airVolumeShare < 1)) { throw new IllegalArgumentException("Air share must be in [0, 1)"); }
        Hydraulics.positive(absolutePressurePa, "Absolute pressure");
        return 1 / (1 / liquidBulkModulusPa + airVolumeShare / (AIR_HEAT_RATIO * absolutePressurePa));
    }
}

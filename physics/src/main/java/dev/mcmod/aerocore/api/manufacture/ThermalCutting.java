package dev.mcmod.aerocore.api.manufacture;

/**
 * Laser and plasma cutting (0.35): the lumped heat balance of the kerf, after Steen (Laser Material Processing, ch. on
 * cutting) and Groover (sec. 26.4).
 *
 * <ul>
 * <li>The beam or arc delivers eta P into the kerf: a slot of width w through a thickness t, moving at v, melts
 *     rho w t v kg a second, each needing e = c_p (T_m - T_0) + L_f (+ a vaporised share of L_v). So
 *     v = eta P / (rho w t e). eta lumps the coupling and the heat conducted away (it is low: a few tenths).</li>
 * <li>With oxygen, burning iron to FeO adds its heat of reaction (272 kJ/mol FeO, 4.87 MJ a kilogram of iron) on the
 *     share of the kerf's iron that burns: e drops by that share times 4.87 MJ/kg.</li>
 * <li>A plasma torch's power is its arc voltage times its current.</li>
 * </ul>
 */
public final class ThermalCutting {
    private ThermalCutting() {}

    /** Heat of burning iron to FeO, a kilogram of iron, J/kg (standard enthalpy of formation of FeO over iron's molar mass). */
    public static final double IRON_OXIDATION_J_PER_KG = 272.0e3 / 0.055845;

    /** Energy to melt a kilogram from a temperature (K): specific heat (J/kg K), melting point (K), heat of fusion, vaporised share and heat of vaporisation (J/kg). */
    public static double meltJPerKg(double specificHeat, double fromK, double meltingK, double fusionJPerKg, double vaporisedShare, double vaporisationJPerKg) {
        if (!(specificHeat > 0) || !(meltingK > fromK) || !(vaporisedShare >= 0 && vaporisedShare <= 1)) { throw new IllegalArgumentException("Bad melt"); }
        return specificHeat * (meltingK - fromK) + Math.max(fusionJPerKg, 0) + vaporisedShare * Math.max(vaporisationJPerKg, 0);
    }

    /** The energy a kilogram with a share of the iron burnt in oxygen (it supplies the rest), J/kg. */
    public static double withOxygenJPerKg(double meltJPerKg, double burntShare) {
        if (!(burntShare >= 0 && burntShare <= 1)) { throw new IllegalArgumentException("Burnt share in [0, 1]"); }
        double e = meltJPerKg - burntShare * IRON_OXIDATION_J_PER_KG;
        if (!(e > 0)) { throw new IllegalArgumentException("The oxygen cannot supply all of the heat"); }
        return e;
    }

    /** Cutting speed: power (W), efficiency, kerf width and thickness (m), density (kg/m^3) and energy a kilogram (J/kg), m/s. */
    public static double speedMps(double powerW, double efficiency, double kerfM, double thicknessM, double density, double energyJPerKg) {
        if (!(powerW > 0) || !(efficiency > 0 && efficiency <= 1) || !(kerfM > 0) || !(thicknessM > 0) || !(density > 0) || !(energyJPerKg > 0)) {
            throw new IllegalArgumentException("Power, efficiency, sizes, density and energy must be positive");
        }
        return efficiency * powerW / (density * kerfM * thicknessM * energyJPerKg);
    }

    /** Time to cut a length (m) at a speed (m/s), with a pierce time (s) for each start, s. */
    public static double cutS(double lengthM, double speedMps, int pierces, double pierceS) {
        if (!(speedMps > 0)) { throw new IllegalArgumentException("Speed must be positive"); }
        return Math.max(lengthM, 0) / speedMps + Math.max(pierces, 0) * Math.max(pierceS, 0);
    }

    /** A plasma torch's power: arc voltage (V) and current (A), W. */
    public static double arcW(double volts, double currentA) { return Math.max(volts, 0) * Math.max(currentA, 0); }
}

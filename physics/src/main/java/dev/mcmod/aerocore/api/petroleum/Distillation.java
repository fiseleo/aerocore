package dev.mcmod.aerocore.api.petroleum;

/**
 * Crude distillation (0.26).
 * <ul>
 * <li>Column size: the Souders-Brown flooding velocity u = C sqrt((rho_L - rho_V) / rho_V) (C about 0.08 m/s for
 *     trays 0.6 m apart), designed at 80 % of it; the vapour's flow through the area gives the diameter (Kister,
 *     Distillation Design, ch. 6).</li>
 * <li>Heater: the crude is heated to about 360 C (a 25 C feed), cp 2.3 kJ/kg K for the liquid; the part boiling below
 *     the flash temperature also takes its heat of vaporization (about 300 kJ/kg); a preheat train recovers a share of
 *     the sensible heat from the products (about 60 %). The heater burns fuel for the rest: 1-2 % of the crude
 *     (Gary, Handwerk and Kaiser, ch. 4). 0.27: at another flash temperature (a column cutting its diesel higher heats
 *     its crude further), the same balance at that temperature.</li>
 * </ul>
 */
public final class Distillation {
    private Distillation() {}

    public static final double SOUDERS_BROWN_C = 0.08, FLOODING = 0.8, CRUDE_CP = 2300, LATENT = 300e3, FLASH_C = 360, FEED_C = 25, PREHEAT_RECOVERY = 0.6;

    /** Flooding vapour velocity, m/s. */
    public static double floodingVelocity(double liquidDensity, double vapourDensity) {
        return SOUDERS_BROWN_C * Math.sqrt(Math.max(liquidDensity - vapourDensity, 0) / vapourDensity);
    }

    /** Column diameter for a vapour flow (kg/s) at 80 % of flooding, m. */
    public static double diameterM(double vapourKgPerS, double liquidDensity, double vapourDensity) {
        double area = vapourKgPerS / (vapourDensity * FLOODING * floodingVelocity(liquidDensity, vapourDensity));
        return Math.sqrt(4 * area / Math.PI);
    }

    /** Vapour a column of a diameter (m) carries at 80 % of flooding, kg/s. */
    public static double capacityKgPerS(double diameterM, double liquidDensity, double vapourDensity) {
        return Math.PI * diameterM * diameterM / 4 * vapourDensity * FLOODING * floodingVelocity(liquidDensity, vapourDensity);
    }

    /** The heater's duty per kg of a crude (sensible less what the preheat train recovers, plus the flashed share's vaporization), J/kg. */
    public static double heaterJPerKg(CrudeAssay crude) { return heaterJPerKg(crude, FLASH_C); }

    /** The same at a flash temperature, C (0.27). */
    public static double heaterJPerKg(CrudeAssay crude, double flashC) {
        double sensible = CRUDE_CP * (flashC - FEED_C);
        return sensible * (1 - PREHEAT_RECOVERY) + crude.massBoiledBelow(flashC) * LATENT;
    }

    /** Fuel the heater burns per kg of crude, kg/kg, at a fuel's heating value (J/kg) and a furnace efficiency. */
    public static double heaterFuelShare(CrudeAssay crude, double heatingValueJPerKg, double furnaceEfficiency) {
        return heaterFuelShare(crude, FLASH_C, heatingValueJPerKg, furnaceEfficiency);
    }

    /** The same at a flash temperature, C (0.27). */
    public static double heaterFuelShare(CrudeAssay crude, double flashC, double heatingValueJPerKg, double furnaceEfficiency) {
        return heaterJPerKg(crude, flashC) / (heatingValueJPerKg * furnaceEfficiency);
    }
}

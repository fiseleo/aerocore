package dev.mcmod.aerocore.api.manufacture;

/**
 * Casting (0.25): how long a casting takes to solidify and how much heat its metal takes to melt.
 *
 * <ul>
 * <li>Chvorinov's rule (1940; Groover eq. 10.7): the total solidification time is T = C_m (V / A)^n with n = 2, C_m
 *     the mould constant of the metal and the mould (s/m^2; found from a test casting: a 50 mm cube of an alloy that
 *     solidifies in 155 s gives 2.232 s/mm^2, Groover example 10.2).</li>
 * <li>Heat to melt and pour (Groover eq. 10.1): H = m (c_s (T_m - T_0) + L + c_l (T_p - T_m)), the solid's heat to the
 *     melting point, the latent heat, and the liquid's superheat to the pouring temperature.</li>
 * </ul>
 */
public final class Casting {
    private Casting() {}

    /** Total solidification time of a casting of a volume (m^3) and surface (m^2) in a mould of a constant (s/m^2), s. */
    public static double solidificationS(double volumeM3, double areaM2, double mouldConstantSPerM2) {
        if (!(volumeM3 > 0) || !(areaM2 > 0) || !(mouldConstantSPerM2 > 0)) { throw new IllegalArgumentException("Volume, area and mould constant must be positive"); }
        double modulus = volumeM3 / areaM2;
        return mouldConstantSPerM2 * modulus * modulus;
    }

    /** The mould constant (s/m^2) a test casting of a volume and area that solidified in a time gives. */
    public static double mouldConstant(double volumeM3, double areaM2, double seconds) {
        double modulus = volumeM3 / areaM2;
        return seconds / (modulus * modulus);
    }

    /**
     * Heat to bring a mass (kg) from an ambient temperature to its melting point (solid specific heat, J/kg K), melt
     * it (latent heat, J/kg) and superheat it to the pouring temperature (liquid specific heat), J.
     */
    public static double meltJ(double massKg, double solidSpecificHeat, double ambientK, double meltingK, double latentJPerKg, double liquidSpecificHeat,
                               double pouringK) {
        return massKg * (solidSpecificHeat * Math.max(meltingK - ambientK, 0) + latentJPerKg + liquidSpecificHeat * Math.max(pouringK - meltingK, 0));
    }
}

package dev.mcmod.aerocore.api.power;

/**
 * Flat belt drives (0.26), as line shafts drove machine tools: the tight side may carry the belt's allowable stress
 * times its section; the slack side's tension is the tight side's over e^(mu theta) (Euler-Eytelwein, for the wrap
 * angle theta on the smaller pulley); the centrifugal tension m' v^2 takes from both. Power = (T1 - Tc)(1 - e^(-mu
 * theta)) v. Leather on cast iron: mu about 0.35, allowable stress about 2.5 MPa, density 970 kg/m^3 (Shigley's
 * Mechanical Engineering Design, ch. 17; Machinery's Handbook). A flat belt loses 1-2 % to creep and 2-3 % to
 * bending and air: efficiency about 0.96.
 */
public final class Belt {
    private Belt() {}

    public static final double LEATHER_MU = 0.35, LEATHER_STRESS_PA = 2.5e6, LEATHER_DENSITY = 970, EFFICIENCY = 0.96;

    /** Wrap angle on the smaller of two pulleys (diameters m) a centre distance (m) apart, open belt, rad. */
    public static double wrapRad(double largeM, double smallM, double centreM) {
        double big = Math.max(largeM, smallM), small = Math.min(largeM, smallM);
        return Math.PI - 2 * Math.asin(Math.clamp((big - small) / (2 * centreM), -1, 1));
    }

    /** Power a flat belt (width, thickness m) transmits at a speed (m/s) over a wrap (rad), W. */
    public static double powerW(double widthM, double thicknessM, double speedMps, double wrapRad, double mu, double allowableStressPa, double density) {
        double t1 = allowableStressPa * widthM * thicknessM, tc = density * widthM * thicknessM * speedMps * speedMps;
        return Math.max(0, (t1 - tc) * (1 - Math.exp(-mu * wrapRad)) * speedMps);
    }

    /** Power a leather belt transmits, W. */
    public static double leatherW(double widthM, double thicknessM, double speedMps, double wrapRad) {
        return powerW(widthM, thicknessM, speedMps, wrapRad, LEATHER_MU, LEATHER_STRESS_PA, LEATHER_DENSITY);
    }
}

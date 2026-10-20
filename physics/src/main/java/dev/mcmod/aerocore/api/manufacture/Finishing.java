package dev.mcmod.aerocore.api.manufacture;

/**
 * Abrasive finishing (0.35): lapping, polishing and honing, after Preston (1927, "The theory and design of plate glass
 * polishing machines", J. Soc. Glass Technology 11) and Kalpakjian and Schmid (ch. 26).
 *
 * <ul>
 * <li>Preston's equation: lapping and polishing wear the surface down at dh/dt = K_p p v, p the pressure on the lap,
 *     v the speed between them and K_p a coefficient of the work, abrasive and fluid (Pa^-1; about 1e-13 for steel
 *     lapped with alumina, the caller's number). Taking a depth h takes h / (K_p p v).</li>
 * <li>Honing: the stones expand into a bore at a radial removal rate (a few micrometres a second for steel and cast
 *     iron), so a stock takes stock / rate; rotation and reciprocation leave a crosshatch whose included angle is
 *     2 atan(v_axial / v_rotation).</li>
 * </ul>
 */
public final class Finishing {
    private Finishing() {}

    /** Preston's wear rate: coefficient (1/Pa), pressure (Pa) and relative speed (m/s), m/s. */
    public static double prestonMps(double coefficientPerPa, double pressurePa, double speedMps) {
        positive(coefficientPerPa, pressurePa, speedMps);
        return coefficientPerPa * pressurePa * speedMps;
    }

    /** Time to lap or polish a depth (m) off at Preston's rate, s. */
    public static double prestonS(double depthM, double coefficientPerPa, double pressurePa, double speedMps) {
        return Math.max(depthM, 0) / prestonMps(coefficientPerPa, pressurePa, speedMps);
    }

    /** Time to hone a radial stock (m) at a radial removal rate (m/s), s. */
    public static double honingS(double stockM, double removalMps) {
        positive(removalMps, 1, 1);
        return Math.max(stockM, 0) / removalMps;
    }

    /** The crosshatch's included angle of an axial (stroking) and a rotational surface speed (m/s), rad. */
    public static double crosshatchRad(double axialMps, double rotationMps) {
        positive(axialMps, rotationMps, 1);
        return 2 * Math.atan(axialMps / rotationMps);
    }

    /** The axial (stroking) speed that leaves a crosshatch angle (rad) at a rotational surface speed (m/s), m/s. */
    public static double strokingMps(double crosshatchRad, double rotationMps) {
        positive(rotationMps, 1, 1);
        if (!(crosshatchRad > 0 && crosshatchRad < Math.PI)) { throw new IllegalArgumentException("Crosshatch between 0 and 180 degrees"); }
        return rotationMps * Math.tan(crosshatchRad / 2);
    }

    private static void positive(double a, double b, double c) {
        if (!(a > 0) || !(b > 0) || !(c > 0)) { throw new IllegalArgumentException("Coefficients, pressures, speeds and rates must be positive"); }
    }
}

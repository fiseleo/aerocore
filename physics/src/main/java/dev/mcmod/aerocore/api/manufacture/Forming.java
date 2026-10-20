package dev.mcmod.aerocore.api.manufacture;

/**
 * Bulk forming (0.25): forging and rolling loads, and how long a piece takes to heat (or cure) through.
 *
 * <ul>
 * <li>Flow stress: Y = K e^n (Groover eq. 3.11); its average over a strain e is K e^n / (1 + n) (eq. 19.2).</li>
 * <li>Open-die upsetting of a cylinder (Groover eq. 19.12-19.13): F = K_f Y A with the shape factor
 *     K_f = 1 + 0.4 mu D / h, D and h the piece's diameter and height and A its area at that moment (example 19.2: a
 *     50 x 50 mm cylinder upset to 25 mm, K 350 MPa, n 0.17, mu 0.2: 1.58 MN).</li>
 * <li>Flat rolling (Groover eq. 19.5-19.8): the contact length L = sqrt(R d) for a draft d, the force F = Y_avg w L
 *     on a strip of width w, the power of two rolls P = 2 pi N F L (example 19.1: 300 x 25 mm to 22 mm, R 250 mm,
 *     50 rev/min, K 275 MPa, n 0.15: 1.44 MN, 207 kW).</li>
 * <li>Heating through by conduction: a slab of half-thickness h (or a cylinder of radius h) reaches near-uniform
 *     temperature at a Fourier number alpha t / h^2 of about 1 (the centre within 10 % of the surface's step for a
 *     slab; Incropera ch. 5), so t = Fo h^2 / alpha with alpha = k / (rho c).</li>
 * </ul>
 */
public final class Forming {
    private Forming() {}

    /** Flow stress at a true strain, Pa. */
    public static double flowStress(double strengthPa, double hardeningExponent, double strain) {
        return strengthPa * Math.pow(Math.max(strain, 0), hardeningExponent);
    }

    /** Average flow stress over a true strain from zero, Pa. */
    public static double averageFlowStress(double strengthPa, double hardeningExponent, double strain) {
        return flowStress(strengthPa, hardeningExponent, strain) / (1 + hardeningExponent);
    }

    /** Force to upset a cylinder of a volume (m^3) to a height (m) from a starting height, friction mu and its flow curve, N. */
    public static double upsetForceN(double volumeM3, double startHeightM, double heightM, double friction, double strengthPa, double hardeningExponent) {
        double area = volumeM3 / heightM, diameter = Math.sqrt(4 * area / Math.PI), strain = Math.log(startHeightM / heightM);
        double shape = 1 + 0.4 * friction * diameter / heightM;
        return shape * flowStress(strengthPa, hardeningExponent, strain) * area;
    }

    /** Contact length of a roll of a radius (m) taking a draft (m), m. */
    public static double rollContactM(double rollRadiusM, double draftM) { return Math.sqrt(rollRadiusM * Math.max(draftM, 0)); }

    /** Rolling force on a strip of a width (m) from one thickness to another (m) by rolls of a radius (m), N. */
    public static double rollForceN(double widthM, double fromM, double toM, double rollRadiusM, double strengthPa, double hardeningExponent) {
        double strain = Math.log(fromM / toM);
        return averageFlowStress(strengthPa, hardeningExponent, strain) * widthM * rollContactM(rollRadiusM, fromM - toM);
    }

    /** Power of two rolls turning at a speed (rev/s) under a force (N) over a contact length (m), W. */
    public static double rollPowerW(double revsPerS, double forceN, double contactM) { return 2 * Math.PI * revsPerS * forceN * contactM; }

    /** Time to heat (or cure) through a half-thickness (m) of a material of a conductivity, density and specific heat, at a Fourier number, s. */
    public static double soakS(double halfThicknessM, double conductivity, double density, double specificHeat, double fourier) {
        double alpha = conductivity / (density * specificHeat);
        return fourier * halfThicknessM * halfThicknessM / alpha;
    }
}

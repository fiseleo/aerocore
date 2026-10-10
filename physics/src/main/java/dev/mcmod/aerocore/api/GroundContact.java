package dev.mcmod.aerocore.api;

/**
 * Wheels on the ground, for land vehicles: tyre forces on firm surfaces, rolling resistance, and rigid wheels sinking
 * into soft soil.
 * <ul>
 * <li>Tyre: Pacejka's Magic Formula F = D Fz sin(C atan(B s - E (B s - atan(B s)))) (Bakker, Nyborg and Pacejka 1987;
 * Pacejka, Tire and Vehicle Dynamics, 2012) for longitudinal slip ratio and lateral slip angle (rad), with the widely
 * used simplified coefficients per surface (B, C, D, E): dry tarmac 10, 1.9, 1, 0.97; wet tarmac 12, 2.3, 0.82, 1;
 * snow 5, 2, 0.3, 1; ice 4, 2, 0.1, 1 (as tabulated in MathWorks' Magic Formula tire documentation). Combined slip by
 * the friction ellipse of the theoretical slips (Pacejka's similarity method in its simplest form): both forces from
 * the resultant slip, shared by direction.</li>
 * <li>Rolling resistance on firm ground (Wong, Theory of Ground Vehicles, 4th ed., 2008, section 1.2): passenger car
 * tyres on concrete or asphalt 0.013 of the load, packed gravel 0.02. On soft ground the tyre's own 0.013 plus the
 * soil's compaction resistance below, which for a car wheel in dry sand comes to about 0.3 of the load, Wong's
 * figure for sand.</li>
 * <li>Soft soil (Bekker 1956, 1969; Wong's restatement, ch. 2): pressure-sinkage p = (k_c / b + k_phi) z^n under a
 * plate of width b; a rigid wheel of diameter D and width b under load W sinks z0 = (3 W / ((3 - n) b k sqrt(D)))^(2 /
 * (2n + 1)), k = k_c / b + k_phi, and meets compaction resistance R_c = b k z0^(n + 1) / (n + 1); shear (Janosi and
 * Hanamoto 1961) tau = (c + p tan phi)(1 - exp(-j / K)) gives at most F_max = A c + W tan phi over the contact area A.
 * Bekker approximates the load integral over the parabolic contact by (3 - n) / 3; this class integrates it exactly,
 * W = b k sqrt(D) z0^(n + 1/2) sqrt(pi) Gamma(n + 1) / (2 Gamma(n + 3/2)), which differs by 2 % for sand and 19 % for
 * snow (n = 1.6). Soil values from Wong's table 2.3 (Land Locomotion Laboratory and others): dry sand n 1.1, k_c 0.99
 * kN/m^(n+1), k_phi 1528.43 kN/m^(n+2), c 1.04 kPa, phi 28 deg; sandy loam (11 % moisture) 0.9, 52.53, 1127.97, 4.83,
 * 20; clayey soil (38 %) 0.5, 13.19, 692.15, 4.14, 13; heavy clay (25 %) 0.13, 12.70, 1555.95, 68.95, 34; snow (U.S.)
 * 1.6, 4.37, 196.72, 1.03, 19.7; K = 0.025 m. Small sinkage is assumed (z0 well below D / 2).</li>
 * </ul>
 */
public final class GroundContact {
    /** Magic Formula coefficients. */
    public record MagicFormula(double b, double c, double d, double e) {
        public static final MagicFormula DRY = new MagicFormula(10, 1.9, 1, 0.97), WET = new MagicFormula(12, 2.3, 0.82, 1),
                SNOW = new MagicFormula(5, 2, 0.3, 1), ICE = new MagicFormula(4, 2, 0.1, 1);

        /** Force over load at a slip (ratio, or angle in rad). */
        public double ratio(double slip) {
            double x = b * slip;
            return d * Math.sin(c * Math.atan(x - e * (x - Math.atan(x))));
        }
    }

    /** Bekker-Wong soil: exponent n, k_c (kN/m^(n+1)), k_phi (kN/m^(n+2)), cohesion c (kPa), friction angle (rad), shear modulus K (m). */
    public record Soil(double n, double kc, double kphi, double cohesionKpa, double frictionAngle, double shearModulusM) {
        public static final Soil DRY_SAND = new Soil(1.1, 0.99, 1528.43, 1.04, Math.toRadians(28), 0.025);
        public static final Soil SANDY_LOAM = new Soil(0.9, 52.53, 1127.97, 4.83, Math.toRadians(20), 0.025);
        public static final Soil FIRM_CLAY = new Soil(0.13, 12.70, 1555.95, 68.95, Math.toRadians(34), 0.025);
        public static final Soil CLAYEY = new Soil(0.5, 13.19, 692.15, 4.14, Math.toRadians(13), 0.025);
        public static final Soil SNOW = new Soil(1.6, 4.37, 196.72, 1.03, Math.toRadians(19.7), 0.025);

        /** Plate pressure at sinkage z (m) under width b (m), Pa. */
        public double pressure(double z, double width) { return 1000 * (kc / width + kphi) * Math.pow(Math.max(z, 0), n); }
    }

    /** A surface: tyre grip, rolling resistance on firm ground, and its soil when soft (null when firm). */
    public record Surface(String name, MagicFormula grip, double rollingResistance, Soil soil) {
        public static final Surface TARMAC = new Surface("tarmac", MagicFormula.DRY, 0.013, null);
        public static final Surface WET_TARMAC = new Surface("wet tarmac", MagicFormula.WET, 0.013, null);
        public static final Surface GRAVEL = new Surface("gravel", new MagicFormula(10, 1.9, 0.6, 0.97), 0.02, null);
        public static final Surface ICE = new Surface("ice", MagicFormula.ICE, 0.013, null);
        public static final Surface SAND = new Surface("sand", new MagicFormula(8, 1.9, 0.6, 1), 0.013, Soil.DRY_SAND);
        public static final Surface SOIL = new Surface("soil", new MagicFormula(8, 1.9, 0.55, 1), 0.013, Soil.FIRM_CLAY);
        public static final Surface MUD = new Surface("mud", new MagicFormula(6, 1.9, 0.35, 1), 0.013, Soil.CLAYEY);
        public static final Surface SNOW = new Surface("snow", MagicFormula.SNOW, 0.013, Soil.SNOW);
    }

    /** A rigid wheel's sinkage (m), compaction resistance (N) and the soil's largest tractive effort (N). */
    public record Sinkage(double sinkageM, double compactionN, double maxTractionN) {}

    private GroundContact() {}

    /** Bekker's rigid wheel of diameter D (m) and width b (m) under load W (N) on soil. */
    public static Sinkage rigidWheel(Soil soil, double loadN, double diameterM, double widthM) {
        double k = 1000 * (soil.kc() / widthM + soil.kphi()), n = soil.n();
        double integral = Math.sqrt(Math.PI) * Math.exp(SeaSpectrum.logGamma(n + 1) - SeaSpectrum.logGamma(n + 1.5)) / 2;
        double z0 = Math.pow(loadN / (widthM * k * Math.sqrt(diameterM) * integral), 2 / (2 * n + 1));
        double compaction = widthM * k * Math.pow(z0, n + 1) / (n + 1);
        // Contact length: the chord from the deepest point to where the wheel meets the undisturbed surface.
        double length = Math.sqrt(Math.max(diameterM * z0 - z0 * z0, 0));
        double traction = widthM * length * soil.cohesionKpa() * 1000 + loadN * Math.tan(soil.frictionAngle());
        return new Sinkage(z0, compaction, traction);
    }

    /** Janosi-Hanamoto shear stress at shear displacement j (m) under normal pressure p (Pa), Pa. */
    public static double shear(Soil soil, double pressurePa, double displacementM) {
        return (soil.cohesionKpa() * 1000 + pressurePa * Math.tan(soil.frictionAngle())) * (1 - Math.exp(-Math.max(displacementM, 0) / soil.shearModulusM()));
    }

    /**
     * Tyre forces {longitudinal, lateral} (N) at slip ratio kappa and slip angle alpha (rad) under load Fz (N): each from
     * the Magic Formula at the resultant slip sqrt(kappa^2 + tan^2 alpha), shared by the slip's direction (friction
     * ellipse), the lateral force opposing the slip angle.
     */
    public static double[] tyreForces(MagicFormula grip, double kappa, double alpha, double loadN) {
        double lateral = Math.tan(alpha), resultant = Math.hypot(kappa, lateral);
        if (resultant == 0) { return new double[]{0, 0}; }
        double f = grip.ratio(resultant) * Math.max(loadN, 0);
        return new double[]{f * kappa / resultant, -f * lateral / resultant};
    }

    /**
     * Resistance to rolling (N) of a wheel under load W on a surface: compaction resistance when soft (Bekker), else
     * the rolling resistance coefficient times the load.
     */
    public static double rollingResistance(Surface surface, double loadN, double diameterM, double widthM) {
        double tyre = surface.rollingResistance() * loadN;
        return surface.soil() == null ? tyre : tyre + rigidWheel(surface.soil(), loadN, diameterM, widthM).compactionN();
    }
}

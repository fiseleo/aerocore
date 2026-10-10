package dev.mcmod.aerocore.api;

/**
 * Combined body-obstacle Coulomb friction coefficient (>= 0) and normal restitution. With a finite yield velocity V_y
 * the restitution follows Thornton's (1997, J. Appl. Mech. 64:383) elastic-perfectly-plastic sphere impact, as
 * restated by Jackson, Green and Marghitu (2010, Nonlinear Dyn. 60:217, eq. 2): restitution (the elastic limit, 1 for
 * an ideal contact) up to V_y, then e = 1.442 sqrt(1 - r^2/6) (r / (r + 2 sqrt(6/5 - r^2/5)))^(1/4) with r = V_y / V,
 * tending to 1.185 r^(1/4) (Johnson's V^(-1/4) law). V_y follows from the yield stress, the contact modulus and the
 * density (yieldVelocity). Collision shapes here are spheres, so the sphere-impact law applies to every contact.
 */
public record ContactMaterial(double friction, double restitution, double yieldVelocityMps) {
    /** Structural steel (ASTM A36 minimum yield 250 MPa, E 200 GPa, nu 0.30, rho 7854 kg/m^3, Incropera plain carbon steel). */
    public static final double STEEL_YIELD_PA = 250e6, STEEL_MODULUS_PA = 200e9, STEEL_POISSON = 0.30, STEEL_DENSITY = 7854;
    /** Granite, mean of 24 samples (Johnson and DeGraff 1988, via Amadei's notes): E 59.3 GPa, nu 0.23. */
    public static final double GRANITE_MODULUS_PA = 59.3e9, GRANITE_POISSON = 0.23;
    /**
     * Steel on rock: friction 0.4, the residual (sliding) value of polished mild steel on sawn andesite in ring shear,
     * friction angle 21 deg (Ziogos et al. 2023, Proc. ICE Geotech. Eng. 176(1)); granite itself was not measured, the
     * same study's limestone and sandstone give 0.12-0.23 and 0.45-0.55. Restitution: Thornton with V_y for steel on granite.
     */
    public static final ContactMaterial STEEL_ON_ROCK = new ContactMaterial(0.4, 1,
            yieldVelocity(STEEL_YIELD_PA, STEEL_DENSITY, contactModulus(STEEL_MODULUS_PA, STEEL_POISSON, GRANITE_MODULUS_PA, GRANITE_POISSON)));

    public ContactMaterial {
        Checks.nonnegative(friction, "Friction coefficient");
        if (!Double.isFinite(restitution) || restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException("Restitution must be in [0, 1]");
        }
        if (!(yieldVelocityMps > 0)) { throw new IllegalArgumentException("Yield velocity must be positive (infinite for a constant restitution)"); }
    }
    /** Constant restitution at every impact speed. */
    public ContactMaterial(double friction, double restitution) { this(friction, restitution, Double.POSITIVE_INFINITY); }

    /** Restitution at the given approach speed (m/s). */
    public double restitutionAt(double approachSpeed) {
        double v = Math.abs(approachSpeed);
        if (!Double.isFinite(yieldVelocityMps) || v <= yieldVelocityMps) { return restitution; }
        return restitution * thornton(yieldVelocityMps / v);
    }

    /** Thornton's coefficient for r = V_y / V in (0, 1]; 1 at r = 1. */
    public static double thornton(double ratio) {
        double r = Math.clamp(ratio, 0, 1);
        return Math.min(1, 1.442 * Math.sqrt(1 - r * r / 6) * Math.pow(r / (r + 2 * Math.sqrt(6.0 / 5 - r * r / 5)), 0.25));
    }

    /** Effective contact modulus E* = 1 / ((1 - nu1^2)/E1 + (1 - nu2^2)/E2), Pa. */
    public static double contactModulus(double e1, double nu1, double e2, double nu2) {
        return 1 / ((1 - nu1 * nu1) / e1 + (1 - nu2 * nu2) / e2);
    }

    /**
     * Velocity at first yield of a sphere impact (Johnson 1985, Contact Mechanics, sec. 11.4), with the limiting
     * pressure p_y = 1.6 Y: V_y = 1.56 sqrt(p_y^5 / (E*^4 rho)) = 5.1 sqrt(Y^5 / (E*^4 rho)).
     */
    public static double yieldVelocity(double yieldStressPa, double densityKgM3, double contactModulusPa) {
        double py = 1.6 * yieldStressPa;
        return 1.56 * Math.sqrt(Math.pow(py, 5) / (Math.pow(contactModulusPa, 4) * densityKgM3));
    }

    /** Two steel bodies: kinetic friction 0.57 of mild steel on mild steel (NASA RP-1228, Table IV; oxide films vary it 0.15-0.8) and steel-on-steel V_y. */
    public static final ContactMaterial STEEL_ON_STEEL = new ContactMaterial(0.57, 1,
            yieldVelocity(STEEL_YIELD_PA, STEEL_DENSITY, contactModulus(STEEL_MODULUS_PA, STEEL_POISSON, STEEL_MODULUS_PA, STEEL_POISSON)));
}

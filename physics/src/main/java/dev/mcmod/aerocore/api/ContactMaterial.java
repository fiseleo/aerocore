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

    /**
     * Velocity at first yield for a given limiting contact pressure p_y of the softer body (Pa), when it is known as a
     * hardness rather than a yield stress: V_y = 1.56 sqrt(p_y^5 / (E*^4 rho)), rho the steel body's density. With bulk
     * yield strength instead, Thornton's law gives restitutions far below measurements for ice, wood and concrete;
     * indentation hardness reproduces them (for example ice on ice: 0.77 at 1 m/s against 0.68-0.79 measured,
     * Deckers and Teiser 2016).
     */
    public static double yieldVelocityFromPressure(double pressurePa, double densityKgM3, double contactModulusPa) {
        return 1.56 * Math.sqrt(Math.pow(pressurePa, 5) / (Math.pow(contactModulusPa, 4) * densityKgM3));
    }

    /**
     * Steel on wood: friction 0.3, the middle of measured steel-on-dry-wood sliding (spruce 0.16-0.26, Lemoine 1970;
     * balsa about 0.5, Atack and Tabor 1958; reviewed by Zheng, Cabboi and van de Kuilen 2024, Tribol. Int. 200:110193).
     * Restitution: Thornton with oak's side-grain hardness about 35 MPa as p_y (Brinell 30-40 N/mm^2) and its
     * tangential modulus 0.9 GPa (white oak, E_L 12.3 GPa x E_T/E_L 0.072, Wood Handbook FPL-GTR-190 table 5-1),
     * Poisson 0.30 (its nu_TR). No measured steel-on-wood restitution was found to check it.
     */
    public static final ContactMaterial STEEL_ON_WOOD = new ContactMaterial(0.3, 1,
            yieldVelocityFromPressure(35e6, STEEL_DENSITY, contactModulus(STEEL_MODULUS_PA, STEEL_POISSON, 0.9e9, 0.30)));
    /**
     * Steel on ice: friction 0.01, measured near -10 C (Weber et al. 2018, J. Phys. Chem. Lett. 9:2838; it rises
     * steeply in colder ice, toward 0.5 at -100 C; temperature is not modelled here). Restitution: Thornton with ice's
     * indentation hardness 100 MPa at -10 C (Butkovich 1954, via Tusima 1977) and E 9.33 GPa, nu 0.325 (Gammon et al.
     * 1983); ice cracks above about 2-3 m/s, which is not modelled.
     */
    public static final ContactMaterial STEEL_ON_ICE = new ContactMaterial(0.01, 1,
            yieldVelocityFromPressure(100e6, STEEL_DENSITY, contactModulus(STEEL_MODULUS_PA, STEEL_POISSON, 9.33e9, 0.325)));
    /**
     * Steel on loose sand: friction 0.30 (steel sheet piles on clean sand, NAVFAC DM-7.02 table 1); no rebound, a body
     * much larger than the grains embeds (Brisset et al. 2018: no rebound at all above 1e-2 g).
     */
    public static final ContactMaterial STEEL_ON_SAND = new ContactMaterial(0.30, 0);
    /**
     * Steel on gravel: friction 0.40 (steel sheet piles on clean gravel, NAVFAC DM-7.02 table 1); restitution 0.31, the
     * middle of rockfall field values on talus (0.30-0.33, RocFall coefficient table).
     */
    public static final ContactMaterial STEEL_ON_GRAVEL = new ContactMaterial(0.40, 0.31);
    /**
     * Steel on soil: friction 0.30, the middle of steel on reconstituted clay (tan delta 0.28-0.33, Chulalongkorn
     * thesis); restitution 0.30, the middle of rockfall field values on soft soil with vegetation (0.28-0.32, RocFall).
     */
    public static final ContactMaterial STEEL_ON_SOIL = new ContactMaterial(0.30, 0.30);
    /**
     * Steel on mud: friction 0.25, wet soil past its plastic limit lubricating a steel blade (0.2-0.3; Birch, Ekwue and
     * Phillip 2016, West Indian J. Eng. 38(2)); its adhesion (0.2-7.6 kPa) is not modelled; no rebound.
     */
    public static final ContactMaterial STEEL_ON_MUD = new ContactMaterial(0.25, 0);
    /** Steel on snow: friction 0.113, wet dense snow on a steel-floored chute (Aggarwal, Das and Gusain 2024, Open Geosci.); snow compacts: no rebound. */
    public static final ContactMaterial STEEL_ON_SNOW = new ContactMaterial(0.113, 0);
    /**
     * Steel on wool: no measured steel-on-fabric value was found; friction 0.3 is a rule between leather fibre on
     * aluminium (0.30) and leather on cast iron (0.56), and the soft fabric is given no rebound (a rule).
     */
    public static final ContactMaterial STEEL_ON_WOOL = new ContactMaterial(0.3, 0);
    /**
     * Steel on glass: glass stays elastic until it cracks, so Thornton's plastic law does not apply; restitution 0.9
     * (glass impacts at up to about 1.4 m/s, 0.85-0.95), friction 0.5 (glass on metal in handbooks, 0.5-0.7, the low
     * end). Breaking is not modelled.
     */
    public static final ContactMaterial STEEL_ON_GLASS = new ContactMaterial(0.5, 0.9);
}

package dev.mcmod.aerocore.api.earthmoving;

/**
 * Pile driving: how far a pile goes per blow against the ground's resistance, and that resistance.
 *
 * <ul>
 * <li>Hiley (1925; as in Tomlinson, Pile Design and Construction Practice, and BS 8004): R = e_h W h eta / (s + c / 2),
 *     so the set per blow s = e_h W h eta / R - c / 2 (0 at refusal); e_h the hammer's efficiency (about 0.75 to 1:
 *     drop hammers on a winch 0.75, free-fall and hydraulic hammers near 1), W h the ram's potential energy, c the
 *     temporary compression of the cap, the pile (R L / (A E)) and the ground (quake, about 2.5 mm; Smith 1960). The
 *     efficiency of the blow from the ram mass W and pile mass P with restitution n (0.25 to 0.5 for steel on a timber
 *     or plastic dolly): eta = (W + n^2 P) / (W + P) when W is at least n P, less ((W - n P) / (W + P))^2 when it is lighter.</li>
 * <li>ENR (Engineering News, 1888): R = W h / (s + C), C = 25.4 mm for drop hammers, 2.54 mm for single-acting steam
 *     hammers (the allowable load is a sixth of this R).</li>
 * <li>Static resistance of a pile in a material: shaft friction c_a + K sigma_v tan(delta) averaged over the embedded
 *     length, K = 1 - sin(phi) (Jaky; the low end for driven piles), plus end bearing c N_c + sigma_v N_q with
 *     Prandtl-Reissner's N_q = exp(pi tan phi) tan^2(45 deg + phi / 2), N_c = (N_q - 1) cot(phi) (pi + 2 at phi = 0),
 *     shallow-footing factors and so on the low side for deep toes; on rock-like materials end bearing by Zhang and
 *     Einstein (1998), q = 4.83 UCS^0.51 (MPa). Dry ground (total stress; the caller handles water).</li>
 * </ul>
 */
public final class Piling {
    private Piling() {}

    /** ENR's constant for drop hammers, m (1 inch). */
    public static final double ENR_DROP_M = 0.0254;

    /** ENR's constant for single-acting steam hammers, m (0.1 inch). */
    public static final double ENR_STEAM_M = 0.00254;

    /** Typical quake of the ground, m (0.1 inch; Smith 1960). */
    public static final double QUAKE_M = 0.0025;

    /** Hiley's efficiency of the blow of a ram (kg) on a pile (kg) with a restitution (0..1). */
    public static double blowEfficiency(double ramKg, double pileKg, double restitution) {
        if (!(ramKg > 0) || !(pileKg >= 0) || !(restitution >= 0 && restitution <= 1) || !Double.isFinite(ramKg) || !Double.isFinite(pileKg)) {
            throw new IllegalArgumentException("Ram positive, pile not negative, restitution within [0, 1]");
        }
        double total = ramKg + pileKg, eta = (ramKg + restitution * restitution * pileKg) / total;
        if (ramKg < restitution * pileKg) {
            double r = (ramKg - restitution * pileKg) / total;
            eta -= r * r;
        }
        return eta;
    }

    /**
     * Hiley's set per blow (m) against a resistance (N): ram (kg) falling a height (m) at a hammer efficiency (0..1],
     * on a pile (kg) with a restitution, temporary compression (m, cap + pile + quake). 0 at refusal; infinite with no
     * resistance (the pile does not stop).
     */
    public static double hileySetM(double resistanceN, double ramKg, double dropM, double hammerEfficiency, double pileKg, double restitution,
                                   double temporaryCompressionM) {
        double energy = hileyEnergyJ(ramKg, dropM, hammerEfficiency, pileKg, restitution, temporaryCompressionM);
        if (!(resistanceN >= 0) || !Double.isFinite(resistanceN)) { throw new IllegalArgumentException("Resistance finite and not negative"); }
        if (resistanceN == 0) { return energy > 0 ? Double.POSITIVE_INFINITY : 0; }
        return Math.max(0, energy / resistanceN - temporaryCompressionM / 2);
    }

    /** Hiley's resistance (N) at a set per blow (m), the inverse of {@link #hileySetM}. */
    public static double hileyResistanceN(double setM, double ramKg, double dropM, double hammerEfficiency, double pileKg, double restitution,
                                          double temporaryCompressionM) {
        if (!(setM >= 0) || !Double.isFinite(setM)) { throw new IllegalArgumentException("Set finite and not negative"); }
        double energy = hileyEnergyJ(ramKg, dropM, hammerEfficiency, pileKg, restitution, temporaryCompressionM);
        double travel = setM + temporaryCompressionM / 2;
        return travel > 0 ? energy / travel : Double.POSITIVE_INFINITY;
    }

    private static double hileyEnergyJ(double ramKg, double dropM, double hammerEfficiency, double pileKg, double restitution, double compressionM) {
        if (!(dropM >= 0) || !Double.isFinite(dropM) || !(hammerEfficiency > 0 && hammerEfficiency <= 1) || !(compressionM >= 0) || !Double.isFinite(compressionM)) {
            throw new IllegalArgumentException("Drop and compression finite and not negative, efficiency within (0, 1]");
        }
        return hammerEfficiency * ramKg * Material.G * dropM * blowEfficiency(ramKg, pileKg, restitution);
    }

    /** ENR's set per blow (m) against a resistance (N) for a ram (kg) falling a height (m), constant C (m): W h / R - C, 0 at refusal, infinite with no resistance. */
    public static double enrSetM(double resistanceN, double ramKg, double dropM, double constantM) {
        if (!(resistanceN >= 0) || !(ramKg > 0) || !(dropM >= 0) || !(constantM >= 0) || !Double.isFinite(resistanceN) || !Double.isFinite(ramKg)
                || !Double.isFinite(dropM) || !Double.isFinite(constantM)) {
            throw new IllegalArgumentException("Resistance, drop and constant finite and not negative, ram positive");
        }
        double energy = ramKg * Material.G * dropM;
        if (resistanceN == 0) { return energy > 0 ? Double.POSITIVE_INFINITY : 0; }
        return Math.max(0, energy / resistanceN - constantM);
    }

    /** Elastic shortening (m) of a pile of a length (m), section (m^2) and modulus (Pa) under a resistance (N): R L / (A E). */
    public static double elasticCompressionM(double resistanceN, double lengthM, double areaM2, double modulusPa) {
        if (!(resistanceN >= 0) || !(lengthM >= 0) || !(areaM2 > 0) || !(modulusPa > 0)) { throw new IllegalArgumentException("Resistance and length not negative, area and modulus positive"); }
        return resistanceN * lengthM / (areaM2 * modulusPa);
    }

    /** Prandtl-Reissner bearing factor N_q at a friction angle (rad): exp(pi tan phi) tan^2(45 deg + phi / 2). */
    public static double bearingNq(double frictionRad) {
        if (!(frictionRad >= 0 && frictionRad < Math.PI / 2)) { throw new IllegalArgumentException("Friction angle within [0, 90) deg"); }
        double t = Math.tan(Math.PI / 4 + frictionRad / 2);
        return Math.exp(Math.PI * Math.tan(frictionRad)) * t * t;
    }

    /**
     * Prandtl's bearing factor N_c at a friction angle (rad): (N_q - 1) cot(phi); below 1e-4 rad its series
     * pi + 2 + (pi^2 / 2 + 2 pi + 2) phi, which keeps it continuous into phi = 0.
     */
    public static double bearingNc(double frictionRad) {
        if (!(frictionRad >= 0 && frictionRad < Math.PI / 2)) { throw new IllegalArgumentException("Friction angle within [0, 90) deg"); }
        if (frictionRad < 1e-4) { return Math.PI + 2 + (Math.PI * Math.PI / 2 + 2 * Math.PI + 2) * frictionRad; }
        return (bearingNq(frictionRad) - 1) / Math.tan(frictionRad);
    }

    /**
     * Static resistance (N) of a pile of a shaft perimeter (m) and toe area (m^2) embedded a length (m) in a material
     * (class note): shaft friction over the embedded length plus end bearing at the toe.
     */
    public static double staticResistanceN(Material m, double perimeterM, double toeAreaM2, double embedmentM) {
        if (!(perimeterM >= 0) || !(toeAreaM2 >= 0) || !(embedmentM >= 0) || !Double.isFinite(perimeterM) || !Double.isFinite(toeAreaM2)
                || !Double.isFinite(embedmentM)) {
            throw new IllegalArgumentException("Perimeter, toe area and embedment finite and not negative");
        }
        double gamma = m.unitWeightNPerM3(), phi = m.frictionRad();
        double k = 1 - Math.sin(phi);
        double shaft = perimeterM * embedmentM * (m.adhesionPa() + k * gamma * embedmentM / 2 * Math.tan(Math.min(m.wallFrictionRad(), phi)));
        double toe;
        if (m.isRock()) {
            toe = 4.83e6 * Math.pow(m.compressivePa() / 1e6, 0.51);
        } else {
            toe = m.cohesionPa() * bearingNc(phi) + gamma * embedmentM * bearingNq(phi);
        }
        return shaft + toeAreaM2 * toe;
    }
}

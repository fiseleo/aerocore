package dev.mcmod.aerocore.api.damage;

import dev.mcmod.aerocore.api.Vec3d;

import java.util.List;

/**
 * 0.42: what a hit does to the parts of a body. Formulas only: the energy a hit leaves in the structure, and how it is
 * shared between parts and used against what each can absorb. Which parts there are, how tough each is and what a used-up
 * part does are the caller's rules.
 *
 * <p>Energy: a body of mass m stopping dead on something rigid loses m v^2 / 2 with a normal impulse J = m v, so a
 * perfectly inelastic hit of impulse J closing at v leaves J v / 2 in the structure whatever the mass behind it. Below
 * the elastic speed the structure springs back; a hit at v keeps the share 1 - (v0 / v)^2 of that energy (the kinetic
 * energy above the elastic speed's).
 *
 * <p>Toughness is the energy per kg of a part that uses it up, for the whole part as built, not the material's specific
 * energy absorption in an ideal crush (mild steel tubes folding axially take 15 to 30 kJ/kg, aluminium extrusions 20 to
 * 40, carbon-fibre tubes 50 to 100: Lu and Yu, Energy Absorption of Structures and Materials, 2003): only the structure
 * in the load path folds, and far from ideally. A whole-car figure: a 1400 kg car in the 64 km/h offset barrier test
 * carries 221 kJ and uses up roughly the front quarter of a 330 kg body shell, about 2.7 kJ per kg of shell used
 * (SHELL_STEEL_J_PER_KG; an order of magnitude from those round numbers, not a measurement).
 */
public final class ImpactDamage {
    private ImpactDamage() {}

    /**
     * Closing speed below which a hit only springs back, m/s: car bumpers are built to take 4 km/h without damage
     * (UN Regulation 42).
     */
    public static final double ELASTIC_MPS = 4 / 3.6;

    /** Energy per kg that uses up a steel car body shell as built, J/kg (see the class comment; order of magnitude). */
    public static final double SHELL_STEEL_J_PER_KG = 2700;

    /** The energy (J) a hit of normal impulse (N s) closing at a speed (m/s) leaves in the structure, with ELASTIC_MPS. */
    public static double energyJ(double impulseNs, double approachMps) { return energyJ(impulseNs, approachMps, ELASTIC_MPS); }

    /** The same with the caller's elastic speed (m/s, not negative). */
    public static double energyJ(double impulseNs, double approachMps, double elasticMps) {
        if (!(impulseNs >= 0) || !(approachMps >= 0) || !(elasticMps >= 0) || !Double.isFinite(impulseNs) || !Double.isFinite(approachMps)) {
            throw new IllegalArgumentException("Impulse and speeds must be finite and not negative");
        }
        if (approachMps <= elasticMps || approachMps == 0) { return 0; }
        double spring = elasticMps / approachMps;
        return 0.5 * impulseNs * approachMps * (1 - spring * spring);
    }

    /**
     * One part as a hit sees it: its centre (in the frame the hit's point is given in), its mass (kg), the energy per kg
     * that uses it up (J/kg) and its reach (m): about half its size, the distance from its centre at which its share has
     * halved.
     */
    public record Target(Vec3d centre, double massKg, double toughnessJPerKg, double reachM) {
        public Target {
            java.util.Objects.requireNonNull(centre, "centre");
            if (!(massKg > 0) || !(toughnessJPerKg > 0) || !(reachM > 0) || !Double.isFinite(massKg) || !Double.isFinite(reachM)) {
                throw new IllegalArgumentException("A target has positive mass, toughness and reach");
            }
        }
    }

    /**
     * Each target's share of a hit's energy at a point: in proportion to m / (1 + (d / reach)^2), d the distance from
     * the point to its centre. The mass counts because the heavy structure near a hit is what stops the rest; the falloff
     * is a rule (a part twice its reach away takes a fifth of what it would at the point). The shares sum to 1 (all zero
     * without targets).
     */
    public static double[] shares(Vec3d point, List<Target> targets) {
        double[] w = new double[targets.size()];
        double sum = 0;
        for (int i = 0; i < w.length; i++) {
            Target t = targets.get(i);
            double d = t.centre().subtract(point).length() / t.reachM();
            w[i] = t.massKg() / (1 + d * d);
            sum += w[i];
        }
        if (sum > 0) { for (int i = 0; i < w.length; i++) { w[i] /= sum; } }
        return w;
    }

    /**
     * How much of each target a hit's energy uses up (a share of the whole part; may pass 1): its share of the energy
     * over what the part absorbs, toughness x mass.
     */
    public static double[] damage(double energyJ, Vec3d point, List<Target> targets) {
        if (!(energyJ >= 0) || !Double.isFinite(energyJ)) { throw new IllegalArgumentException("Energy must be finite and not negative"); }
        double[] out = shares(point, targets);
        for (int i = 0; i < out.length; i++) {
            Target t = targets.get(i);
            out[i] = energyJ * out[i] / (t.toughnessJPerKg() * t.massKg());
        }
        return out;
    }

    /**
     * A blast as a hit: the energy a body takes from an explosion's overpressure impulse on its presented area. With the
     * side-on impulse per area i (Pa s) on area A (m^2) the body of mass m (kg) takes the impulse i A and, starting at
     * rest, the kinetic energy (i A)^2 / (2 m); what stops it again (the ground, its own structure) dissipates that.
     */
    public static double blastEnergyJ(double impulsePaS, double areaM2, double massKg) {
        if (!(impulsePaS >= 0) || !(areaM2 >= 0) || !(massKg > 0)) { throw new IllegalArgumentException("Impulse and area not negative, mass positive"); }
        double j = impulsePaS * areaM2;
        return j * j / (2 * massKg);
    }
}

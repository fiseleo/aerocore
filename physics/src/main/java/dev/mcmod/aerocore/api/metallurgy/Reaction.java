package dev.mcmod.aerocore.api.metallurgy;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A balanced reaction (0.40): moles of each reactant and product as written, its enthalpy from the species' standard
 * enthalpies of formation (Hess's law, at 298.15 K: the heat to bring the charge to temperature and the products' heat
 * at it are the caller's), and the masses that go in and come out.
 *
 * <ul>
 * <li>Mass balance is checked when the reaction is made (reactants and products within 0.1 %: the molar masses are
 *     rounded); the atoms are the caller's to balance.</li>
 * <li>A thermite-type reaction sustains itself when it releases more than about 2.3 kJ a gram of its charge
 *     (Shidlovskii's criterion, the one aluminothermic practice uses: Belitskus, J. Met. 24 (1972) 30; reactions near
 *     it need preheating or a booster).</li>
 * </ul>
 */
public record Reaction(String name, Map<Species, Double> reactants, Map<Species, Double> products) {
    /** Least heat a gram of charge must release for an aluminothermic reaction to run by itself, J/kg. */
    public static final double SELF_SUSTAINING_J_PER_KG = 2.3e6;

    public Reaction {
        reactants = Collections.unmodifiableMap(new LinkedHashMap<>(reactants));
        products = Collections.unmodifiableMap(new LinkedHashMap<>(products));
        double in = mass(reactants), out = mass(products);
        if (!(in > 0) || Math.abs(in - out) > 1e-3 * in) { throw new IllegalArgumentException(name + ": reactants " + in + " kg/mol, products " + out); }
    }

    /** A reaction from alternating species and moles: reactants, then the marker null, then products. */
    public static Reaction of(String name, Object... terms) {
        Map<Species, Double> r = new LinkedHashMap<>(), p = new LinkedHashMap<>();
        Map<Species, Double> side = r;
        for (int i = 0; i < terms.length; i++) {
            if (terms[i] == null) { side = p; continue; }
            side.merge((Species) terms[i], ((Number) terms[++i]).doubleValue(), Double::sum);
        }
        return new Reaction(name, r, p);
    }

    private static double mass(Map<Species, Double> side) {
        double m = 0;
        for (Map.Entry<Species, Double> e : side.entrySet()) { m += e.getKey().molarMassKg * e.getValue(); }
        return m;
    }

    /** Enthalpy of the reaction as written at 298.15 K, J (positive: it takes heat). */
    public double enthalpyJ() {
        double h = 0;
        for (Map.Entry<Species, Double> e : products.entrySet()) { h += e.getKey().formationJPerMol * e.getValue(); }
        for (Map.Entry<Species, Double> e : reactants.entrySet()) { h -= e.getKey().formationJPerMol * e.getValue(); }
        return h;
    }

    /** Mass of the reactants (the charge) as written, kg. */
    public double chargeKg() { return mass(reactants); }

    /** Moles of a species on either side (0 when absent). */
    public double moles(Species s) { return reactants.getOrDefault(s, products.getOrDefault(s, 0.0)); }

    /** Kilograms of one species (either side) a kilogram of another (either side). */
    public double kgPerKg(Species what, Species per) {
        double n = moles(per);
        if (!(n > 0)) { throw new IllegalArgumentException(per + " is not in " + name); }
        return moles(what) * what.molarMassKg / (n * per.molarMassKg);
    }

    /** Enthalpy a kilogram of a species (either side), J/kg. */
    public double enthalpyJPerKg(Species per) {
        double n = moles(per);
        if (!(n > 0)) { throw new IllegalArgumentException(per + " is not in " + name); }
        return enthalpyJ() / (n * per.molarMassKg);
    }

    /** Heat released a kilogram of charge, J/kg (negative when the reaction takes heat). */
    public double releasedJPerKgCharge() { return -enthalpyJ() / chargeKg(); }

    /** Whether an aluminothermic (thermite-type) reaction runs by itself once lit. */
    public boolean selfSustaining() { return releasedJPerKgCharge() > SELF_SUSTAINING_J_PER_KG; }
}

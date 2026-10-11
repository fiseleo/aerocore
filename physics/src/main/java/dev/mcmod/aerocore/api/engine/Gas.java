package dev.mcmod.aerocore.api.engine;

import java.util.EnumMap;
import java.util.Map;

/**
 * A fixed ideal-gas mixture (moles of each Species per kilogram of mixture) with its sensible internal energy tabulated
 * every kelvin from 200 to 4000 K, so a cycle simulation can go from energy to temperature cheaply. Sensible means
 * relative to 298.15 K: u_s(T) = sum n_i (h_i(T) - h_i(298.15)) - R (T - 298.15). Immutable.
 */
public final class Gas {
    private static final int T_MIN = 200, T_MAX = 4000;
    private static final double T_REF = 298.15;

    /** Dry air: O2, N2 and Ar (Species). */
    public static final Gas AIR = of(Map.of(Species.O2, 1.0, Species.N2, Species.AIR_N2_PER_O2, Species.AR, Species.AIR_AR_PER_O2));

    private final Map<Species, Double> molesPerKg;
    private final double gasConstant;
    private final double[] energy;
    /** Entropy function phi(T) = integral of cp / T from 200 K, J/(kg K), exact for the table's piecewise-constant cp. */
    private final double[] entropy;

    private Gas(Map<Species, Double> molesPerKg) {
        this.molesPerKg = Map.copyOf(molesPerKg);
        double moles = 0;
        for (double n : molesPerKg.values()) { moles += n; }
        gasConstant = Species.R_UNIVERSAL * moles;
        energy = new double[T_MAX - T_MIN + 1];
        double[] reference = new double[Species.values().length];
        for (Species s : molesPerKg.keySet()) { reference[s.ordinal()] = s.enthalpy(T_REF); }
        for (int i = 0; i < energy.length; i++) {
            double t = T_MIN + i, h = 0;
            for (Map.Entry<Species, Double> e : molesPerKg.entrySet()) { h += e.getValue() * (e.getKey().enthalpy(t) - reference[e.getKey().ordinal()]); }
            energy[i] = h - gasConstant * (t - T_REF);
        }
        entropy = new double[energy.length];
        for (int i = 1; i < entropy.length; i++) {
            double cp = energy[i] - energy[i - 1] + gasConstant;
            entropy[i] = entropy[i - 1] + cp * Math.log((double) (T_MIN + i) / (T_MIN + i - 1));
        }
    }

    /** A mixture from relative mole amounts (any scale); they are normalised to one kilogram. */
    public static Gas of(Map<Species, Double> relativeMoles) {
        double mass = 0;
        for (Map.Entry<Species, Double> e : relativeMoles.entrySet()) {
            if (!(e.getValue() >= 0) || !Double.isFinite(e.getValue())) { throw new IllegalArgumentException("Mole amounts must be finite and nonnegative"); }
            mass += e.getValue() * e.getKey().molarMass();
        }
        if (!(mass > 0)) { throw new IllegalArgumentException("Mixture has no mass"); }
        Map<Species, Double> perKg = new EnumMap<>(Species.class);
        for (Map.Entry<Species, Double> e : relativeMoles.entrySet()) { if (e.getValue() > 0) { perKg.put(e.getKey(), e.getValue() / mass); } }
        return new Gas(perKg);
    }

    /** Moles of each species per kilogram. */
    public Map<Species, Double> molesPerKg() { return molesPerKg; }
    /** Specific gas constant, J/(kg K). */
    public double gasConstant() { return gasConstant; }

    /** Sensible internal energy per kilogram relative to 298.15 K, J/kg (linear between whole kelvins; held slope outside 200..4000 K). */
    public double energy(double temperatureK) {
        double x = temperatureK - T_MIN;
        if (x <= 0) { return energy[0] + (energy[1] - energy[0]) * x; }
        int i = (int) x;
        if (i >= energy.length - 1) { return energy[energy.length - 1] + (energy[energy.length - 1] - energy[energy.length - 2]) * (x - energy.length + 1); }
        return energy[i] + (energy[i + 1] - energy[i]) * (x - i);
    }

    /** Sensible enthalpy per kilogram relative to 298.15 K, J/kg: sum n_i (h_i(T) - h_i(298.15)). */
    public double enthalpy(double temperatureK) { return energy(temperatureK) + gasConstant * (temperatureK - T_REF); }

    /**
     * Entropy function phi(T) = integral of cp / T dT (from 200 K), J/(kg K): an isentropic change from (T1, p1) to
     * (T2, p2) has phi(T2) - phi(T1) = R ln(p2 / p1). Linear in ln T between whole kelvins (constant cp per kelvin).
     */
    public double entropyFunction(double temperatureK) {
        if (temperatureK <= T_MIN + 1) { return (energy[1] - energy[0] + gasConstant) * Math.log(temperatureK / T_MIN); }
        double x = temperatureK - T_MIN;
        int i = (int) x;
        if (i >= entropy.length - 1) {
            int last = entropy.length - 1;
            return entropy[last] + (energy[last] - energy[last - 1] + gasConstant) * Math.log(temperatureK / (T_MIN + last));
        }
        return entropy[i] + (energy[i + 1] - energy[i] + gasConstant) * Math.log(temperatureK / (T_MIN + i));
    }

    /** The temperature whose entropy function is phi (inverse of entropyFunction), K. */
    public double temperatureAtEntropy(double phi) {
        if (phi <= entropy[1]) { return T_MIN * Math.exp(phi / (energy[1] - energy[0] + gasConstant)); }
        int last = entropy.length - 1;
        if (phi >= entropy[last]) { return (T_MIN + last) * Math.exp((phi - entropy[last]) / (energy[last] - energy[last - 1] + gasConstant)); }
        int lo = 0, hi = last;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (entropy[mid] <= phi) { lo = mid; } else { hi = mid; }
        }
        return (T_MIN + lo) * Math.exp((phi - entropy[lo]) / (energy[lo + 1] - energy[lo] + gasConstant));
    }

    /** The temperature with sensible enthalpy h (inverse of enthalpy), K. */
    public double temperatureAtEnthalpy(double enthalpyJPerKg) {
        double t = 1000;
        for (int i = 0; i < 40; i++) {
            double step = (enthalpy(t) - enthalpyJPerKg) / cp(t);
            t -= step;
            if (Math.abs(step) < 1e-7) { break; }
        }
        return t;
    }

    /** Heat capacity at constant volume, J/(kg K), as the slope of the table at T. */
    public double cv(double temperatureK) {
        int i = (int) Math.clamp(temperatureK - T_MIN, 0, energy.length - 2);
        return energy[i + 1] - energy[i];
    }

    /** Heat capacity at constant pressure, J/(kg K). */
    public double cp(double temperatureK) { return cv(temperatureK) + gasConstant; }

    /** The temperature whose energy is e (inverse of energy), K. */
    public double temperature(double energyJPerKg) {
        if (energyJPerKg <= energy[0]) { return T_MIN + (energyJPerKg - energy[0]) / (energy[1] - energy[0]); }
        int last = energy.length - 1;
        if (energyJPerKg >= energy[last]) { return T_MIN + last + (energyJPerKg - energy[last]) / (energy[last] - energy[last - 1]); }
        int lo = 0, hi = last;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (energy[mid] <= energyJPerKg) { lo = mid; } else { hi = mid; }
        }
        return T_MIN + lo + (energyJPerKg - energy[lo]) / (energy[hi] - energy[lo]);
    }

    /**
     * Energy and gas constant of a mass blend of two gases (fraction of b by mass), as used by a single-zone cylinder
     * whose charge is part fresh mixture, part burned gas.
     */
    public static double blendEnergy(Gas a, Gas b, double fractionB, double temperatureK) {
        return (1 - fractionB) * a.energy(temperatureK) + fractionB * b.energy(temperatureK);
    }
    /** Temperature of a blend with the given sensible energy, by Newton iteration from a guess. */
    public static double blendTemperature(Gas a, Gas b, double fractionB, double energyJPerKg, double guessK) {
        double t = guessK;
        for (int i = 0; i < 30; i++) {
            double f = blendEnergy(a, b, fractionB, t) - energyJPerKg;
            double slope = (1 - fractionB) * a.cv(t) + fractionB * b.cv(t);
            double step = f / slope;
            t -= step;
            if (Math.abs(step) < 1e-6) { break; }
        }
        return t;
    }
}

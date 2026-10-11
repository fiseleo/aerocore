package dev.mcmod.aerocore.api.engine;

import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * A spark-ignition fuel: atoms per "molecule" (C_alpha H_beta O_gamma), lower heating value, heat of vaporization
 * (zero for a fuel that reaches the engine as a gas), research and motor octane numbers, liquid density (for injector
 * flow; propane as the liquid under tank pressure) and its laminar burning velocity correlation.
 * <p>
 * Property values of the presets are those tabulated by Heywood, Internal Combustion Engine Fundamentals (1988),
 * appendix D; gasoline is taken as CH1.87 (the hydrogen-to-carbon ratio Heywood uses for gasoline). Laminar burning
 * velocity S_L = S_L0(phi) (T/298 K)^alpha (p/1 atm)^beta (1 - 2.06 x_b^0.77): Metghalchi and Keck, Combust. Flame 48
 * (1982) 191, for methanol, propane and isooctane (S_L0 = B_m + B_phi (phi - phi_m)^2, alpha = 2.98 - 0.8 (phi - 1),
 * beta = -0.38 + 0.22 (phi - 1)); Rhodes and Keck, SAE 850047 (1985), for gasoline (alpha = 2.4 - 0.271 phi^3.51,
 * beta = -0.357 + 0.14 phi^2.77); constants as listed in the ANSYS Forte theory manual, table 7.1. The dilution factor
 * (1 - 2.06 x_b^0.77) is Rhodes and Keck's. S_L0 is not allowed below 5 % of B_m (the quadratic goes negative far
 * from stoichiometric; such mixtures barely burn in an engine anyway).
 */
public record Fuel(String name, double carbon, double hydrogen, double oxygen, double lowerHeatingValueJPerKg,
                   double vaporizationHeatJPerKg, double researchOctane, double motorOctane, double liquidDensityKgPerM3, FlameSpeed flameSpeed) {
    /** Water-gas shift equilibrium constant used for rich products (Heywood 1988, section 4.9.2). */
    public static final double WATER_GAS_K = 3.5;

    /** Laminar burning velocity correlation; S_L0 coefficients in m/s. */
    public record FlameSpeed(double baseMps, double curvatureMps, double peakEquivalenceRatio, boolean rhodesKeckExponents) {
        /** Laminar burning velocity at equivalence ratio phi, unburned temperature T (K), pressure p (Pa) and burned-gas dilution x (mass fraction), m/s. */
        public double at(double phi, double temperatureK, double pressurePa, double dilution) {
            double s0 = Math.max(0.05 * baseMps, baseMps + curvatureMps * (phi - peakEquivalenceRatio) * (phi - peakEquivalenceRatio));
            double alpha = rhodesKeckExponents ? 2.4 - 0.271 * Math.pow(phi, 3.51) : 2.98 - 0.8 * (phi - 1);
            double beta = rhodesKeckExponents ? -0.357 + 0.14 * Math.pow(phi, 2.77) : -0.38 + 0.22 * (phi - 1);
            double diluted = Math.max(0.02, 1 - 2.06 * Math.pow(Math.clamp(dilution, 0, 1), 0.77));
            return s0 * Math.pow(temperatureK / 298, alpha) * Math.pow(pressurePa / 101325, beta) * diluted;
        }
    }

    public static final FlameSpeed GASOLINE_FLAME = new FlameSpeed(0.305, -0.549, 1.21, true);

    public Fuel {
        Objects.requireNonNull(name, "name");
        Objects.requireNonNull(flameSpeed, "flameSpeed");
        if (!(carbon > 0) || !(hydrogen >= 0) || !(oxygen >= 0)) { throw new IllegalArgumentException("Fuel needs carbon and nonnegative H, O"); }
        if (!(lowerHeatingValueJPerKg > 0) || !(vaporizationHeatJPerKg >= 0)) { throw new IllegalArgumentException("Heating values must be positive"); }
        if (!(researchOctane > 0) || !(motorOctane > 0)) { throw new IllegalArgumentException("Octane numbers must be positive"); }
        if (!(liquidDensityKgPerM3 > 0)) { throw new IllegalArgumentException("Liquid density must be positive"); }
    }

    /** Gasoline CH1.87 of the given octane numbers: LHV 44.0 MJ/kg, heat of vaporization 305 kJ/kg (Heywood, appendix D). */
    public static Fuel gasoline(double researchOctane, double motorOctane) {
        return new Fuel("gasoline", 1, 1.87, 0, 44.0e6, 305e3, researchOctane, motorOctane, 740, GASOLINE_FLAME);
    }
    /** Methanol CH3OH: LHV 20.0 MJ/kg, 1103 kJ/kg, RON 106, MON 92. */
    public static final Fuel METHANOL = new Fuel("methanol", 1, 4, 1, 20.0e6, 1103e3, 106, 92, 792, new FlameSpeed(0.3692, -1.4051, 1.11, false));
    /** Propane C3H8 (LPG) injected as a gas: LHV 46.4 MJ/kg, RON 112, MON 97. */
    public static final Fuel PROPANE = new Fuel("propane", 3, 8, 0, 46.4e6, 0, 112, 97, 500, new FlameSpeed(0.3422, -1.3865, 1.08, false));
    /**
     * Kerosene jet fuel (Jet A) as C12H23: LHV 43.2 MJ/kg (typical; ASTM D1655 requires at least 42.8), density
     * 800 kg/m^3, heat of vaporization about 360 kJ/kg; its octane numbers (about 15) only matter if it is put in a
     * spark-ignition engine, where it knocks.
     */
    public static final Fuel KEROSENE = new Fuel("kerosene", 12, 23, 0, 43.2e6, 360e3, 15, 15, 800, GASOLINE_FLAME);
    /** Isooctane C8H18, the 100-octane reference fuel: LHV 44.3 MJ/kg, 308 kJ/kg. */
    public static final Fuel ISOOCTANE = new Fuel("isooctane", 8, 18, 0, 44.3e6, 308e3, 100, 100, 692, new FlameSpeed(0.2632, -0.8472, 1.13, false));

    /** Mass of one "molecule" C_alpha H_beta O_gamma, kg/mol. */
    public double molarMass() { return carbon * 0.012011 + hydrogen * 0.0010079 + oxygen * 0.0159994; }
    /** Moles of O2 per mole of fuel for complete combustion. */
    public double stoichiometricOxygen() { return carbon + hydrogen / 4 - oxygen / 2; }
    /** Stoichiometric air-fuel mass ratio. */
    public double stoichiometricAirFuel() {
        return stoichiometricOxygen() * Species.airMassPerMoleO2() / molarMass();
    }
    /** Anti-knock index (RON + MON) / 2. */
    public double antiKnockIndex() { return (researchOctane + motorOctane) / 2; }

    /** Products of burning one mole of fuel in air at relative air-fuel ratio lambda (moles of each species). */
    public Map<Species, Double> productMoles(double lambda) {
        if (!(lambda > 0) || !Double.isFinite(lambda)) { throw new IllegalArgumentException("Lambda must be positive"); }
        double as = stoichiometricOxygen(), o2 = as * lambda;
        Map<Species, Double> p = new EnumMap<>(Species.class);
        p.put(Species.N2, Species.AIR_N2_PER_O2 * o2); p.put(Species.AR, Species.AIR_AR_PER_O2 * o2);
        if (lambda >= 1) {
            p.put(Species.CO2, carbon); p.put(Species.H2O, hydrogen / 2); p.put(Species.O2, as * (lambda - 1));
            return p;
        }
        // Rich: C, H and O balances with the water-gas shift CO + H2O = CO2 + H2 at K = 3.5 give a quadratic in CO2.
        double oxygenLeft = 2 * o2 + oxygen - carbon;   // = CO2 + H2O (moles), from the O balance with CO = C - CO2
        double k = WATER_GAS_K, a = 1 - k, b = -carbon - oxygenLeft - k * hydrogen / 2 + k * oxygenLeft, c = carbon * oxygenLeft;
        double co2;
        if (Math.abs(a) < 1e-12) { co2 = -c / b; }
        else {
            double root = Math.sqrt(Math.max(0, b * b - 4 * a * c));
            double r1 = (-b + root) / (2 * a), r2 = (-b - root) / (2 * a);
            co2 = r1 >= 0 && r1 <= carbon ? r1 : r2;
        }
        co2 = Math.clamp(co2, 0, carbon);
        double h2o = Math.clamp(oxygenLeft - co2, 0, hydrogen / 2);
        p.put(Species.CO2, co2); p.put(Species.CO, carbon - co2); p.put(Species.H2O, h2o); p.put(Species.H2, hydrogen / 2 - h2o);
        return p;
    }

    /** The burned gas at lambda as a mixture. */
    public Gas products(double lambda) { return Gas.of(productMoles(lambda)); }

    /**
     * Chemical energy released per kilogram of fuel at lambda, J/kg: the lower heating value less the heating value of
     * the CO and H2 a rich mixture leaves unburned (their heats of combustion at 298.15 K from the species
     * polynomials). The few percent of fuel that escapes burning in crevices is the caller's combustion efficiency.
     */
    public double heatRelease(double lambda) {
        if (lambda >= 1) { return lowerHeatingValueJPerKg; }
        Map<Species, Double> p = productMoles(lambda);
        double t = 298.15;
        double co = Species.CO.enthalpy(t) + 0.5 * Species.O2.enthalpy(t) - Species.CO2.enthalpy(t);
        double h2 = Species.H2.enthalpy(t) + 0.5 * Species.O2.enthalpy(t) - Species.H2O.enthalpy(t);
        double unreleased = p.getOrDefault(Species.CO, 0.0) * co + p.getOrDefault(Species.H2, 0.0) * h2;
        return lowerHeatingValueJPerKg - unreleased / molarMass();
    }
}

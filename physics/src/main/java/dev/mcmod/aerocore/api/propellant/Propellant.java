package dev.mcmod.aerocore.api.propellant;

import java.util.List;
import java.util.Objects;

/**
 * A solid rocket propellant: density, burn rate (Saint Robert's law r = a p^n in pieces over pressure), and the
 * thermochemistry of its products in the chamber (flame temperature, effective molar mass and ratio of specific heats,
 * the condensed phase counted in, as amateur and textbook motor design does). From these the characteristic velocity
 * c* = sqrt(gamma R T) / (gamma sqrt((2 / (gamma + 1))^((gamma + 1)/(gamma - 1)))) (Sutton and Biblarz, Rocket
 * Propulsion Elements, 9th ed., eq. 3-32).
 * <p>
 * Presets:
 * <ul>
 * <li>KNSB, potassium nitrate and sorbitol 65/35 cast ("sugar rocket"): density 1841 kg/m^3 (ideal; cast grains reach
 * about 95 %), 1600 K, 39.9 g/mol, k 1.1361 (gas-and-condensed mixture), giving c* about 909 m/s; burn rate in five
 * pressure ranges as measured by Nakka ("KNDX and KNSB propellants - burn rate experimentation", nakka-rocketry.net,
 * 2.59 mm/s at 1 atm and 11.25 mm/s at 1000 psia), with its plateau ("mesa") between 0.8 and 3.8 MPa.</li>
 * <li>APCP, ammonium perchlorate, aluminium and HTPB binder 70/16/14 (composite, as in the Space Shuttle's boosters):
 * density 1770 kg/m^3, 3500 K, 29.4 g/mol, k 1.17 (alumina particles counted in), giving c* about 1548 m/s, so that
 * the Shuttle booster's nozzle (expansion 7.72) delivers within a few percent of its 242 s at sea level and 268 s in
 * vacuum; r = 5.61 mm/s (p / 1 MPa)^0.35, the booster's TP-H1148 (0.368 in/s at 625 psia, n 0.35; Sutton and Biblarz,
 * ch. 13 and table 13-1).</li>
 * </ul>
 */
public record Propellant(String name, double densityKgPerM3, List<BurnRate> burnRates, double flameTemperatureK, double molarMassKgPerMol,
                         double gamma) {

    /** Saint Robert's law r = a (p / 1 MPa)^n, in mm/s, from a pressure up (the last range holds above its start). */
    public record BurnRate(double fromPa, double aMmPerS, double exponent) {}

    public static final double R_UNIVERSAL = 8.314462618;

    public Propellant {
        Objects.requireNonNull(name, "name");
        if (!(densityKgPerM3 > 0) || !(flameTemperatureK > 0) || !(molarMassKgPerMol > 0) || !(gamma > 1)) {
            throw new IllegalArgumentException("A propellant needs positive density, temperature and molar mass, and gamma > 1");
        }
        if (burnRates == null || burnRates.isEmpty()) { throw new IllegalArgumentException("A propellant needs a burn rate"); }
        burnRates = List.copyOf(burnRates);
        for (int i = 0; i < burnRates.size(); i++) {
            BurnRate b = burnRates.get(i);
            if (!(b.aMmPerS() > 0) || !Double.isFinite(b.exponent()) || !(b.exponent() < 1)) { throw new IllegalArgumentException("Burn rate a > 0 and n < 1"); }
            if (i > 0 && !(b.fromPa() > burnRates.get(i - 1).fromPa())) { throw new IllegalArgumentException("Burn-rate ranges must ascend"); }
        }
    }

    public static final Propellant KNSB = new Propellant("knsb", 1841, List.of(
            new BurnRate(0, 10.708, 0.625), new BurnRate(0.807e6, 8.763, -0.314), new BurnRate(1.503e6, 7.852, -0.013),
            new BurnRate(3.792e6, 3.907, 0.535), new BurnRate(7.033e6, 9.653, 0.064)), 1600, 0.0399, 1.1361);
    public static final Propellant APCP = new Propellant("apcp", 1770, List.of(new BurnRate(0, 5.61, 0.35)), 3500, 0.0294, 1.17);

    public static Propellant byName(String name) {
        return switch (name) {
            case "knsb" -> KNSB;
            case "apcp" -> APCP;
            default -> null;
        };
    }

    /** Specific gas constant of the products, J/(kg K). */
    public double gasConstant() { return R_UNIVERSAL / molarMassKgPerMol; }

    /** Burn rate at chamber pressure p, m/s. */
    public double burnRate(double pressurePa) {
        double p = Math.max(1, pressurePa);
        BurnRate b = burnRates.get(0);
        for (BurnRate x : burnRates) { if (p >= x.fromPa()) { b = x; } }
        return b.aMmPerS() * 1e-3 * Math.pow(p / 1e6, b.exponent());
    }

    /** Theoretical characteristic velocity, m/s. */
    public double characteristicVelocity() {
        double g = gamma;
        return Math.sqrt(g * gasConstant() * flameTemperatureK) / (g * Math.sqrt(Math.pow(2 / (g + 1), (g + 1) / (g - 1))));
    }
}

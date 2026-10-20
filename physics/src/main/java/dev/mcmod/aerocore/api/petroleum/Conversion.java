package dev.mcmod.aerocore.api.petroleum;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Refinery and petrochemical conversion units (0.26) as typical product yields (mass fractions of the feed, adding up
 * to 1) and the energy each takes per kg of feed.
 * <ul>
 * <li>Fluid catalytic cracking of vacuum gas oil: dry gas 3.5 %, LPG 17 %, gasoline (RON about 92) 50 %, light cycle oil
 *     (a diesel blending stock) 17 %, slurry oil 7.5 %, coke 5 % (burned off the catalyst: its heat runs the unit)
 *     (Sadeghbeigi, Fluid Catalytic Cracking Handbook, 3rd ed., ch. 1).</li>
 * <li>Catalytic reforming of heavy naphtha: reformate (RON about 98) 85 %, LPG 8 %, fuel gas 4.5 %, hydrogen 2.5 %;
 *     about 1.8 MJ/kg feed (Gary, Handwerk and Kaiser, ch. 10).</li>
 * <li>Steam cracking of naphtha: hydrogen 1 %, methane 15 %, ethylene 30 %, propylene 16 %, butadiene 4.5 %, other C4
 *     4 %, pyrolysis gasoline (aromatics) 24.5 %, fuel oil 5 %; of propane: ethylene 42 %, propylene 16 %, methane
 *     26 %, hydrogen 1.5 %, butadiene 3 %, pyrolysis gasoline 7 %, other 4.5 % (Zimmermann and Walzl, "Ethylene",
 *     Ullmann's). About 26 GJ per tonne of ethylene (Ren, Patel and Blok, Energy 31 (2006) 425).</li>
 * </ul>
 * And (0.27) the same units at other operating conditions, each the table above at its nominal point:
 * <ul>
 * <li>fcc(riser outlet temperature, 480-560 C, nominal 530): four-lump riser kinetics (Lee, Chen, Huang and Pan, Can. J.
 *     Chem. Eng. 67 (1989) 615): gas oil cracks second order to gasoline, gas and coke, gasoline first order on to gas
 *     and coke, activation energies 68.25, 89.21, 64.42, 52.71 and 115.5 kJ/mol; the riser's space time is held (the
 *     temperature moves the rates). The rate constants at 530 C are fitted to the table (conversion 75.5 %), with the
 *     gasoline's overcracking set so its yield peaks at 540 C (units run their maximum-gasoline mode at 520-540 C,
 *     maximum distillate near 500 C, maximum olefins at 550 C and above: Sadeghbeigi ch. 6) and a twentieth of what
 *     overcracks going to coke. The table's splits stand within the lumps: gas is a sixth dry gas, five sixths LPG; what
 *     is left of the gas oil splits 17 to 7.5 between light cycle oil and slurry.</li>
 * <li>reformer(research octane, 95-104, nominal 98): the C5+ reformate falls as the octane is pushed up (about 1 % of the
 *     feed a number near 98, more at the top), the hydrogen rises 0.12 % a number (more aromatics made); LPG and fuel gas
 *     share the rest 8 to 4.5; the reactors' heat follows the hydrogen made (the dehydrogenation is what absorbs it)
 *     (Gary, Handwerk and Kaiser, ch. 10: reformate yield against octane for a medium naphtha; Little, Catalytic
 *     Reforming).</li>
 * <li>steamCracking(feed, coil outlet temperature, 800-860 C, nominal 835): a low-severity table at 800 C (propylene to
 *     ethylene about 0.67 for naphtha) and a high-severity one at 860 C (about 0.44), linear between them through the
 *     nominal table (Zimmermann and Walzl give the yields over that range); 26 GJ a tonne of ethylene throughout.</li>
 * </ul>
 */
public record Conversion(String name, String feed, Map<String, Double> yields, double energyJPerKgFeed) {
    public Conversion {
        yields = java.util.Collections.unmodifiableMap(new LinkedHashMap<>(yields));
        double sum = 0;
        for (double v : yields.values()) {
            if (!(v >= 0)) { throw new IllegalArgumentException("Yields are nonnegative"); }
            sum += v;
        }
        if (Math.abs(sum - 1) > 1e-6) { throw new IllegalArgumentException(name + ": yields add up to " + sum); }
    }

    private static Map<String, Double> of(Object... kv) {
        Map<String, Double> m = new LinkedHashMap<>();
        for (int i = 0; i < kv.length; i += 2) { m.put((String) kv[i], ((Number) kv[i + 1]).doubleValue()); }
        return m;
    }

    public static final Conversion FCC = new Conversion("fcc", "vacuum_gas_oil", of("fuel_gas", 0.035, "lpg", 0.17, "gasoline", 0.50,
            "light_cycle_oil", 0.17, "slurry", 0.075, "coke", 0.05), 0.3e6);
    public static final Conversion REFORMER = new Conversion("reformer", "heavy_naphtha", of("reformate", 0.85, "lpg", 0.08, "fuel_gas", 0.045,
            "hydrogen", 0.025), 1.8e6);
    public static final Conversion NAPHTHA_CRACKER = new Conversion("naphtha_cracker", "naphtha", of("hydrogen", 0.01, "methane", 0.15, "ethylene", 0.30,
            "propylene", 0.16, "butadiene", 0.045, "c4", 0.04, "aromatics", 0.245, "fuel_oil", 0.05), 26e6 * 0.30);
    public static final Conversion PROPANE_CRACKER = new Conversion("propane_cracker", "lpg", of("ethylene", 0.42, "propylene", 0.16, "methane", 0.26,
            "hydrogen", 0.015, "butadiene", 0.03, "aromatics", 0.07, "c4", 0.045), 26e6 * 0.42);

    public double yield(String product) { return yields.getOrDefault(product, 0.0); }

    // ---- 0.27: operating conditions ----

    /** Nominal conditions (where the tables above hold): riser outlet, C; research octane; coil outlet, C. */
    public static final double NOMINAL_RISER_C = 530, NOMINAL_OCTANE = 98, NOMINAL_COIL_C = 835;
    /** The ranges the conditions are given for. */
    public static final double RISER_MIN_C = 480, RISER_MAX_C = 560, OCTANE_MIN = 95, OCTANE_MAX = 104, COIL_MIN_C = 800, COIL_MAX_C = 860;

    private static final double GAS_CONSTANT = 8.314462618;
    /** Four-lump activation energies (J/mol): gas oil to gasoline, to gas, to coke; gasoline to gas, to coke. */
    static final double[] FCC_ACTIVATION = {68.25e3, 89.21e3, 64.42e3, 52.71e3, 115.5e3};
    /** Steps of the riser's integration over its space time. */
    private static final int RISER_STEPS = 400;
    /** Where gasoline peaks, C, and the share of overcracking that makes coke (the fit's two choices). */
    static final double FCC_GASOLINE_PEAK_C = 540, FCC_OVERCRACK_COKE = 0.05;
    /** The fitted rate constants times the space time at the nominal riser temperature, in FCC_ACTIVATION's order. */
    private static final double[] FCC_RATES = fitFcc();

    /** A riser's products per kg of feed: unconverted gas oil, gasoline, gas, coke. */
    record Riser(double gasOil, double gasoline, double gas, double coke) {}

    /** The riser's lumps at a temperature (K) for rate constants (times the space time) at the nominal one. */
    static Riser riser(double[] nominal, double kelvin) {
        double t0 = NOMINAL_RISER_C + 273.15;
        double[] a = new double[5];
        for (int i = 0; i < 5; i++) { a[i] = nominal[i] * Math.exp(-FCC_ACTIVATION[i] / GAS_CONSTANT * (1 / kelvin - 1 / t0)); }
        double crack = a[0] + a[1] + a[2], over = a[3] + a[4];
        // gas oil: second order, A(s) = 1 / (1 + crack s) over the space time s in 0..1
        double gasOil = 1 / (1 + crack);
        // gasoline: dB/ds = a0 A^2 - over B (RK4); what overcracks is integrated alongside
        double b = 0, lost = 0, h = 1.0 / RISER_STEPS;
        for (int k = 0; k < RISER_STEPS; k++) {
            double s = k * h;
            double k1 = rate(a[0], crack, over, s, b), k2 = rate(a[0], crack, over, s + h / 2, b + h / 2 * k1);
            double k3 = rate(a[0], crack, over, s + h / 2, b + h / 2 * k2), k4 = rate(a[0], crack, over, s + h, b + h * k3);
            double l1 = over * b, l2 = over * (b + h / 2 * k1), l3 = over * (b + h / 2 * k2), l4 = over * (b + h * k3);
            b += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            lost += h / 6 * (l1 + 2 * l2 + 2 * l3 + l4);
        }
        double converted = 1 - gasOil, primary = converted / crack;   // the integral of A^2 over the riser is the conversion / crack
        double toGas = over > 0 ? a[3] / over : 0;
        double gas = a[1] * primary + toGas * lost, coke = a[2] * primary + (1 - toGas) * lost;
        // close the balance on the gasoline (the integration's error, about 1e-12)
        return new Riser(gasOil, 1 - gasOil - gas - coke, gas, coke);
    }

    private static double rate(double toGasoline, double crack, double over, double s, double b) {
        double a = 1 / (1 + crack * s);
        return toGasoline * a * a - over * b;
    }

    /** The nominal rate constants: conversion, gasoline, gas and coke of the table at 530 C, gasoline peaking at FCC_GASOLINE_PEAK_C. */
    private static double[] fitFcc() {
        // more overcracking moves the peak down; it stays a fit while the gas and coke made directly from the gas oil are positive
        double lo = 1e-3;
        if (!rising(lo)) { throw new IllegalStateException("The FCC fit finds no gasoline peak above " + FCC_GASOLINE_PEAK_C + " C"); }
        double hi = lo + 0.02;
        while (rising(hi)) {
            lo = hi;
            hi += 0.02;
            if (hi > 20) { throw new IllegalStateException("The FCC fit finds no gasoline peak"); }
        }
        for (int i = 0; i < 60; i++) {
            double m = 0.5 * (lo + hi);
            if (rising(m)) { lo = m; } else { hi = m; }
        }
        return fitAt(lo);
    }

    /** Whether, with an overcracking constant, the fit is valid and the gasoline still rises at the peak temperature. */
    private static boolean rising(double over) {
        double[] a = fitAt(over);
        if (a == null) { return false; }
        double t = FCC_GASOLINE_PEAK_C + 273.15;
        return riser(a, t + 0.5).gasoline() - riser(a, t - 0.5).gasoline() > 0;
    }

    /** Rate constants at 530 C for an overcracking constant, matching the table's conversion, gasoline, gas and coke (null: none can). */
    private static double[] fitAt(double over) {
        double unconverted = FCC.yield("light_cycle_oil") + FCC.yield("slurry"), converted = 1 - unconverted;
        double crack = converted / unconverted;
        double[] probe = {crack, 0, 0, over * (1 - FCC_OVERCRACK_COKE), over * FCC_OVERCRACK_COKE};
        Riser r = riser(probe, NOMINAL_RISER_C + 273.15);
        // with all primary cracking to gasoline: what overcracks, as a share of the gasoline formed (it does not depend on how much forms)
        double lostShare = 1 - r.gasoline() / converted;
        double formed = FCC.yield("gasoline") / (1 - lostShare);   // gasoline formed by primary cracking, per kg of feed
        double lost = formed - FCC.yield("gasoline");
        double gasPrimary = FCC.yield("fuel_gas") + FCC.yield("lpg") - (1 - FCC_OVERCRACK_COKE) * lost;
        double cokePrimary = FCC.yield("coke") - FCC_OVERCRACK_COKE * lost;
        if (!(gasPrimary > 0) || !(cokePrimary > 0)) { return null; }
        return new double[]{crack * formed / converted, crack * gasPrimary / converted, crack * cokePrimary / converted, probe[3], probe[4]};
    }

    /** Fluid catalytic cracking at a riser outlet temperature, C (0.27; FCC at 530). */
    public static Conversion fcc(double riserOutletC) {
        check(riserOutletC, RISER_MIN_C, RISER_MAX_C, "Riser outlet temperature");
        if (riserOutletC == NOMINAL_RISER_C) { return FCC; }
        Riser r = riser(FCC_RATES, riserOutletC + 273.15);
        double dry = FCC.yield("fuel_gas") / (FCC.yield("fuel_gas") + FCC.yield("lpg"));
        double lco = FCC.yield("light_cycle_oil") / (FCC.yield("light_cycle_oil") + FCC.yield("slurry"));
        return new Conversion(FCC.name(), FCC.feed(), of("fuel_gas", dry * r.gas(), "lpg", (1 - dry) * r.gas(), "gasoline", r.gasoline(),
                "light_cycle_oil", lco * r.gasOil(), "slurry", (1 - lco) * r.gasOil(), "coke", r.coke()), FCC.energyJPerKgFeed());
    }

    /** Catalytic reforming to a research octane (0.27; REFORMER at 98). */
    public static Conversion reformer(double researchOctane) {
        check(researchOctane, OCTANE_MIN, OCTANE_MAX, "Research octane");
        if (researchOctane == NOMINAL_OCTANE) { return REFORMER; }
        double d = researchOctane - NOMINAL_OCTANE;
        double reformate = REFORMER.yield("reformate") - 0.0105 * d - 0.0003 * d * d;
        double hydrogen = REFORMER.yield("hydrogen") + 0.0012 * d;
        double rest = 1 - reformate - hydrogen, lpg = REFORMER.yield("lpg"), gas = REFORMER.yield("fuel_gas");
        return new Conversion(REFORMER.name(), REFORMER.feed(), of("reformate", reformate, "lpg", rest * lpg / (lpg + gas), "fuel_gas", rest * gas / (lpg + gas),
                "hydrogen", hydrogen), REFORMER.energyJPerKgFeed() * hydrogen / REFORMER.yield("hydrogen"));
    }

    /** The low (800 C) and high (860 C) severity tables, in the nominal tables' order. */
    private static final Map<String, Double> NAPHTHA_LOW = of("hydrogen", 0.008, "methane", 0.125, "ethylene", 0.255, "propylene", 0.170, "butadiene", 0.045,
            "c4", 0.065, "aromatics", 0.290, "fuel_oil", 0.042);
    private static final Map<String, Double> NAPHTHA_HIGH = of("hydrogen", 0.011, "methane", 0.165, "ethylene", 0.320, "propylene", 0.140, "butadiene", 0.046,
            "c4", 0.025, "aromatics", 0.235, "fuel_oil", 0.058);
    private static final Map<String, Double> PROPANE_LOW = of("ethylene", 0.37, "propylene", 0.20, "methane", 0.22, "hydrogen", 0.013, "butadiene", 0.027,
            "aromatics", 0.055, "c4", 0.115);
    private static final Map<String, Double> PROPANE_HIGH = of("ethylene", 0.45, "propylene", 0.12, "methane", 0.29, "hydrogen", 0.017, "butadiene", 0.032,
            "aromatics", 0.080, "c4", 0.011);

    /** Steam cracking of a feed ("naphtha" or "lpg") at a coil outlet temperature, C (0.27; the crackers above at 835). */
    public static Conversion steamCracking(String feed, double coilOutletC) {
        check(coilOutletC, COIL_MIN_C, COIL_MAX_C, "Coil outlet temperature");
        Conversion nominal = switch (feed) {
            case "naphtha" -> NAPHTHA_CRACKER;
            case "lpg" -> PROPANE_CRACKER;
            default -> throw new IllegalArgumentException("Steam cracking takes naphtha or lpg, not " + feed);
        };
        if (coilOutletC == NOMINAL_COIL_C) { return nominal; }
        boolean low = coilOutletC < NOMINAL_COIL_C;
        Map<String, Double> end = nominal == NAPHTHA_CRACKER ? (low ? NAPHTHA_LOW : NAPHTHA_HIGH) : (low ? PROPANE_LOW : PROPANE_HIGH);
        double t = low ? (NOMINAL_COIL_C - coilOutletC) / (NOMINAL_COIL_C - COIL_MIN_C) : (coilOutletC - NOMINAL_COIL_C) / (COIL_MAX_C - NOMINAL_COIL_C);
        Map<String, Double> y = new LinkedHashMap<>();
        for (Map.Entry<String, Double> e : nominal.yields().entrySet()) { y.put(e.getKey(), e.getValue() + t * (end.get(e.getKey()) - e.getValue())); }
        return new Conversion(nominal.name(), nominal.feed(), y, 26e6 * y.get("ethylene"));
    }

    private static void check(double value, double min, double max, String what) {
        if (!(value >= min && value <= max)) { throw new IllegalArgumentException(what + " is " + min + " to " + max + ": " + value); }
    }
}

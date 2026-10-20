package dev.mcmod.aerocore.api.engine;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The gas path pieces the gas turbines share (Turbojet, Turbofan, FreeTurbine): burned gas of variable properties,
 * the burner, ducts whose losses go with the dynamic head, nozzles (a turbine passes flow as one across its pressure
 * ratio) and the inlet's recovery.
 */
final class GasPath {
    private GasPath() {}

    /** A convergent-divergent jet separates when its exit would be below this fraction of ambient (Summerfield's criterion). */
    static final double SEPARATION = 0.4;

    /** Gas that is a mass blend of air and stoichiometric products: lean products of a fuel-air ratio f. */
    static final class Mix implements dev.mcmod.aerocore.api.gasdynamics.Thermo {
        final double share, r;   // mass fraction of stoichiometric products
        final Gas products;
        Mix(Fuel fuel, double fuelAirRatio) { this(stoichiometric(fuel), fuel.stoichiometricAirFuel(), fuelAirRatio); }
        /** With the fuel's stoichiometric products and air-fuel ratio already at hand. */
        Mix(Gas products, double afr, double fuelAirRatio) {
            share = Math.clamp(fuelAirRatio * (1 + afr) / (1 + fuelAirRatio), 0, 1);
            this.products = products;
            r = share * products.gasConstant() + (1 - share) * Gas.AIR.gasConstant();
        }
        double h(double t) { return share * products.enthalpy(t) + (1 - share) * Gas.AIR.enthalpy(t); }
        double phi(double t) { return share * products.entropyFunction(t) + (1 - share) * Gas.AIR.entropyFunction(t); }
        @Override public double cp(double t) { return share * products.cp(t) + (1 - share) * Gas.AIR.cp(t); }
        // the gas dynamics' view of the same gas (api.gasdynamics.Thermo)
        @Override public double gasConstant() { return r; }
        @Override public double enthalpy(double t) { return h(t); }
        @Override public double entropyFunction(double t) { return phi(t); }
        @Override public double temperatureAtEnthalpy(double h) { return tAtH(h); }
        @Override public double temperatureAtEntropy(double phi) { return tAtPhi(phi); }
        double tAtH(double h) {
            double t = 1000;
            for (int i = 0; i < 40; i++) { double step = (h(t) - h) / cp(t); t -= step; if (Math.abs(step) < 1e-9) { break; } }
            return t;
        }
        double tAtPhi(double phi) {
            // Newton in ln T: phi is nearly linear there (phi ~ cp ln T), so it cannot overshoot below zero
            double y = Math.log(1000);
            for (int i = 0; i < 40; i++) {
                double t = Math.exp(y), step = (phi(t) - phi) / cp(t);
                y -= step;
                if (Math.abs(step) < 1e-10) { break; }
            }
            return Math.exp(y);
        }
        /** Total temperature after a turbine of polytropic efficiency eta takes the pressure ratio exp(lnPr) from total temperature tt. */
        double turbineExit(double tt, double lnPr, double eta) { return tAtPhi(phi(tt) - eta * r * lnPr); }
        /**
         * Static state where the flow is sonic, from total temperature: {T*, p* / pt, mass flux / pt (s/m)}. The
         * velocity squared 2 (h(tt) - h(T)) falls and the speed of sound squared gamma R T rises with T, so their
         * difference has one root; Newton's method from the perfect-gas value, kept inside (tt / 2, tt).
         */
        double[] sonic(double tt) {
            double ht = h(tt), cpt = cp(tt), t = 2 * tt / (cpt / (cpt - r) + 1), lo = 0.5 * tt, hi = tt;
            for (int i = 0; i < 40; i++) {
                double cp = cp(t), g = cp / (cp - r), residual = 2 * (ht - h(t)) - g * r * t;
                if (residual > 0) { lo = t; } else { hi = t; }
                double next = t + residual / (2 * cp + g * r);
                if (!(next > lo && next < hi)) { next = 0.5 * (lo + hi); }
                boolean done = Math.abs(next - t) < 1e-11 * tt;
                t = next;
                if (done) { break; }
            }
            double pr = Math.exp(-(phi(tt) - phi(t)) / r), v = Math.sqrt(Math.max(0, 2 * (ht - h(t))));
            return new double[]{t, pr, pr / (r * t) * v};
        }
        /** Mass flux over total pressure (s/m) of flow expanding from total temperature tt by a total-to-static pressure ratio; sonic at most. */
        double flux(double tt, double pressureRatio) {
            double[] star = sonic(tt);
            if (1 / pressureRatio <= star[1]) { return star[2]; }
            double[] e = expand(tt, 1, 1 / pressureRatio);
            return 1 / pressureRatio / (r * e[0]) * e[1];
        }
        /** Static temperature and speed after expanding from (tt, pt) to p. */
        double[] expand(double tt, double pt, double p) {
            double t = tAtPhi(phi(tt) - r * Math.log(pt / p));
            return new double[]{t, Math.sqrt(Math.max(0, 2 * (h(tt) - h(t))))};
        }
        /**
         * Static pressure (Pa) of subsonic flow from total (tt, pt) that carries a mass flux (kg/(s m^2)): the inverse
         * of the flux on its subsonic branch; the sonic pressure if the flux is more than the flow can carry.
         */
        double staticForFlux(double tt, double pt, double massFlux) {
            double[] star = sonic(tt);
            if (massFlux >= star[2] * pt) { return star[1] * pt; }
            if (massFlux <= 0) { return pt; }
            double lo = star[1] * pt, hi = pt;   // the flux falls from sonic at lo to nothing at hi
            for (int i = 0; i < 60 && hi - lo > 1e-10 * pt; i++) {
                double p = 0.5 * (lo + hi);
                double[] e = expand(tt, pt, p);
                if (p / (r * e[0]) * e[1] > massFlux) { lo = p; } else { hi = p; }
            }
            return 0.5 * (lo + hi);
        }
    }

    /** Pressure retained by a duct that keeps r at design, at a corrected flow (fraction of design): the loss goes with the dynamic head. */
    static double retained(double designRetention, double flowFraction) {
        return Math.max(0.3, 1 - (1 - designRetention) * flowFraction * flowFraction);
    }
    /** Flow corrected to the reference state, kg/s: w sqrt(T / T_REF) / (p / P_REF). */
    static double corrected(double flow, double tt, double pt) { return flow * Math.sqrt(tt / Turbojet.T_REF) * Turbojet.P_REF / pt; }

    private static final Map<Fuel, Gas> STOICHIOMETRIC = new ConcurrentHashMap<>();
    static Gas stoichiometric(Fuel fuel) { return STOICHIOMETRIC.computeIfAbsent(fuel, f -> f.products(1)); }

    /** Fuel-air ratio (per kilogram of burner air) that heats air from T3 to T4 in a burner of efficiency eta (fuel at 298 K). */
    static double fuelAirRatio(Fuel fuel, double t3, double t4, double eta) {
        Gas s = stoichiometric(fuel), a = Gas.AIR;
        double afr = fuel.stoichiometricAirFuel();
        double f = (a.enthalpy(t4) - a.enthalpy(t3)) / (eta * fuel.lowerHeatingValueJPerKg() - (1 + afr) * s.enthalpy(t4) + afr * a.enthalpy(t4));
        return Math.max(0, f);
    }
    /** Burner exit temperature for a fuel-air ratio (inverse of fuelAirRatio), K. */
    static double burnerExit(Fuel fuel, double t3, double f, double eta) {
        Mix m = new Mix(fuel, f);
        return m.tAtH((Gas.AIR.enthalpy(t3) + f * eta * fuel.lowerHeatingValueJPerKg()) / (1 + f));
    }

    /**
     * Inlet total pressure recovery at a flight Mach number and a corrected airflow (fraction of design): the subsonic
     * duct keeps designRecovery at design flow and loses with the square of it; above Mach 1 a pitot inlet loses a
     * normal shock, an external-compression ramp inlet follows MIL-E-5008B (as given by Mattingly, Elements of
     * Propulsion): 1 - 0.075 (M - 1)^1.35 up to Mach 5, 800 / (M^4 + 935) above.
     */
    static double inletRecovery(double designRecovery, Turbojet.Inlet inlet, double mach, double flowFraction) {
        double r = retained(designRecovery, flowFraction);
        if (mach <= 1) { return r; }
        if (inlet == Turbojet.Inlet.RAMP) {
            double spec = mach <= 5 ? 1 - 0.075 * Math.pow(mach - 1, 1.35) : 800 / (Math.pow(mach, 4) + 935);
            return r * Math.max(0.05, spec);
        }
        double g = 1.4, m2 = mach * mach;
        double shock = Math.pow((g + 1) * m2 / ((g - 1) * m2 + 2), g / (g - 1)) * Math.pow((g + 1) / (2 * g * m2 - (g - 1)), 1 / (g - 1));
        return r * shock;
    }

    /** Nozzle exit for gas (mix) at nozzle total (tt, pt) into ambient p0 through throat a8 (and exit a9): {mass flow, exit speed, exit pressure, exit area, choked 0/1}. */
    static double[] nozzle(Mix m, double tt, double pt, double p0, double a8, boolean cd, double areaRatio) {
        // No outflow (a stopped spool, whose fixed-fraction losses leave the tailpipe below ambient): the exit sits at ambient.
        if (pt <= p0) { return new double[]{0, 0, p0, a8, 0}; }
        double[] star = m.sonic(tt);
        double pStar = star[1] * pt;
        if (p0 >= pStar) {   // unchoked: subsonic to ambient at the throat
            double[] e = m.expand(tt, pt, Math.min(p0, pt));
            double flux = Math.min(p0, pt) / (m.r * e[0]) * e[1];
            return new double[]{flux * a8, e[1], Math.min(p0, pt), a8, 0};
        }
        double flow = star[2] * pt * a8;
        if (!cd || areaRatio <= 1.0000001) {
            double[] e = m.expand(tt, pt, pStar);
            return new double[]{flow, e[1], pStar, a8, 1};
        }
        // supersonic exit at the area ratio: mass flux falls to flux* / ratio below p*
        double target = star[2] * pt / areaRatio, lo = 1e-4 * pt, hi = pStar;
        for (int i = 0; i < 60; i++) {
            double p = Math.sqrt(lo * hi);
            double[] e = m.expand(tt, pt, p);
            double flux = p / (m.r * e[0]) * e[1];
            if (flux > target) { hi = p; } else { lo = p; }
        }
        double p9 = Math.sqrt(lo * hi);
        if (p9 < SEPARATION * p0) {   // the jet separates and leaves at ambient
            double[] e = m.expand(tt, pt, p0);
            return new double[]{flow, e[1], p0, a8 * areaRatio, 1};
        }
        double[] e = m.expand(tt, pt, p9);
        return new double[]{flow, e[1], p9, a8 * areaRatio, 1};
    }

    /** Throat area (m^2) that passes a flow of gas at total (tt, pt): sonic, or (when the pressure ratio is too small to choke) subsonic to ambient p0. */
    static double throatFor(Mix m, double flow, double tt, double pt, double p0) {
        double[] star = m.sonic(tt);
        if (star[1] * pt > p0) { return flow / (star[2] * pt); }
        double[] e = m.expand(tt, pt, p0);
        return flow / (p0 / (m.r * e[0]) * e[1]);
    }
}

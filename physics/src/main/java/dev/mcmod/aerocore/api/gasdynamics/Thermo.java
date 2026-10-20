package dev.mcmod.aerocore.api.gasdynamics;

/**
 * A thermally perfect gas (p = rho R T, enthalpy and cp functions of temperature only), the way the gas dynamics of this
 * package see it: enthalpy and the entropy function phi(T) = integral of cp / T dT, so an isentropic change from (T1,
 * p1) to (T2, p2) has phi(T2) - phi(T1) = R ln(p2 / p1). Air and burned gas of the engines (api.engine.Gas) are ones;
 * perfect(gamma, R) is the calorically perfect gas of the textbook tables. SI units.
 */
public interface Thermo {
    /** Specific gas constant, J/(kg K). */
    double gasConstant();
    /** Sensible enthalpy, J/kg (any fixed reference). */
    double enthalpy(double temperatureK);
    /** Heat capacity at constant pressure, J/(kg K). */
    double cp(double temperatureK);
    /** Entropy function phi(T), J/(kg K) (any fixed reference). */
    double entropyFunction(double temperatureK);

    /** Ratio of specific heats at T. */
    default double gamma(double temperatureK) {
        double cp = cp(temperatureK);
        return cp / (cp - gasConstant());
    }

    /** Speed of sound, m/s. */
    default double speedOfSound(double temperatureK) { return Math.sqrt(gamma(temperatureK) * gasConstant() * temperatureK); }

    /** The temperature with enthalpy h (Newton from 1000 K), K. */
    default double temperatureAtEnthalpy(double enthalpyJPerKg) {
        double t = 1000;
        for (int i = 0; i < 60; i++) {
            double step = (enthalpy(t) - enthalpyJPerKg) / cp(t);
            t = Math.max(1e-3, t - step);
            if (Math.abs(step) < 1e-9 * t) { break; }
        }
        return t;
    }

    /** The temperature with entropy function phi (Newton in ln T, where phi is nearly linear), K. */
    default double temperatureAtEntropy(double phi) {
        double y = Math.log(1000);
        for (int i = 0; i < 60; i++) {
            double t = Math.exp(y), step = (entropyFunction(t) - phi) / cp(t);
            y -= step;
            if (Math.abs(step) < 1e-12) { break; }
        }
        return Math.exp(y);
    }

    /** A calorically perfect gas: constant gamma and R (the gas of NACA Report 1135's tables). */
    static Thermo perfect(double gamma, double gasConstant) {
        if (!(gamma > 1) || !(gasConstant > 0)) { throw new IllegalArgumentException("A perfect gas needs gamma > 1 and R > 0"); }
        double cp = gamma * gasConstant / (gamma - 1);
        return new Thermo() {
            @Override public double gasConstant() { return gasConstant; }
            @Override public double enthalpy(double t) { return cp * t; }
            @Override public double cp(double t) { return cp; }
            @Override public double entropyFunction(double t) { return cp * Math.log(t); }
            @Override public double gamma(double t) { return gamma; }
            @Override public double temperatureAtEnthalpy(double h) { return h / cp; }
            @Override public double temperatureAtEntropy(double phi) { return Math.exp(phi / cp); }
        };
    }

    /** Air as the tables take it: gamma 1.4, R 287.05 J/(kg K). */
    Thermo PERFECT_AIR = perfect(1.4, 287.05287);
}

package dev.mcmod.aerocore.api.gasdynamics;

/**
 * A normal shock in a calorically perfect gas (Rankine-Hugoniot; NACA Report 1135 eqs. 93-100): the Mach number
 * behind it and the static and total ratios across it (behind over ahead).
 */
public record NormalShock(double machAhead, double machBehind, double pressureRatio, double temperatureRatio, double densityRatio,
                          double totalPressureRatio) {

    public static NormalShock of(double mach, double gamma) {
        if (!(mach >= 1) || !Double.isFinite(mach) || !(gamma > 1)) { throw new IllegalArgumentException("A normal shock needs M >= 1 and gamma > 1"); }
        double m2 = mach * mach, g = gamma;
        double behind = Math.sqrt((1 + 0.5 * (g - 1) * m2) / (g * m2 - 0.5 * (g - 1)));
        double p = 1 + 2 * g / (g + 1) * (m2 - 1);
        double rho = (g + 1) * m2 / ((g - 1) * m2 + 2);
        double t = p / rho;
        double p0 = Math.pow(rho, g / (g - 1)) * Math.pow(1 / p, 1 / (g - 1));
        return new NormalShock(mach, behind, p, t, rho, p0);
    }

    /**
     * A normal shock in a thermally perfect gas from the state ahead (static T, p and the normal speed): mass,
     * momentum and enthalpy conserved with the gas's own h(T), iterating on the density ratio (Anderson, Hypersonic
     * and High-Temperature Gas Dynamics, sec. 14.3). Returns the state behind.
     */
    public static GasState behind(Thermo gas, GasState ahead) {
        double r = gas.gasConstant(), t1 = ahead.temperatureK(), p1 = ahead.pressurePa(), u1 = ahead.speedMps();
        if (!(u1 > gas.speedOfSound(t1) * (1 - 1e-12))) { throw new IllegalArgumentException("The flow ahead of a shock must be supersonic"); }
        double rho1 = p1 / (r * t1), h1 = gas.enthalpy(t1), mass = rho1 * u1;
        double eps = 1 / NormalShock.of(ahead.mach(gas), gas.gamma(t1)).densityRatio();   // perfect-gas start
        double t2 = t1, p2 = p1;
        for (int i = 0; i < 200; i++) {
            p2 = p1 + mass * u1 * (1 - eps);
            t2 = gas.temperatureAtEnthalpy(h1 + 0.5 * u1 * u1 * (1 - eps * eps));
            double next = rho1 / (p2 / (r * t2));
            boolean done = Math.abs(next - eps) < 1e-13;
            eps = next;
            if (done) { break; }
        }
        p2 = p1 + mass * u1 * (1 - eps);
        t2 = gas.temperatureAtEnthalpy(h1 + 0.5 * u1 * u1 * (1 - eps * eps));
        return new GasState(t2, p2, eps * u1);
    }
}

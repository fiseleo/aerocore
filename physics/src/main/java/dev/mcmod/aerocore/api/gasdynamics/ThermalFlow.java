package dev.mcmod.aerocore.api.gasdynamics;

/**
 * Isentropic flow of a thermally perfect gas (cp a function of T): the stagnation state, the sonic state, expansion
 * to a pressure, mass flux, and the Prandtl-Meyer turn, all from the gas's enthalpy and entropy function instead of a
 * fixed gamma (Anderson, Hypersonic and High-Temperature Gas Dynamics, ch. 14; the same method as the engines' burned
 * gas). Above about 1000 K air's gamma falls (1.40 at 300 K, 1.33 at 1500 K), which is what makes hypersonic inlets and
 * combustors differ from the tables.
 */
public final class ThermalFlow {
    private ThermalFlow() {}

    /**
     * Static state where the flow is sonic, from total temperature tt: {T*, p* / pt, mass flux / pt (s/m)}. The
     * velocity squared 2 (h(tt) - h(T)) falls and the speed of sound squared gamma R T rises with T, so their difference
     * has one root.
     */
    public static double[] sonic(Thermo gas, double tt) {
        double r = gas.gasConstant(), ht = gas.enthalpy(tt), cpt = gas.cp(tt);
        double t = 2 * tt / (cpt / (cpt - r) + 1), lo = 0.3 * tt, hi = tt;
        for (int i = 0; i < 80; i++) {
            double cp = gas.cp(t), g = cp / (cp - r), residual = 2 * (ht - gas.enthalpy(t)) - g * r * t;
            if (residual > 0) { lo = t; } else { hi = t; }
            double next = t + residual / (2 * cp + g * r);
            if (!(next > lo && next < hi)) { next = 0.5 * (lo + hi); }
            boolean done = Math.abs(next - t) < 1e-12 * tt;
            t = next;
            if (done) { break; }
        }
        double pr = Math.exp(-(gas.entropyFunction(tt) - gas.entropyFunction(t)) / r), v = Math.sqrt(Math.max(0, 2 * (ht - gas.enthalpy(t))));
        return new double[]{t, pr, pr / (r * t) * v};
    }

    /** The static state after expanding (or compressing) isentropically from total (tt, pt) to static pressure p. */
    public static GasState expand(Thermo gas, double tt, double pt, double p) {
        double t = gas.temperatureAtEntropy(gas.entropyFunction(tt) - gas.gasConstant() * Math.log(pt / p));
        return new GasState(t, p, Math.sqrt(Math.max(0, 2 * (gas.enthalpy(tt) - gas.enthalpy(t)))));
    }

    /** The static state of flow from total (tt, pt) at Mach M. */
    public static GasState atMach(Thermo gas, double tt, double pt, double mach) {
        if (!(mach >= 0)) { throw new IllegalArgumentException("Mach must be nonnegative"); }
        double ht = gas.enthalpy(tt), r = gas.gasConstant();
        // 2 (h(tt) - h(T)) = M^2 gamma(T) R T: the left falls and the right rises with T
        double lo = 1e-3, hi = tt;
        for (int i = 0; i < 200 && hi - lo > 1e-12 * tt; i++) {
            double t = 0.5 * (lo + hi);
            if (2 * (ht - gas.enthalpy(t)) > mach * mach * gas.gamma(t) * r * t) { lo = t; } else { hi = t; }
        }
        double t = 0.5 * (lo + hi);
        double p = pt * Math.exp(-(gas.entropyFunction(tt) - gas.entropyFunction(t)) / r);
        return new GasState(t, p, mach * gas.speedOfSound(t));
    }

    /** The static state of flow from total (tt, pt) that carries a mass flux (kg/(s m^2)) on the subsonic or supersonic branch; the sonic state if the flux is more than the flow can carry. */
    public static GasState atMassFlux(Thermo gas, double tt, double pt, double massFlux, boolean supersonic) {
        double[] star = sonic(gas, tt);
        if (massFlux >= star[2] * pt) { return expand(gas, tt, pt, star[1] * pt); }
        if (massFlux <= 0) { return supersonic ? expand(gas, tt, pt, 1e-9 * pt) : new GasState(tt, pt, 0); }
        double lo = supersonic ? 1e-12 * pt : star[1] * pt, hi = supersonic ? star[1] * pt : pt;
        for (int i = 0; i < 200 && hi - lo > 1e-12 * pt; i++) {
            double p = supersonic ? Math.sqrt(lo * hi) : 0.5 * (lo + hi);
            GasState e = expand(gas, tt, pt, p);
            double flux = e.massFlux(gas);
            // subsonic: the flux falls toward pt; supersonic: it falls toward vacuum
            if ((flux > massFlux) != supersonic) { lo = p; } else { hi = p; }
        }
        return expand(gas, tt, pt, supersonic ? Math.sqrt(lo * hi) : 0.5 * (lo + hi));
    }

    /**
     * A Prandtl-Meyer expansion of a supersonic stream through a turn (radians, positive opens the flow): along the
     * isentrope from its stagnation state, d nu = sqrt(M^2 - 1) dV / V, integrated by Simpson's rule in static
     * temperature until the turn is reached. Compressive turns are oblique shocks (ObliqueShock.behind), not this.
     */
    public static GasState turn(Thermo gas, GasState ahead, double angle) {
        if (!(angle >= 0)) { throw new IllegalArgumentException("A Prandtl-Meyer turn must open the flow"); }
        if (!(ahead.mach(gas) >= 1)) { throw new IllegalArgumentException("A Prandtl-Meyer turn needs supersonic flow"); }
        if (angle == 0) { return ahead; }
        double tt = ahead.totalTemperature(gas), pt = ahead.totalPressure(gas), ht = gas.enthalpy(tt);
        double t = ahead.temperatureK(), nu = 0, dt = -t / 4000;
        java.util.function.DoubleUnaryOperator dnu = temp -> {
            double v2 = 2 * (ht - gas.enthalpy(temp)), a2 = gas.gamma(temp) * gas.gasConstant() * temp;
            // dV / dT = -cp / V; d nu = sqrt(M^2 - 1) dV / V
            return Math.sqrt(Math.max(0, v2 / a2 - 1)) * (-gas.cp(temp)) / v2;
        };
        while (t + dt > 1) {
            double step = (dnu.applyAsDouble(t) + 4 * dnu.applyAsDouble(t + dt / 2) + dnu.applyAsDouble(t + dt)) / 6 * dt;
            if (nu + step >= angle) {
                // the remaining turn inside this step: linear in T
                t += dt * (angle - nu) / step;
                nu = angle;
                break;
            }
            nu += step;
            t += dt;
        }
        if (nu < angle) { throw new IllegalArgumentException("The turn exceeds the stream's largest Prandtl-Meyer turn"); }
        double p = pt * Math.exp(-(gas.entropyFunction(tt) - gas.entropyFunction(t)) / gas.gasConstant());
        return new GasState(t, p, Math.sqrt(Math.max(0, 2 * (ht - gas.enthalpy(t)))));
    }
}

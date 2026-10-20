package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.engine.GasPath.Mix;
import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import dev.mcmod.aerocore.api.gasdynamics.ObliqueShock;
import dev.mcmod.aerocore.api.gasdynamics.PerfectGas;
import dev.mcmod.aerocore.api.gasdynamics.Rayleigh;

import java.util.Objects;

/**
 * A ramjet (0.28): a supersonic inlet, a constant-area combustor behind flameholders and a choked nozzle; one
 * dimensional, steady (Mattingly, Elements of Propulsion, ch. 7 and 10; Zucrow, Aircraft and Missile Propulsion II,
 * ch. 9; Seddon and Goldsmith, Intake Aerodynamics, for the inlet's matching).
 * <ul>
 * <li>Combustor and nozzle: the choked throat (area A8, the nozzle keeping its share of total pressure) fixes the
 * combustor's exit Mach number by the area ratio, and the heat added at constant area fixes its entry Mach number by
 * Rayleigh flow (gamma of the gas at the mean temperature); the flameholders lose K dynamic heads. So for a combustor
 * temperature the engine asks for an airflow in proportion to the inlet's recovery.</li>
 * <li>Matching (Seddon and Goldsmith ch. 11): if the engine would take more than the inlet captures at its critical point,
 * the terminal shock moves into the diffuser and the recovery falls until the two agree (supercritical); if less, the
 * shock moves out ahead of the lip and the rest spills (subcritical), with Ferri's buzz below 85 % of the critical
 * flow. Spilled air leaves with the momentum it lost through the shock it crossed (additive drag, approximately).</li>
 * <li>Fuel: the burner heats the inlet's air at its total temperature to the combustor temperature (GasPath, lean, at
 * most stoichiometric); the fuel's mass is neglected in the Rayleigh relations (a few percent).</li>
 * </ul>
 * Thrust is the nozzle's gross thrust less the captured air's ram drag and the spillage drag. Sized at a design flight
 * condition and combustor temperature so the inlet runs a little supercritical (the throat a margin over critical).
 * Not modelled: flameholder blowout and ignition limits (the part's rules), inlet unstart dynamics and buzz cycles,
 * combustion instability, cooling.
 */
public final class Ramjet {
    private Ramjet() {}

    public enum Mode { SUBSONIC, SUPERCRITICAL, CRITICAL, SUBCRITICAL, BUZZ }

    /** Below this share of the critical flow a subcritical inlet buzzes (Ferri and Nucci, NACA RM L50K30, about 0.85). */
    public static final double BUZZ = 0.85;

    /**
     * A ramjet: inlet, capture area, combustor cross-section (m^2), flameholder loss in dynamic heads (V-gutters about
     * 1-2), burner efficiency, largest combustor temperature, the nozzle's retained total pressure, convergent or
     * convergent-divergent with its exit area ratio, the throat's margin over the critical flow at design, and the fuel.
     */
    public record Design(SupersonicInlet.Design inlet, double captureAreaM2, double combustorAreaM2, double flameholderLoss, double burnerEfficiency,
                         double maxTemperatureK, double nozzlePressureRatio, boolean convergentDivergent, double exitAreaRatio, double supercriticalMargin,
                         Fuel fuel) {
        public Design {
            Objects.requireNonNull(inlet, "inlet"); Objects.requireNonNull(fuel, "fuel");
            if (!(captureAreaM2 > 0) || !(combustorAreaM2 > captureAreaM2 * 0.5) || !(flameholderLoss >= 0) || !(burnerEfficiency > 0 && burnerEfficiency <= 1)
                    || !(maxTemperatureK > 0) || !(nozzlePressureRatio > 0 && nozzlePressureRatio <= 1) || !(supercriticalMargin >= 1)) {
                throw new IllegalArgumentException("A ramjet needs a capture area, a combustor at least half its size, losses and efficiencies in range");
            }
            if (convergentDivergent && !(exitAreaRatio >= 1)) { throw new IllegalArgumentException("A convergent-divergent nozzle needs an exit area ratio of at least 1"); }
        }
    }

    /** A sized ramjet: its design, the design flight condition and combustor temperature, the throat area, the design point, its inlet's table. */
    public record Sizing(Design design, Turbojet.Flight condition, double designTemperatureK, double throatAreaM2, Point point, SupersonicInlet.Table inlet) {}

    /**
     * An operating point: thrust (N, net of ram and spillage drag), fuel (kg/s), airflow (kg/s), inlet recovery,
     * captured over capture-area stream, the matching mode, the combustor's entry Mach number and speed (m/s), the
     * equivalence ratio, the combustor exit's total temperature and pressure, the exit speed, the spillage drag (N).
     */
    public record Point(double thrustN, double fuelKgPerS, double airflowKgPerS, double recovery, double captureRatio, Mode mode, double combustorMach,
                        double combustorSpeedMps, double equivalenceRatio, double tt4, double pt4, double exitSpeedMps, double spillageDragN) {
        /** Thrust-specific fuel consumption, kg/(N s) (infinite at no thrust). */
        public double tsfc() { return thrustN > 0 ? fuelKgPerS / thrustN : Double.POSITIVE_INFINITY; }
        /** Fuel specific impulse, s. */
        public double specificImpulse() { return fuelKgPerS > 0 ? thrustN / (fuelKgPerS * 9.80665) : 0; }
    }

    /** Sizes the throat so the inlet runs the margin supercritical at a design flight condition and combustor temperature. */
    public static Sizing size(Design d, Turbojet.Flight flight, double temperatureK) {
        if (!(flight.mach() > 1)) { throw new IllegalArgumentException("A ramjet is sized at a supersonic design point"); }
        double lo = 1e-6 * d.combustorAreaM2(), hi = 0.999 * d.combustorAreaM2();
        SupersonicInlet.Critical c = SupersonicInlet.critical(d.inlet(), flight.mach());
        double supply = flight.pressurePa() / (Gas.AIR.gasConstant() * flight.temperatureK()) * flight.speedMps() * d.captureAreaM2() * c.captureRatio();
        double target = d.supercriticalMargin() * supply;
        for (int i = 0; i < 200 && hi - lo > 1e-12 * d.combustorAreaM2(); i++) {
            double a8 = Math.sqrt(lo * hi);
            if (demand(d, a8, flight, temperatureK, c.recovery()) < target) { lo = a8; } else { hi = a8; }
        }
        double a8 = Math.sqrt(lo * hi);
        SupersonicInlet.Table table = SupersonicInlet.Table.of(d.inlet(), Math.max(6, d.inlet().designMach() + 3));
        Sizing provisional = new Sizing(d, flight, temperatureK, a8, null, table);
        return new Sizing(d, flight, temperatureK, a8, at(provisional, flight, temperatureK), table);
    }

    /** Airflow (kg/s) the combustor and nozzle take at a recovery: the combustor chain at that combustor temperature. */
    private static double demand(Design d, double a8, Turbojet.Flight flight, double t4, double recovery) {
        return chain(d, a8, flight, t4)[0] * recovery * flight.totalPressure();
    }

    /** {airflow per unit of inlet total pressure (kg/s/Pa), combustor entry Mach, combustor exit Mach, pt4/pt2, fuel-air ratio}. */
    private static double[] chain(Design d, double a8, Turbojet.Flight flight, double t4) {
        double tt0 = flight.totalTemperature();
        double f = t4 > tt0 ? GasPath.fuelAirRatio(d.fuel(), tt0, t4, d.burnerEfficiency()) : 0;
        Mix hot = new Mix(d.fuel(), f);
        double tExit = f > 0 ? t4 : tt0;
        double g = hot.gamma(0.5 * (tt0 + tExit));
        // combustor exit Mach: the throat passes the flow at its share of total pressure, so the exit's sonic area is A8 times that share
        double ratio = d.combustorAreaM2() / (a8 * d.nozzlePressureRatio());
        double m4 = PerfectGas.machAtAreaRatio(Math.max(1, ratio), g, false);
        double t0Ratio = Rayleigh.totalTemperatureRatio(m4, g) * tt0 / tExit;
        double m3 = Rayleigh.machAtTotalTemperatureRatio(Math.min(1, t0Ratio), g, false);
        double rayleigh = Rayleigh.totalPressureRatio(m4, g) / Rayleigh.totalPressureRatio(m3, g);
        double holder = Math.max(0.05, 1 - d.flameholderLoss() * 0.5 * g * m3 * m3 / PerfectGas.totalToStaticPressure(m3, g));
        double perPt = a8 * hot.sonic(tExit)[2] * d.nozzlePressureRatio() * holder * rayleigh / (1 + f);
        return new double[]{perPt, m3, m4, holder * rayleigh, f};
    }

    /** The operating point at a flight condition and combustor temperature (clamped to the largest, and to stoichiometric). */
    public static Point at(Sizing s, Turbojet.Flight flight, double temperatureK) {
        Design d = s.design();
        double tt0 = flight.totalTemperature(), pt0 = flight.totalPressure(), mach = flight.mach();
        double t4 = Math.clamp(temperatureK, tt0, d.maxTemperatureK());
        double stoich = GasPath.burnerExit(d.fuel(), tt0, 1 / d.fuel().stoichiometricAirFuel(), d.burnerEfficiency());
        t4 = Math.min(t4, stoich);
        double[] ch = chain(d, s.throatAreaM2(), flight, t4);
        SupersonicInlet.Critical c = s.inlet().at(mach);
        double rho0 = flight.pressurePa() / (Gas.AIR.gasConstant() * flight.temperatureK()), v0 = flight.speedMps();
        double full = rho0 * v0 * d.captureAreaM2();
        double airflow, recovery, spill = 0, captured;
        Mode mode;
        if (mach <= 1) {
            recovery = c.recovery();
            airflow = ch[0] * recovery * pt0;
            captured = full > 0 ? airflow / full : Double.NaN;
            mode = Mode.SUBSONIC;
        } else {
            double supply = full * c.captureRatio(), want = ch[0] * c.recovery() * pt0;
            if (want >= supply * 1.01) {
                // supercritical: the terminal shock in the diffuser takes the recovery down until the engine takes what is captured
                double floor = NormalShock.of(mach, 1.4).totalPressureRatio() * d.inlet().subsonicRecovery() * 0.5;
                recovery = Math.max(floor, supply / (ch[0] * pt0));
                airflow = supply;
                mode = Mode.SUPERCRITICAL;
            } else if (want >= supply * 0.99) {
                recovery = c.recovery();
                airflow = want;
                mode = Mode.CRITICAL;
            } else {
                recovery = c.recovery();
                airflow = want;
                mode = airflow < BUZZ * supply ? Mode.BUZZ : Mode.SUBCRITICAL;
            }
            captured = airflow / full;
            // spillage: the geometric spill crossed the first shock, the subcritical spill a normal shock
            double vFirst = firstShockSpeed(d.inlet(), flight), vNormal = v0 * NormalShock.of(mach, 1.4).machBehind()
                    * Math.sqrt(NormalShock.of(mach, 1.4).temperatureRatio()) / mach;
            spill = (full - supply) * (v0 - vFirst) + Math.max(0, supply - airflow) * (v0 - vNormal);
        }
        double f = ch[4], pt4 = ch[3] * recovery * pt0;
        Mix hot = new Mix(d.fuel(), f);
        double tExit = f > 0 ? t4 : tt0;
        double[] nz = GasPath.nozzle(hot, tExit, pt4 * d.nozzlePressureRatio(), flight.pressurePa(), s.throatAreaM2(), d.convergentDivergent(), d.exitAreaRatio());
        double thrust = airflow * (1 + f) * nz[1] - airflow * v0 + (nz[2] - flight.pressurePa()) * nz[3] - spill;
        double g3 = Gas.AIR.gamma(tt0), m3 = ch[1];
        double t3 = tt0 / PerfectGas.totalToStaticTemperature(m3, g3), v3 = m3 * Math.sqrt(g3 * Gas.AIR.gasConstant() * t3);
        return new Point(thrust, f * airflow, airflow, recovery, captured, mode, m3, v3, f * d.fuel().stoichiometricAirFuel(), tExit, pt4, nz[1], spill);
    }

    /** Combustor temperature that burns a fuel flow at a flight condition (bisection; the largest if the fuel is more than it takes). */
    public static Point withFuel(Sizing s, Turbojet.Flight flight, double fuelKgPerS) {
        double tt0 = flight.totalTemperature(), lo = tt0, hi = s.design().maxTemperatureK();
        if (fuelKgPerS <= 0) { return at(s, flight, tt0); }
        if (at(s, flight, hi).fuelKgPerS() <= fuelKgPerS) { return at(s, flight, hi); }
        for (int i = 0; i < 60 && hi - lo > 1e-3; i++) {
            double t = 0.5 * (lo + hi);
            if (at(s, flight, t).fuelKgPerS() < fuelKgPerS) { lo = t; } else { hi = t; }
        }
        return at(s, flight, 0.5 * (lo + hi));
    }

    /** Speed behind the inlet's first shock (the spilled stream's), m/s. */
    private static double firstShockSpeed(SupersonicInlet.Design in, Turbojet.Flight flight) {
        double mach = flight.mach(), v0 = flight.speedMps();
        if (in.kind() == SupersonicInlet.Kind.PITOT) {
            NormalShock n = NormalShock.of(mach, 1.4);
            return v0 * n.machBehind() * Math.sqrt(n.temperatureRatio()) / mach;
        }
        try {
            ObliqueShock o = ObliqueShock.of(mach, in.angles()[0], 1.4, false);
            return v0 * o.machBehind() * Math.sqrt(o.temperatureRatio()) / mach;
        } catch (IllegalArgumentException detached) {
            NormalShock n = NormalShock.of(mach, 1.4);
            return v0 * n.machBehind() * Math.sqrt(n.temperatureRatio()) / mach;
        }
    }
}

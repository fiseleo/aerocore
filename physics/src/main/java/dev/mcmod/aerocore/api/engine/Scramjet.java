package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.engine.GasPath.Mix;
import dev.mcmod.aerocore.api.gasdynamics.GasState;
import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import dev.mcmod.aerocore.api.gasdynamics.ObliqueShock;
import dev.mcmod.aerocore.api.gasdynamics.PerfectGas;
import dev.mcmod.aerocore.api.gasdynamics.QuasiOneD;
import dev.mcmod.aerocore.api.gasdynamics.ThermalFlow;
import dev.mcmod.aerocore.api.gasdynamics.Thermo;

import java.util.Objects;

/**
 * A dual-mode scramjet (0.28), one-dimensional and steady, after Heiser and Pratt, Hypersonic Airbreathing
 * Propulsion (AIAA 1994), ch. 4-6 and 8, with thermally perfect gas throughout.
 * <ul>
 * <li>Inlet: external ramps (perfect-gas oblique shocks, shock-on-lip at the design Mach number; slower, the stream
 * between the first shock and the lip spills), then the compression to the isolator's throat area by a kinetic energy
 * efficiency eta_KE: the throat state is the supersonic one that passes the captured flow through the throat area, at
 * the pressure for which expanding it back to ambient would give the free stream's kinetic energy times eta_KE
 * (Heiser and Pratt's definition, with h(T) instead of cp T). It starts when its internal contraction is within
 * Kantrowitz's limit at the Mach number behind the ramps, and once started stays started down to the isentropic limit
 * (the caller keeps whether it was started); unstarted it only spills and drags.</li>
 * <li>Combustor: a diverging duct, fuel added and burned uniformly along it at the mixing-limited efficiency (in eight
 * steps, each with the gas of its fuel-air ratio), wall friction, by QuasiOneD. Scram mode: the flow stays supersonic.
 * When that would choke it thermally, dual mode: a shock train in the isolator slows the flow to the subsonic entry
 * Mach number for which the heat brings it to Mach 1 at the combustor's end (its thermal throat), between a full normal
 * shock and sonic; the isolator holds the pressure rise if it is at least ten duct heights long (proportionally less if
 * shorter; Heiser and Pratt ch. 6), else, or if even behind a normal shock the heat chokes the duct early, the engine
 * unstarts.</li>
 * <li>Nozzle: isentropic expansion to its exit area with a velocity coefficient, separated where it would fall below
 * 0.4 of ambient. Thrust = gross thrust - ram drag - spillage drag.</li>
 * <li>Ignition: the fuel's autoignition delay (Fuel.ignitionDelayS) at the combustor's entry against the residence time
 * there; whether it lights is the part's rule (it reports both).</li>
 * </ul>
 * Not modelled: real-gas dissociation in the combustor (above about 2500 K the temperatures are high), finite-rate
 * chemistry beyond the ignition check, the pseudo-shock's partly supersonic core, fuel injection's own momentum beyond
 * its axial velocity ratio, wall heat loss, unstart dynamics.
 */
public final class Scramjet {
    private Scramjet() {}

    public enum Mode { OFF, SCRAM, DUAL, UNSTART }

    private static final double G = 1.4;
    private static final int STEPS = 8;
    /** Isolator length (in duct heights) that holds a full normal shock's pressure rise. */
    public static final double FULL_ISOLATOR = 10;

    /**
     * A scramjet: capture area (m^2) and design Mach number (shock on lip), the ramps' deflections (radians), the
     * isolator's (throat) area (m^2) and the internal contraction (cowl entrance over throat), the compression's kinetic
     * energy efficiency, the isolator length in duct heights, the combustor's length (m) and exit-over-entry area, its
     * Fanning friction factor, the combustion efficiency, the nozzle's exit over combustor exit area, its velocity
     * coefficient, the fuel's axial injection speed over the stream's, and the fuel.
     */
    public record Design(double captureAreaM2, double designMach, double[] rampAngles, double throatAreaM2, double internalContraction,
                         double kineticEnergyEfficiency, double isolatorLengthOverHeight, double combustorLengthM, double combustorAreaRatio,
                         double frictionFactor, double combustionEfficiency, double nozzleAreaRatio, double nozzleVelocityCoefficient,
                         double injectantVelocityRatio, Fuel fuel) {
        public Design {
            Objects.requireNonNull(fuel, "fuel");
            rampAngles = rampAngles == null ? new double[0] : rampAngles.clone();
            if (!(captureAreaM2 > 0) || !(designMach > 1) || rampAngles.length < 1 || !(throatAreaM2 > 0 && throatAreaM2 < captureAreaM2)
                    || !(internalContraction >= 1) || !(kineticEnergyEfficiency > 0.8 && kineticEnergyEfficiency <= 1) || !(isolatorLengthOverHeight >= 0)
                    || !(combustorLengthM > 0) || !(combustorAreaRatio >= 1) || !(frictionFactor >= 0) || !(combustionEfficiency > 0 && combustionEfficiency <= 1)
                    || !(nozzleAreaRatio >= 1) || !(nozzleVelocityCoefficient > 0.8 && nozzleVelocityCoefficient <= 1)) {
                throw new IllegalArgumentException("Scramjet parameters out of range");
            }
        }
        SupersonicInlet.Design ramps() { return new SupersonicInlet.Design(SupersonicInlet.Kind.RAMPS, designMach, rampAngles, 1); }
    }

    /**
     * An operating point: thrust (N), fuel (kg/s), airflow (kg/s), the mode, whether the inlet is started, captured over
     * capture-area stream, the combustor's entry and exit Mach numbers, its entry static temperature (K) and pressure (Pa),
     * the fuel's ignition delay and the residence time in the combustor (s), the isolator's static pressure rise, the exit
     * speed (m/s), the equivalence ratio.
     */
    public record Point(double thrustN, double fuelKgPerS, double airflowKgPerS, Mode mode, boolean started, double captureRatio, double combustorEntryMach,
                        double combustorExitMach, double combustorEntryK, double combustorEntryPa, double ignitionDelayS, double residenceTimeS,
                        double isolatorPressureRatio, double exitSpeedMps, double equivalenceRatio) {
        public double specificImpulse() { return fuelKgPerS > 0 ? thrustN / (fuelKgPerS * 9.80665) : 0; }
    }

    /**
     * The compression's static pressure ratio p3/p0 for a compressed static temperature T3 and a kinetic energy
     * efficiency: T_x from h(T_x) = h(Tt0) - eta_KE V0^2 / 2 (the state that, expanded back to p0, keeps eta_KE of the
     * free stream's kinetic energy), p3 = p0 exp((phi(T3) - phi(T_x)) / R). For a perfect gas this is Heiser and Pratt's
     * (T3 / (T0 (1 + (gamma - 1)/2 M0^2 (1 - eta_KE))))^(gamma/(gamma - 1)).
     */
    public static double compressionPressureRatio(Thermo gas, GasState free, double t3, double kineticEnergyEfficiency) {
        double ht = gas.enthalpy(free.temperatureK()) + 0.5 * free.speedMps() * free.speedMps();
        double tx = gas.temperatureAtEnthalpy(ht - 0.5 * kineticEnergyEfficiency * free.speedMps() * free.speedMps());
        return Math.exp((gas.entropyFunction(t3) - gas.entropyFunction(tx)) / gas.gasConstant());
    }

    /** The engine at a flight condition and equivalence ratio, lit or not, its inlet started before or not. */
    public static Point at(Design d, Turbojet.Flight flight, double equivalenceRatio, boolean lit, boolean wasStarted) {
        Thermo air = Gas.AIR;
        double mach = flight.mach(), v0 = flight.speedMps(), rho0 = flight.pressurePa() / (air.gasConstant() * flight.temperatureK());
        double full = rho0 * v0 * d.captureAreaM2();
        double phi = Math.max(0, equivalenceRatio);
        if (mach <= 1.05) { return unstarted(d, flight, full, phi); }
        // external ramps
        double m1 = mach;
        for (double a : d.rampAngles()) {
            if (m1 <= 1 || a > ObliqueShock.maxDeflection(m1, G)[0]) { return unstarted(d, flight, full, phi); }
            m1 = ObliqueShock.of(m1, a, G, false).machBehind();
        }
        if (m1 <= 1.05) { return unstarted(d, flight, full, phi); }
        if (!started(d, mach, wasStarted)) { return unstarted(d, flight, full, phi); }
        double capture = SupersonicInlet.captureRatio(d.ramps(), mach), airflow = full * capture;
        // compression to the throat: the supersonic state that passes the airflow through the throat area
        GasState free = new GasState(flight.temperatureK(), flight.pressurePa(), v0);
        GasState s3 = throat(air, free, d.kineticEnergyEfficiency(), airflow / d.throatAreaM2());
        if (s3 == null) { return unstarted(d, flight, full, phi); }
        double fst = 1 / d.fuel().stoichiometricAirFuel(), f = phi * fst;
        double tau = d.fuel().ignitionDelayS(s3.temperatureK(), s3.pressurePa()), residence = d.combustorLengthM() / Math.max(1, s3.speedMps());
        double spill = (full - airflow) * (v0 - firstShockSpeed(d, flight));
        if (!lit || phi <= 0) {
            GasState s4 = march(d, s3, 0, 0, null);
            return finish(d, flight, airflow, 0, Mode.OFF, capture, s3, s4, s3, tau, residence, 1, spill, 0);
        }
        // scram mode: supersonic all the way
        GasState s4 = march(d, s3, f, Math.min(f, fst), null);
        if (s4 != null) { return finish(d, flight, airflow, f, Mode.SCRAM, capture, s3, s4, s3, tau, residence, 1, spill, phi); }
        // dual mode: the subsonic entry that chokes at the combustor's end
        GasState ns = NormalShock.behind(air, s3);
        double mns = ns.mach(air), tt3 = s3.totalTemperature(air), flux = airflow / d.throatAreaM2();
        double lo = mns, hi = 0.999;
        GasState entry = null, exit = null;
        boolean[] choked = new boolean[1];
        if (march(d, entryAt(air, tt3, flux, lo), f, Math.min(f, fst), choked) == null || choked[0]) {
            return unstarted(d, flight, full, phi);   // even behind a normal shock the heat chokes the combustor early
        }
        for (int i = 0; i < 50; i++) {
            double me = 0.5 * (lo + hi);
            GasState e = entryAt(air, tt3, flux, me);
            GasState x = march(d, e, f, Math.min(f, fst), choked);
            if (x == null || choked[0]) { hi = me; } else { lo = me; entry = e; exit = x; }
        }
        if (entry == null) { entry = entryAt(air, tt3, flux, lo); exit = march(d, entry, f, Math.min(f, fst), choked); }
        double rise = entry.pressurePa() / s3.pressurePa();
        double hold = 1 + (ns.pressurePa() / s3.pressurePa() - 1) * Math.min(1, d.isolatorLengthOverHeight() / FULL_ISOLATOR);
        if (rise > hold * (1 + 1e-9)) { return unstarted(d, flight, full, phi); }
        double residenceDual = d.combustorLengthM() / Math.max(1, entry.speedMps());
        return finish(d, flight, airflow, f, Mode.DUAL, capture, entry, exit, s3, d.fuel().ignitionDelayS(entry.temperatureK(), entry.pressurePa()), residenceDual,
                rise, spill, phi);
    }

    /**
     * Whether the inlet is started at a flight Mach number: its internal contraction within Kantrowitz's limit at the Mach
     * number behind the ramps, or (if it was started) within the isentropic limit; false where the ramps detach.
     */
    public static boolean started(Design d, double mach, boolean wasStarted) {
        double m1 = mach;
        for (double a : d.rampAngles()) {
            if (m1 <= 1.05 || a > ObliqueShock.maxDeflection(m1, G)[0]) { return false; }
            m1 = ObliqueShock.of(m1, a, G, false).machBehind();
        }
        if (m1 <= 1.05) { return false; }
        double throatShare = 1 / d.internalContraction();
        return throatShare >= PerfectGas.kantrowitzContraction(m1, G) || wasStarted && throatShare >= PerfectGas.isentropicContraction(m1, G);
    }

    /**
     * The engine tabulated for a part to look up every step (0.28): at fixed flight Mach number and ambient temperature
     * every flow in this model scales with ambient pressure (Reynolds number effects aside: the friction factor is
     * given), so thrust, fuel and airflow are stored per pascal of ambient pressure, with the combustor's entry
     * temperature, its entry pressure per pascal, residence time and entry and exit Mach numbers and the mode, over Mach
     * numbers (0.25 apart), equivalence ratios (0.2 apart, the first row unlit) and three ambient temperatures (216.65,
     * 250, 288.15 K), the inlet taken started. Linear between rows; the mode is the nearest row's.
     */
    public record Map(Design design, double[] mach, double[] phi, double[] temperature, double[][][][] values, Mode[][][] modes) {
        static final int THRUST = 0, FUEL = 1, AIR = 2, ENTRY_K = 3, ENTRY_P = 4, RESIDENCE = 5, ENTRY_MACH = 6, EXIT_MACH = 7, FIELDS = 8;

        public static Map build(Design d, double machMin, double machMax, double maxPhi) {
            int nm = (int) Math.ceil((machMax - machMin) / 0.25) + 1, np = (int) Math.ceil(maxPhi / 0.2) + 1;
            double[] mach = new double[nm], phi = new double[np + 1], temps = {216.65, 250, 288.15};
            for (int i = 0; i < nm; i++) { mach[i] = machMin + 0.25 * i; }
            phi[0] = 0;
            for (int j = 0; j < np; j++) { phi[j + 1] = 0.2 * j; }
            double[][][][] v = new double[FIELDS][temps.length][nm][np + 1];
            Mode[][][] modes = new Mode[temps.length][nm][np + 1];
            for (int k = 0; k < temps.length; k++) {
                for (int i = 0; i < nm; i++) {
                    Turbojet.Flight f = new Turbojet.Flight(temps[k], 101325, mach[i] * Math.sqrt(1.4 * Gas.AIR.gasConstant() * temps[k]));
                    for (int j = 0; j <= np; j++) {
                        Point p = Scramjet.at(d, f, phi[j], j > 0, true);
                        double[] row = {p.thrustN(), p.fuelKgPerS(), p.airflowKgPerS(), p.combustorEntryK(), p.combustorEntryPa(), p.residenceTimeS(),
                                p.combustorEntryMach(), p.combustorExitMach()};
                        for (int q = 0; q < FIELDS; q++) { v[q][k][i][j] = q == ENTRY_K || q == RESIDENCE || q >= ENTRY_MACH ? row[q] : row[q] / 101325; }
                        modes[k][i][j] = p.mode();
                    }
                }
            }
            return new Map(d, mach, phi, temps, v, modes);
        }

        /** The engine at a flight condition and equivalence ratio, lit or not (its inlet started; Scramjet.started tells). */
        public Point at(Turbojet.Flight flight, double equivalenceRatio, boolean lit) {
            double m = Math.clamp(flight.mach(), mach[0], mach[mach.length - 1]);
            double xm = (m - mach[0]) / 0.25, xt;
            int i = Math.min((int) xm, mach.length - 2);
            double wm = xm - i;
            double t = Math.clamp(flight.temperatureK(), temperature[0], temperature[temperature.length - 1]);
            int k = t <= temperature[1] ? 0 : 1;
            xt = (t - temperature[k]) / (temperature[k + 1] - temperature[k]);
            double ph = lit ? Math.clamp(equivalenceRatio, 0, phi[phi.length - 1]) : 0;
            int j;
            double wp;
            if (!lit) { j = 0; wp = 0; }
            else {
                double xp = ph / 0.2;
                j = 1 + Math.min((int) xp, phi.length - 3);
                wp = xp - (j - 1);
            }
            double[] r = new double[FIELDS];
            for (int q = 0; q < FIELDS; q++) {
                double[][][] f = values[q];
                double a = lerp(f[k][i][j], f[k][i + 1][j], wm), b = lerp(f[k + 1][i][j], f[k + 1][i + 1][j], wm);
                double lo = lerp(a, b, xt);
                if (lit) {
                    double a2 = lerp(f[k][i][j + 1], f[k][i + 1][j + 1], wm), b2 = lerp(f[k + 1][i][j + 1], f[k + 1][i + 1][j + 1], wm);
                    lo = lerp(lo, lerp(a2, b2, xt), wp);
                }
                r[q] = lo;
            }
            double p0 = flight.pressurePa();
            Mode mode = modes[xt < 0.5 ? k : k + 1][wm < 0.5 ? i : i + 1][lit && wp >= 0.5 ? j + 1 : j];
            double entryP = r[ENTRY_P] * p0;
            double tau = mode == Mode.UNSTART || !(r[ENTRY_K] > 0) || !(entryP > 0) ? Double.POSITIVE_INFINITY : design.fuel().ignitionDelayS(r[ENTRY_K], entryP);
            return new Point(r[THRUST] * p0, r[FUEL] * p0, r[AIR] * p0, mode, mode != Mode.UNSTART, Double.NaN, r[ENTRY_MACH], r[EXIT_MACH], r[ENTRY_K], entryP, tau,
                    r[RESIDENCE], Double.NaN, Double.NaN, lit ? ph : 0);
        }

        private static double lerp(double a, double b, double w) {
            if (Double.isNaN(a)) { return b; }
            if (Double.isNaN(b)) { return a; }
            return a + (b - a) * w;
        }
    }

    /** The supersonic throat state by eta_KE that carries a mass flux; null if the flux is more than sonic flow can carry (choked). */
    static GasState throat(Thermo air, GasState free, double etaKE, double flux) {
        double ht = air.enthalpy(free.temperatureK()) + 0.5 * free.speedMps() * free.speedMps();
        java.util.function.DoubleFunction<GasState> at = t3 -> {
            double v = Math.sqrt(Math.max(0, 2 * (ht - air.enthalpy(t3))));
            return new GasState(t3, free.pressurePa() * compressionPressureRatio(air, free, t3, etaKE), Math.max(1e-6, v));
        };
        // the sonic temperature: Mach 1 where 2 (ht - h(T)) = gamma R T
        double lo = free.temperatureK(), hi = air.temperatureAtEnthalpy(ht);
        for (int i = 0; i < 100 && hi - lo > 1e-9 * hi; i++) {
            double t = 0.5 * (lo + hi);
            if (at.apply(t).mach(air) > 1) { lo = t; } else { hi = t; }
        }
        double tSonic = 0.5 * (lo + hi);
        if (flux > at.apply(tSonic).massFlux(air)) { return null; }
        lo = free.temperatureK();
        hi = tSonic;
        // on the supersonic side the flux rises with compression up to sonic
        for (int i = 0; i < 100 && hi - lo > 1e-10 * hi; i++) {
            double t = 0.5 * (lo + hi);
            if (at.apply(t).massFlux(air) < flux) { lo = t; } else { hi = t; }
        }
        return at.apply(0.5 * (lo + hi));
    }

    /** A subsonic combustor entry at Mach m carrying a mass flux, air at total temperature tt. */
    private static GasState entryAt(Thermo air, double tt, double flux, double m) {
        GasState shape = ThermalFlow.atMach(air, tt, 1e5, m);
        double p = flux * air.gasConstant() * shape.temperatureK() / shape.speedMps();
        return new GasState(shape.temperatureK(), p, shape.speedMps());
    }

    /**
     * The combustor from its entry: fuel f (per kg of air) added and heat from fBurned released uniformly, in steps;
     * null if the flow chokes (choked[0] set) before the end; supersonic or subsonic as it enters.
     */
    private static GasState march(Design d, GasState entry, double f, double fBurned, boolean[] choked) {
        if (choked != null) { choked[0] = false; }
        double length = d.combustorLengthM() / STEPS, a3 = d.throatAreaM2(), a4 = a3 * d.combustorAreaRatio();
        double eta = d.combustionEfficiency(), lhv = d.fuel().lowerHeatingValueJPerKg();
        GasState s = entry;
        for (int i = 0; i < STEPS; i++) {
            double fBefore = f * i / STEPS, fAfter = f * (i + 1) / STEPS, fMid = Math.min(fBurned, f * (i + 0.5) / STEPS);
            Thermo gas = fMid > 0 ? new Mix(d.fuel(), fMid) : Gas.AIR;
            double heat = eta * fBurned / STEPS * lhv / (1 + fBefore);   // J per kg of the stream entering this step
            double added = (fAfter - fBefore) / (1 + fBefore);
            QuasiOneD.Segment seg = new QuasiOneD.Segment(length, a3 + (a4 - a3) * i / STEPS, a3 + (a4 - a3) * (i + 1) / STEPS, heat, d.frictionFactor(),
                    Double.NaN, added, d.injectantVelocityRatio());
            QuasiOneD.Result r = QuasiOneD.march(gas, seg, s, 40);
            if (r.choked()) {
                if (choked != null) { choked[0] = true; }
                return null;
            }
            s = r.exit();
        }
        return s;
    }

    private static Point finish(Design d, Turbojet.Flight flight, double airflow, double f, Mode mode, double capture, GasState entry, GasState exit,
                                GasState s3, double tau, double residence, double rise, double spill, double phi) {
        Thermo gas = f > 0 ? new Mix(d.fuel(), Math.min(f, 1 / d.fuel().stoichiometricAirFuel())) : Gas.AIR;
        double flow = airflow * (1 + f), tt = exit.totalTemperature(gas), pt = exit.totalPressure(gas);
        double a9 = d.throatAreaM2() * d.combustorAreaRatio() * d.nozzleAreaRatio(), p0 = flight.pressurePa();
        GasState e9 = ThermalFlow.atMassFlux(gas, tt, pt, flow / a9, true);
        double area = a9;
        if (e9.pressurePa() < GasPath.SEPARATION * p0 && pt > p0) {
            e9 = ThermalFlow.expand(gas, tt, pt, GasPath.SEPARATION * p0);
            area = flow / e9.massFlux(gas);
        }
        double v9 = d.nozzleVelocityCoefficient() * e9.speedMps();
        double thrust = flow * v9 - airflow * flight.speedMps() + (e9.pressurePa() - p0) * area - spill;
        return new Point(thrust, f * airflow, airflow, mode, true, capture, entry.mach(Gas.AIR), exit.mach(gas), entry.temperatureK(), entry.pressurePa(), tau,
                residence, rise, v9, phi);
    }

    /** An unstarted (or subsonic) inlet: the captured stream through a normal shock and spilled; no combustion. */
    private static Point unstarted(Design d, Turbojet.Flight flight, double full, double phi) {
        double mach = flight.mach(), v0 = flight.speedMps();
        double behind = mach > 1 ? v0 * NormalShock.of(mach, G).machBehind() * Math.sqrt(NormalShock.of(mach, G).temperatureRatio()) / mach : 0.5 * v0;
        return new Point(-full * (v0 - behind), 0, 0, Mode.UNSTART, false, 0, Double.NaN, Double.NaN, flight.temperatureK(), flight.pressurePa(),
                Double.POSITIVE_INFINITY, 0, 1, 0, phi);
    }

    private static double firstShockSpeed(Design d, Turbojet.Flight flight) {
        double mach = flight.mach(), v0 = flight.speedMps();
        ObliqueShock o = ObliqueShock.of(mach, d.rampAngles()[0], G, false);
        return v0 * o.machBehind() * Math.sqrt(o.temperatureRatio()) / mach;
    }
}

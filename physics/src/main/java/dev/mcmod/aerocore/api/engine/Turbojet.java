package dev.mcmod.aerocore.api.engine;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Single-spool turbojet: inlet, compressor, combustor, turbine and a fixed nozzle, with variable gas properties (air
 * and kerosene products from the GRI-Mech polynomials of Species, mixed by mass; lean mixtures only).
 * <ul>
 * <li>Design point (size): the compressor's pressure ratio and airflow, the turbine inlet temperature and the
 *   component efficiencies give the cycle; the turbine's flow area and the exhaust nozzle throat (choked, or for low
 *   pressure ratios exhausting subsonically at ambient) are sized from it. The turbine passes flow as a nozzle across
 *   its pressure ratio: choked above about 1.85 (its guide vanes choke; a single stage chokes somewhat later), less
 *   below (a nozzle form of Stodola's cone law), so low pressure ratio engines and low power are matched too.</li>
 * <li>Off design (equilibrium): with fixed flow areas, the turbine's flow, the nozzle's flow and the spool's
 *   power balance fix the operating point for a turbine inlet temperature; constant polytropic efficiencies, no
 *   component maps. This is the matching of single-spool turbojets taught from Mattingly (Elements of Propulsion,
 *   ch. 8) and Seitzman (Georgia Tech AE4803, "Engine off-design performance"); the turbine pressure ratio is solved
 *   from the nozzle flow, so an unchoked nozzle at low power is handled. Rotor speed scales with the square root of the
 *   compressor work (Euler's equation at constant velocity triangles).</li>
 * <li>Transient: the compressor characteristic the equilibrium implies (speed fixes the work and the pressure ratio,
 *   the turbine the flow) at the spool's actual speed; the burner turns the actual fuel flow into the turbine inlet
 *   temperature. The turbine's flow at the running line's pressure ratio scales with its flow parameter
 *   (flux(T4) / (1 + f), about 1 / sqrt(T4)), so hotter gas than the running line's passes less flow at the same
 *   pressure ratio, towards surge (the pressure ratio over the running line's at that flow is the surge margin used);
 *   colder gas is held to the running line's flow (the compressor chokes). The turbine and nozzle match the gas, and
 *   turbine power less compressor power accelerates the spool.</li>
 * </ul>
 * Polytropic relations with variable cp: compression ln pi = e (phi(T3) - phi(T2)) / R, expansion
 * ln pi = (phi(T4) - phi(T5)) / (e R), phi the entropy function. The inlet duct's, burner's and tailpipe's
 * total-pressure losses are given at the design point and scale off design with the dynamic head, i.e. with the square
 * of the corrected flow entering them (Saravanamuttoo et al., Gas Turbine Theory, pressure-loss factor; the burner's
 * hot share is held at its design fraction): a slowly turning spool loses almost nothing, so it can light and run up.
 * Cooling air (a fraction of compressor delivery) bypasses burner and turbine and rejoins before the nozzle. Supersonic inlets: a pitot inlet loses a normal shock;
 * an external-compression ramp inlet follows MIL-E-5008B, 1 - 0.075 (M - 1)^1.35 (as given by Mattingly).
 * A convergent-divergent nozzle separates when its exit would be below 0.4 of ambient (Summerfield's criterion) and
 * then exhausts at ambient.
 */
public final class Turbojet {
    private Turbojet() {}

    /** Standard sea-level reference for corrected quantities. */
    public static final double T_REF = 288.15, P_REF = 101325;
    /** Lowest corrected speed (fraction of design) the running line table holds. */
    public static final double MIN_LINE_SPEED = 0.35;
    private static final double SEPARATION = 0.4;

    public enum Inlet { PITOT, RAMP }

    /** A cycle: compressor pressure ratio and design airflow (kg/s at the design condition), efficiencies, turbine inlet temperature, losses, nozzle. */
    public record Design(double pressureRatio, double airflowKgPerS, double compressorEfficiency, double turbineInletK, double burnerPressureRatio,
                         double burnerEfficiency, double turbineEfficiency, double mechanicalEfficiency, double coolingBleed, double nozzlePressureRatio,
                         boolean convergentDivergent, double exitAreaRatio, double inletRecovery, Inlet inlet, Fuel fuel) {
        public Design {
            Objects.requireNonNull(inlet, "inlet"); Objects.requireNonNull(fuel, "fuel");
            if (!(pressureRatio > 1) || !(airflowKgPerS > 0) || !(turbineInletK > 0)) { throw new IllegalArgumentException("Pressure ratio, airflow and turbine temperature must be positive"); }
            for (double e : new double[]{compressorEfficiency, burnerPressureRatio, burnerEfficiency, turbineEfficiency, mechanicalEfficiency, nozzlePressureRatio, inletRecovery}) {
                if (!(e > 0 && e <= 1)) { throw new IllegalArgumentException("Efficiencies and pressure ratios of losses must be in (0, 1]"); }
            }
            if (!(coolingBleed >= 0 && coolingBleed < 0.5)) { throw new IllegalArgumentException("Cooling bleed must be in [0, 0.5)"); }
            if (convergentDivergent && !(exitAreaRatio >= 1)) { throw new IllegalArgumentException("A convergent-divergent nozzle needs an exit area ratio of at least 1"); }
        }
    }

    /** Ambient static temperature (K), static pressure (Pa) and flight speed (m/s, into the inlet). */
    public record Flight(double temperatureK, double pressurePa, double speedMps) {
        public Flight {
            if (!(temperatureK > 0) || !(pressurePa > 0) || !(speedMps >= 0)) { throw new IllegalArgumentException("Flight condition must be positive"); }
        }
        public static Flight sea(double speedMps) { return new Flight(T_REF, P_REF, speedMps); }
        public double mach() { return speedMps / Math.sqrt(1.4 * Gas.AIR.gasConstant() * temperatureK); }
        /** Total temperature of the free stream, K. */
        public double totalTemperature() { return Gas.AIR.temperatureAtEnthalpy(Gas.AIR.enthalpy(temperatureK) + speedMps * speedMps / 2); }
        /** Total pressure of the free stream, Pa. */
        public double totalPressure() {
            double tt = totalTemperature();
            return pressurePa * Math.exp((Gas.AIR.entropyFunction(tt) - Gas.AIR.entropyFunction(temperatureK)) / Gas.AIR.gasConstant());
        }
    }

    /** Inlet total pressure recovery at a flight Mach number, at the design corrected airflow. */
    public static double inletRecovery(Design d, double mach) { return inletRecovery(d, mach, 1); }
    /** Inlet total pressure recovery at a flight Mach number and a corrected airflow (fraction of design): the duct loses with the square of it. */
    public static double inletRecovery(Design d, double mach, double flowFraction) {
        double r = retained(d.inletRecovery(), flowFraction);
        if (mach <= 1) { return r; }
        if (d.inlet() == Inlet.RAMP) { return r * Math.max(0.05, 1 - 0.075 * Math.pow(mach - 1, 1.35)); }
        double g = 1.4, m2 = mach * mach;
        double shock = Math.pow((g + 1) * m2 / ((g - 1) * m2 + 2), g / (g - 1)) * Math.pow((g + 1) / (2 * g * m2 - (g - 1)), 1 / (g - 1));
        return r * shock;
    }

    /** Gas that is a mass blend of air and stoichiometric products: lean products of a fuel-air ratio f. */
    static final class Mix {
        final double share, r;   // mass fraction of stoichiometric products
        final Gas products;
        Mix(Fuel fuel, double fuelAirRatio) {
            double afr = fuel.stoichiometricAirFuel();
            share = Math.clamp(fuelAirRatio * (1 + afr) / (1 + fuelAirRatio), 0, 1);
            products = stoichiometric(fuel);
            r = share * products.gasConstant() + (1 - share) * Gas.AIR.gasConstant();
        }
        double h(double t) { return share * products.enthalpy(t) + (1 - share) * Gas.AIR.enthalpy(t); }
        double phi(double t) { return share * products.entropyFunction(t) + (1 - share) * Gas.AIR.entropyFunction(t); }
        double cp(double t) { return share * products.cp(t) + (1 - share) * Gas.AIR.cp(t); }
        double tAtH(double h) {
            double t = 1000;
            for (int i = 0; i < 40; i++) { double step = (h(t) - h) / cp(t); t -= step; if (Math.abs(step) < 1e-7) { break; } }
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
        /** Static state where the flow is sonic, from total temperature: {T*, p* / pt, mass flux / pt (s/m)}. */
        double[] sonic(double tt) {
            double lo = 0.5 * tt, hi = tt, ht = h(tt);
            for (int i = 0; i < 50; i++) {
                double t = 0.5 * (lo + hi), v2 = 2 * (ht - h(t)), cp = cp(t), a2 = cp / (cp - r) * r * t;
                if (v2 > a2) { lo = t; } else { hi = t; }
            }
            double t = 0.5 * (lo + hi), pr = Math.exp(-(phi(tt) - phi(t)) / r), v = Math.sqrt(Math.max(0, 2 * (ht - h(t))));
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
    }

    /** Pressure retained by a duct that keeps r at design, at a corrected flow (fraction of design): the loss goes with the dynamic head. */
    static double retained(double designRetention, double flowFraction) {
        return Math.max(0.3, 1 - (1 - designRetention) * flowFraction * flowFraction);
    }
    /** Flow corrected to the reference state, kg/s: w sqrt(T / T_REF) / (p / P_REF). */
    static double corrected(double flow, double tt, double pt) { return flow * Math.sqrt(tt / T_REF) * P_REF / pt; }

    private static final java.util.Map<Fuel, Gas> STOICHIOMETRIC = new java.util.concurrent.ConcurrentHashMap<>();
    static Gas stoichiometric(Fuel fuel) { return STOICHIOMETRIC.computeIfAbsent(fuel, f -> f.products(1)); }

    /** Fuel-air ratio (per kilogram of burner air) that heats air from T3 to T4 in a burner of efficiency eta (fuel at 298 K). */
    public static double fuelAirRatio(Fuel fuel, double t3, double t4, double eta) {
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

    /** One operating point. Temperatures and pressures are totals at stations 2 (compressor face), 3, 4, 5 (after cooling air rejoins). */
    public record Point(double thrustN, double fuelKgPerS, double airflowKgPerS, double pressureRatio, double tt2, double pt2, double tt3, double tt4,
                        double pt4, double tt5, double pt5, double turbinePressureRatio, double exitSpeedMps, double exitPressurePa, double exitAreaM2,
                        boolean nozzleChoked, double compressorWorkJPerKg, double turbinePowerW, double compressorPowerW, double speedFraction,
                        double surgeMarginUsed) {
        public double specificThrust() { return thrustN / airflowKgPerS; }
        /** Thrust-specific fuel consumption, kg/(N s). */
        public double tsfc() { return fuelKgPerS / thrustN; }
    }

    /**
     * A sized engine: its design, the flow areas (the turbine's, as a nozzle across its pressure ratio, and the exhaust
     * nozzle's throat, m^2) and the design point's values the off-design relations refer to (the
     * compressor work, and the corrected flows into the compressor face, the burner and the tailpipe, kg/s, that set
     * the losses).
     */
    public record Sizing(Design design, Flight condition, double turbineThroatM2, double nozzleThroatM2, double designWorkJPerKg, Point point,
                         double faceFlow, double burnerFlow, double tailpipeFlow) {}

    /** Sizes the engine at a design flight condition. */
    public static Sizing size(Design d, Flight flight) {
        double tt2 = flight.totalTemperature(), pt2 = flight.totalPressure() * inletRecovery(d, flight.mach());
        Gas air = Gas.AIR;
        double t3 = air.temperatureAtEntropy(air.entropyFunction(tt2) + air.gasConstant() * Math.log(d.pressureRatio()) / d.compressorEfficiency());
        double wc = air.enthalpy(t3) - air.enthalpy(tt2);
        double c = d.coolingBleed(), f = fuelAirRatio(d.fuel(), t3, d.turbineInletK(), d.burnerEfficiency());
        double pt4 = d.burnerPressureRatio() * d.pressureRatio() * pt2;
        Mix hot = new Mix(d.fuel(), f);
        double flow4 = d.airflowKgPerS() * (1 - c) * (1 + f);
        // turbine work to drive the compressor, then the pressure ratio by the polytropic relation
        double wt = wc / (d.mechanicalEfficiency() * (1 - c) * (1 + f));
        double t5 = hot.tAtH(hot.h(d.turbineInletK()) - wt);
        double lnPt = (hot.phi(d.turbineInletK()) - hot.phi(t5)) / (d.turbineEfficiency() * hot.r);
        double a4 = flow4 / (hot.flux(d.turbineInletK(), Math.exp(lnPt)) * pt4);
        double pt5 = pt4 * Math.exp(-lnPt);
        double fo = f * (1 - c);
        Mix out = new Mix(d.fuel(), fo);
        double tt5 = out.tAtH(((1 - c) * (1 + f) * hot.h(t5) + c * air.enthalpy(t3)) / (1 + fo));
        double pt9 = d.nozzlePressureRatio() * pt5;
        double flow9 = d.airflowKgPerS() * (1 + fo);
        if (pt9 <= flight.pressurePa() * 1.0001) {
            throw new IllegalArgumentException("The cycle does not close: the turbine leaves no pressure to drive the exhaust out");
        }
        double[] star = out.sonic(tt5);
        double a8;
        if (star[1] * pt9 > flight.pressurePa()) { a8 = flow9 / (star[2] * pt9); }
        else {   // low pressure ratio engines (model turbojets) exhaust subsonically even at design
            double[] e = out.expand(tt5, pt9, flight.pressurePa());
            a8 = flow9 / (flight.pressurePa() / (out.r * e[0]) * e[1]);
        }
        double[] n = nozzle(out, tt5, pt9, flight.pressurePa(), a8, d.convergentDivergent(), d.exitAreaRatio());
        double thrust = flow9 * n[1] - d.airflowKgPerS() * flight.speedMps() + (n[2] - flight.pressurePa()) * n[3];
        Point p = new Point(thrust, f * (1 - c) * d.airflowKgPerS(), d.airflowKgPerS(), d.pressureRatio(), tt2, pt2, t3, d.turbineInletK(), pt4, tt5, pt5,
                Math.exp(lnPt), n[1], n[2], n[3], n[4] > 0.5, wc, d.mechanicalEfficiency() * flow4 * wt, d.airflowKgPerS() * wc, 1, 0);
        return new Sizing(d, flight, a4, a8, wc, p, corrected(d.airflowKgPerS(), tt2, pt2), corrected(d.airflowKgPerS(), t3, d.pressureRatio() * pt2),
                corrected(flow9, tt5, pt5));
    }

    /**
     * Equilibrium operating point of a sized engine at a flight condition and turbine inlet temperature (K), from the
     * inlet's total state (pt2 may carry a different recovery than the design's: give it, or NaN to use the design's
     * inlet model).
     */
    public static Point equilibrium(Sizing s, Flight flight, double turbineInletK, double pt2Given) {
        Design d = s.design();
        double tt2 = flight.totalTemperature();
        Gas air = Gas.AIR;
        double c = d.coolingBleed(), t4 = turbineInletK;
        double[] f = {fuelAirRatio(d.fuel(), tt2 + 10, t4, d.burnerEfficiency())};
        java.util.function.DoubleFunction<double[]> state = lnPt -> {
            // given the turbine pressure ratio: turbine work, compressor work and pressure ratio, flows, nozzle
            double[] out = null;
            for (int k = 0; k < 6; k++) {
                Mix hot = new Mix(d.fuel(), f[0]);
                double t5 = hot.tAtPhi(hot.phi(t4) - d.turbineEfficiency() * hot.r * lnPt);
                double wt = hot.h(t4) - hot.h(t5);
                double wc = d.mechanicalEfficiency() * (1 - c) * (1 + f[0]) * wt;
                double t3 = air.temperatureAtEnthalpy(air.enthalpy(tt2) + wc);
                double fNew = fuelAirRatio(d.fuel(), t3, t4, d.burnerEfficiency());
                double pc = Math.exp(d.compressorEfficiency() * (air.entropyFunction(t3) - air.entropyFunction(tt2)) / air.gasConstant());
                // burner loss L (w3c / w3c,D)^2 where w3c = q pi_b through the turbine: L q^2 pi^2 + pi - 1 = 0
                double flux4 = hot.flux(t4, Math.exp(lnPt));
                double q = corrected(s.turbineThroatM2() * flux4 / ((1 - c) * (1 + fNew)), t3, 1) / s.burnerFlow();
                double lq = (1 - d.burnerPressureRatio()) * q * q;
                double pb = Math.max(0.3, 2 / (1 + Math.sqrt(1 + 4 * lq)));
                // the corrected flow at the face does not depend on pt2 (it cancels), so the inlet's loss follows from it
                double face = corrected(s.turbineThroatM2() * flux4 * pb * pc / ((1 - c) * (1 + fNew)), tt2, 1);
                double pt2 = Double.isNaN(pt2Given) ? flight.totalPressure() * inletRecovery(d, flight.mach(), face / s.faceFlow()) : pt2Given;
                double pt4 = pb * pc * pt2;
                double flow4 = s.turbineThroatM2() * flux4 * pt4;
                double airflow = flow4 / ((1 - c) * (1 + fNew));
                out = new double[]{t5, wt, wc, t3, pc, pt4, airflow, fNew, pt2};
                boolean done = Math.abs(fNew - f[0]) < 1e-9;
                f[0] = fNew;
                if (done) { break; }
            }
            return out;
        };
        // solve the turbine pressure ratio for which the nozzle passes the flow (positive: the nozzle passes more than
        // delivered, so the turbine takes more pressure)
        java.util.function.DoubleUnaryOperator mismatch = x -> {
            double[] st = state.apply(x);
            double fo = st[7] * (1 - c);
            Mix out = new Mix(d.fuel(), fo);
            Mix hot = new Mix(d.fuel(), st[7]);
            double tt5 = out.tAtH(((1 - c) * (1 + st[7]) * hot.h(st[0]) + c * air.enthalpy(st[3])) / (1 + fo));
            double pt5 = st[5] * Math.exp(-x), need = st[6] * (1 + fo);
            double pt9 = retained(d.nozzlePressureRatio(), corrected(need, tt5, pt5) / s.tailpipeFlow()) * pt5;
            return nozzle(out, tt5, pt9, flight.pressurePa(), s.nozzleThroatM2(), d.convergentDivergent(), d.exitAreaRatio())[0] - need;
        };
        // It has two roots: a degenerate one near no turbine pressure ratio (a spool barely turning with full flow) and
        // the physical one above it. Scan down from the largest ratio to bracket the upper root, then bisect.
        double hi = Math.log(40), lo = 0.7 * hi;
        while (lo > 1e-4 && mismatch.applyAsDouble(lo) <= 0) { hi = lo; lo *= 0.7; }
        lo = Math.max(lo, 1e-4);
        for (int it = 0; it < 60 && hi - lo > 1e-10; it++) {
            double x = 0.5 * (lo + hi);
            if (mismatch.applyAsDouble(x) > 0) { lo = x; } else { hi = x; }
        }
        double x = 0.5 * (lo + hi);
        double[] st = state.apply(x), nz;
        double fb = st[7], fo = fb * (1 - c), airflow = st[6];
        Mix out = new Mix(d.fuel(), fo), hot = new Mix(d.fuel(), fb);
        double tt5 = out.tAtH(((1 - c) * (1 + fb) * hot.h(st[0]) + c * air.enthalpy(st[3])) / (1 + fo));
        double pt5 = st[5] * Math.exp(-x);
        double pt9 = retained(d.nozzlePressureRatio(), corrected(airflow * (1 + fo), tt5, pt5) / s.tailpipeFlow()) * pt5;
        nz = nozzle(out, tt5, pt9, flight.pressurePa(), s.nozzleThroatM2(), d.convergentDivergent(), d.exitAreaRatio());
        double thrust = airflow * (1 + fo) * nz[1] - airflow * flight.speedMps() + (nz[2] - flight.pressurePa()) * nz[3];
        double speed = Math.sqrt(Math.max(0, st[2] / s.designWorkJPerKg()));
        return new Point(thrust, fo * airflow, airflow, st[4], tt2, st[8], st[3], t4, st[5], tt5, pt5, Math.exp(x), nz[1], nz[2], nz[3], nz[4] > 0.5,
                st[2], d.mechanicalEfficiency() * airflow * (1 - c) * (1 + fb) * st[1], airflow * st[2], speed, 0);
    }

    /**
     * The equilibrium running line at one flight Mach number, in corrected terms (speed fraction over sqrt(theta2),
     * airflow sqrt(theta2)/delta2, fuel flow /(delta2 sqrt(theta2)), pressure ratio), from low to full power.
     */
    public record RunningLine(double[] speed, double[] pressureRatio, double[] airflow, double[] fuel, double[] turbineInletK) {
        int index(double n) {
            int i = 0;
            while (i < speed.length - 2 && n > speed[i + 1]) { i++; }
            return i;
        }
        double interpolate(double[] y, double n) {
            int i = index(n);
            double t = (n - speed[i]) / (speed[i + 1] - speed[i]);
            return y[i] + Math.clamp(t, n < speed[0] ? -10 : 0, n > speed[speed.length - 1] ? 10 : 1) * (y[i + 1] - y[i]);
        }
        /** Pressure ratio at a corrected speed fraction (below the table: pressure rise with speed squared, the affinity law). */
        public double pressureRatioAt(double n) {
            if (n < speed[0]) { double r = n / speed[0]; return 1 + (pressureRatio[0] - 1) * r * r; }
            return Math.max(1, interpolate(pressureRatio, n));
        }
        /** Corrected airflow at a corrected speed fraction (below the table: proportional to speed). */
        public double airflowAt(double n) {
            if (n < speed[0]) { return airflow[0] * Math.max(0, n) / speed[0]; }
            return Math.max(0, interpolate(airflow, n));
        }
        /** Corrected fuel flow on the running line at a corrected speed fraction. */
        public double fuelAt(double n) {
            if (n < speed[0]) { return fuel[0] * Math.max(0, n) / speed[0]; }
            return Math.max(0, interpolate(fuel, n));
        }
        /** Corrected speed at which the running line has a pressure ratio (inverse of pressureRatioAt). */
        public double speedAtPressureRatio(double pr) {
            if (pr < pressureRatio[0]) { return speed[0] * Math.sqrt(Math.max(0, pr - 1) / (pressureRatio[0] - 1)); }
            return inverse(pressureRatio, pr);
        }
        /** Corrected speed at which the running line passes a corrected airflow (inverse of airflowAt). */
        public double speedAtAirflow(double w) {
            if (w < airflow[0]) { return speed[0] * Math.max(0, w) / airflow[0]; }
            return inverse(airflow, w);
        }
        private double inverse(double[] y, double v) {
            int i = 0;
            while (i < y.length - 2 && v > y[i + 1]) { i++; }
            double t = (v - y[i]) / (y[i + 1] - y[i]);
            return speed[i] + Math.min(t, 10) * (speed[i + 1] - speed[i]);
        }
    }

    /** Flight Mach numbers the running lines are built at, and turbine temperatures per line. */
    static final double[] LINE_MACH = {0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0};
    static final int LINE_POINTS = 100;
    /** Coldest standard ambient temperature (ISA tropopause), K. */
    static final double COLD_AMBIENT = 216.65;

    /**
     * Running lines at flight Mach numbers from static up (LINE_MACH, as far as the engine has one). In corrected terms
     * a fixed engine's operating point depends on its corrected speed and the flight Mach number only; with a choked
     * nozzle hardly on the latter, but an unchoked nozzle (low power, low pressure ratio engines) gains pressure ratio
     * from ram. Between two Mach numbers values are interpolated linearly in the ram pressure ratio; above the last the
     * last is used.
     */
    public record RunningLines(double[] mach, RunningLine[] lines) {
        /** The running line at a flight Mach number. */
        public LineAt at(double flightMach) {
            int i = 0;
            while (i < mach.length - 2 && flightMach > mach[i + 1]) { i++; }
            if (mach.length == 1) { return new LineAt(lines[0], lines[0], 0); }
            double r = ram(flightMach), lo = ram(mach[i]), hi = ram(mach[i + 1]);
            return new LineAt(lines[i], lines[i + 1], Math.clamp((r - lo) / (hi - lo), 0, 1));
        }
        /** Free-stream total over static pressure (gamma 1.4): flight speed acts on the cycle through it. */
        private static double ram(double m) { return Math.pow(1 + 0.2 * m * m, 3.5); }
    }

    /** The running line at a flight Mach number: a blend (weight w on b) of the two nearest lines. */
    public record LineAt(RunningLine a, RunningLine b, double w) {
        public double pressureRatioAt(double n) { return (1 - w) * a.pressureRatioAt(n) + w * b.pressureRatioAt(n); }
        public double airflowAt(double n) { return (1 - w) * a.airflowAt(n) + w * b.airflowAt(n); }
        public double fuelAt(double n) { return (1 - w) * a.fuelAt(n) + w * b.fuelAt(n); }
        public double firstSpeed() { return (1 - w) * a.speed()[0] + w * b.speed()[0]; }
        public double speedAtPressureRatio(double pr) {
            return invert(this::pressureRatioAt, pr, (1 - w) * a.speedAtPressureRatio(pr) + w * b.speedAtPressureRatio(pr));
        }
        public double speedAtAirflow(double flow) { return invert(this::airflowAt, flow, (1 - w) * a.speedAtAirflow(flow) + w * b.speedAtAirflow(flow)); }
        /** The inverse of the blend (secant steps from the blend of the inverses; exact on a single line). */
        private double invert(java.util.function.DoubleUnaryOperator f, double y, double guess) {
            if (w == 0 || w == 1 || a == b) { return guess; }
            double x0 = guess, f0 = f.applyAsDouble(x0) - y, x1 = guess * 1.001 + 1e-6, f1 = f.applyAsDouble(x1) - y;
            for (int i = 0; i < 30 && Math.abs(f1) > 1e-12 * Math.max(1, Math.abs(y)) && f1 != f0; i++) {
                double x2 = Math.max(0, x1 - f1 * (x1 - x0) / (f1 - f0));
                x0 = x1; f0 = f1;
                x1 = x2; f1 = f.applyAsDouble(x1) - y;
            }
            return x1;
        }
    }

    /** The running line of a sized engine at its design condition (a sweep of turbine inlet temperatures). */
    public static RunningLine runningLine(Sizing s) { return runningLine(s, s.condition()); }

    /**
     * The running line of a sized engine at a flight condition: from 1.15 Tt2 (as cold as a spool held by ram turns) up to 1.08 times the design turbine inlet
     * temperature as it would be in corrected terms in the coldest standard air (COLD_AMBIENT), where the corrected speed
     * at full power is highest.
     */
    public static RunningLine runningLine(Sizing s, Flight f) {
        double theta = f.totalTemperature() / T_REF;
        List<double[]> rows = new ArrayList<>();
        double t4max = s.design().turbineInletK() * 1.08 * Math.max(1, f.temperatureK() / COLD_AMBIENT), t4min = f.totalTemperature() * 1.15;
        for (int i = 0; i <= LINE_POINTS && t4min < t4max; i++) {
            double t4 = t4min + (t4max - t4min) * i / LINE_POINTS;
            Point p = equilibrium(s, f, t4, Double.NaN);
            double delta = p.pt2() / P_REF;
            double n = p.speedFraction() / Math.sqrt(theta);
            // The table starts at MIN_LINE_SPEED or where the spool can hold itself up, whichever is faster; the affinity laws carry on below
            // (constant component efficiencies do not describe a compressor far below its design speed).
            if (n < MIN_LINE_SPEED || !rows.isEmpty() && n <= rows.getLast()[0] + 1e-9) { continue; }
            rows.add(new double[]{n, p.pressureRatio(), p.airflowKgPerS() * Math.sqrt(theta) / delta, p.fuelKgPerS() / (delta * Math.sqrt(theta)), t4});
        }
        if (rows.size() < 2) { throw new IllegalArgumentException("No running line: the engine cannot hold itself up at any turbine temperature"); }
        double[][] cols = new double[5][rows.size()];
        for (int i = 0; i < rows.size(); i++) for (int k = 0; k < 5; k++) { cols[k][i] = rows.get(i)[k]; }
        return new RunningLine(cols[0], cols[1], cols[2], cols[3], cols[4]);
    }

    /** Running lines at LINE_MACH (at the design condition's ambient state), up to the last Mach number with a line. */
    public static RunningLines runningLines(Sizing s) {
        Flight c = s.condition();
        double sound = Math.sqrt(1.4 * Gas.AIR.gasConstant() * c.temperatureK());
        List<Double> machs = new ArrayList<>();
        List<RunningLine> lines = new ArrayList<>();
        for (double m : LINE_MACH) {
            try { lines.add(runningLine(s, new Flight(c.temperatureK(), c.pressurePa(), m * sound))); machs.add(m); }
            catch (IllegalArgumentException e) {
                if (lines.isEmpty()) { throw e; }
                break;
            }
        }
        return new RunningLines(machs.stream().mapToDouble(Double::doubleValue).toArray(), lines.toArray(RunningLine[]::new));
    }


    /**
     * The compressor at a spool speed in flight. As in the equilibrium (where speed follows from the work by Euler's
     * equation at constant velocity triangles), the speed fixes the work and so the pressure ratio; the flow is what the
     * turbine passes of the actual gas, at most the running line's (the compressor chokes). Below the running line's
     * table (where the affinity laws stand in) the flow stays on the line; the turbine's say phases in between 80 and
     * 100 % of the table's first speed (blend). lineParameter is the running line's turbine flow parameter at this speed.
     */
    private record Spool(double n, double tt2, double theta, double pc, double t3, double face, double blend, double lineParameter) {}

    private static Spool spool(Sizing s, LineAt line, Flight flight, double speedFraction) {
        Design d = s.design();
        Gas air = Gas.AIR;
        double tt2 = flight.totalTemperature(), theta = tt2 / T_REF, n = Math.max(0, speedFraction) / Math.sqrt(theta);
        double pc = line.pressureRatioAt(n), face = line.airflowAt(n);
        double t3 = air.temperatureAtEntropy(air.entropyFunction(tt2) + air.gasConstant() * Math.log(pc) / d.compressorEfficiency());
        double s0 = line.firstSpeed();
        // the running line's burner fuel-air ratio: corrected fuel over corrected airflow, times theta
        double f = face > 0 ? line.fuelAt(n) * theta / (face * (1 - d.coolingBleed())) : 0;
        double t4 = f > 0 ? burnerExit(d.fuel(), t3, f, d.burnerEfficiency()) : t3;
        return new Spool(n, tt2, theta, pc, t3, face, Math.clamp((n - 0.8 * s0) / (0.2 * s0), 0, 1), parameter(d, f, t4));
    }

    /** Turbine flow parameter of gas with burner fuel-air ratio f at t4: choked flux over (1 + f), s/m. */
    private static double parameter(Design d, double f, double t4) { return new Mix(d.fuel(), f).sonic(t4)[2] / (1 + f); }

    /** Inlet and burner at a flow ratio r (to the running line's): {pt2, the running line's airflow (kg/s), pt4}. */
    private static double[] ducts(Sizing s, Spool sp, Flight flight, double r) {
        Design d = s.design();
        double pt2 = flight.totalPressure() * inletRecovery(d, flight.mach(), r * sp.face() / s.faceFlow());
        double lineFlow = Math.max(1e-6, sp.face() * pt2 / P_REF / Math.sqrt(sp.theta()));
        double pb = retained(d.burnerPressureRatio(), corrected(r * lineFlow, sp.t3(), sp.pc() * pt2) / s.burnerFlow());
        return new double[]{pt2, lineFlow, pb * sp.pc() * pt2};
    }

    /** Airflow the turbine passes (over the running line's, at the same pressures) of gas with burner fuel-air ratio f at t4. */
    private static double passes(Sizing s, Spool sp, double f, double t4) { return parameter(s.design(), f, t4) / sp.lineParameter(); }

    private static double flowRatio(Spool sp, double choked) { return 1 - sp.blend() + sp.blend() * Math.min(1, choked); }

    /**
     * Quasi-steady state at a spool speed (fraction of design) and fuel flow (kg/s): the compressor at that speed (see
     * Spool), burner, turbine and nozzle matched to the actual gas. Hotter gas than the running line's passes less flow
     * at the same pressure ratio, towards surge: the surge margin used is the pressure ratio over the running line's at
     * that flow, less one (negative for colder gas). At the running line's fuel this is the equilibrium. Below light-off
     * fuel is zero and the turbine just windmills.
     */
    public static Point quasiSteady(Sizing s, RunningLines lines, Flight flight, double speedFraction, double fuelKgPerS) {
        Design d = s.design();
        Gas air = Gas.AIR;
        LineAt line = lines.at(flight.mach());
        Spool sp = spool(s, line, flight, speedFraction);
        double c = d.coolingBleed(), fuel = Math.max(0, fuelKgPerS), t3 = sp.t3();
        double r = 1, fb = 0, t4 = t3, ch = 1;
        double[] du = ducts(s, sp, flight, 1);
        double pt4Line = du[2];
        for (int k = 0; k < 40; k++) {
            if (k > 0) { du = ducts(s, sp, flight, r); }
            fb = fuel / (r * du[1] * (1 - c));
            t4 = fb > 0 ? burnerExit(d.fuel(), t3, fb, d.burnerEfficiency()) : t3;
            ch = passes(s, sp, fb, t4) * du[2] / pt4Line;
            double next = flowRatio(sp, ch);
            if (Math.abs(next - r) < 1e-10) { break; }
            r = next;
        }
        double airflow = r * du[1], pt2 = du[0], pt4 = du[2];
        Mix hot = new Mix(d.fuel(), fb);
        double fo = fb * (1 - c);
        Mix out = new Mix(d.fuel(), fo);
        double flow4 = airflow * (1 - c) * (1 + fb), flow9 = airflow * (1 + fo);
        double lo = 1e-6, hi = Math.log(40);
        double[] nz = null;
        double t5 = t4, tt5 = t4;
        for (int it = 0; it < 60; it++) {
            double x = 0.5 * (lo + hi);
            t5 = hot.tAtPhi(hot.phi(t4) - d.turbineEfficiency() * hot.r * x);
            tt5 = out.tAtH(((1 - c) * (1 + fb) * hot.h(t5) + c * air.enthalpy(t3)) / (1 + fo));
            double pt5 = pt4 * Math.exp(-x);
            nz = nozzle(out, tt5, retained(d.nozzlePressureRatio(), corrected(flow9, tt5, pt5) / s.tailpipeFlow()) * pt5, flight.pressurePa(),
                    s.nozzleThroatM2(), d.convergentDivergent(), d.exitAreaRatio());
            if (nz[0] > flow9) { lo = x; } else { hi = x; }
            if (hi - lo < 1e-10) { break; }
        }
        double x = 0.5 * (lo + hi);
        double wc = air.enthalpy(t3) - air.enthalpy(sp.tt2());
        double turbinePower = d.mechanicalEfficiency() * flow4 * (hot.h(t4) - hot.h(t5)), compressorPower = airflow * wc;
        double thrust = flow9 * nz[1] - airflow * flight.speedMps() + (nz[2] - flight.pressurePa()) * nz[3];
        double used = sp.pc() / line.pressureRatioAt(line.speedAtAirflow(ch * sp.face())) - 1;
        return new Point(thrust, fuelKgPerS, airflow, sp.pc(), sp.tt2(), pt2, t3, t4, pt4, tt5, pt4 * Math.exp(-x), Math.exp(x), nz[1], nz[2], nz[3],
                nz[4] > 0.5, wc, turbinePower, compressorPower, speedFraction, used);
    }

    /** Fuel flow on the running line at a spool speed (fraction of design) in flight, kg/s. */
    public static double lineFuel(Sizing s, RunningLines lines, Flight flight, double speedFraction) {
        LineAt line = lines.at(flight.mach());
        Spool sp = spool(s, line, flight, speedFraction);
        double pt2 = flight.totalPressure() * inletRecovery(s.design(), flight.mach(), sp.face() / s.faceFlow());
        return line.fuelAt(sp.n()) * pt2 / P_REF * Math.sqrt(sp.theta());
    }

    /** Fuel flow (kg/s) that makes the quasi-steady turbine inlet temperature a given value at a spool speed in flight. */
    public static double fuelForTurbineInlet(Sizing s, RunningLines lines, Flight flight, double speedFraction, double turbineInletK) {
        Design d = s.design();
        LineAt line = lines.at(flight.mach());
        Spool sp = spool(s, line, flight, speedFraction);
        if (!(turbineInletK > sp.t3()) || sp.face() <= 0) { return 0; }
        double f = fuelAirRatio(d.fuel(), sp.t3(), turbineInletK, d.burnerEfficiency()), r = 1;
        double[] du = ducts(s, sp, flight, r);
        double pt4Line = du[2], ch = passes(s, sp, f, turbineInletK);
        for (int k = 0; k < 10; k++) {
            double next = flowRatio(sp, ch * du[2] / pt4Line);
            if (Math.abs(next - r) < 1e-10) { break; }
            r = next;
            du = ducts(s, sp, flight, r);
        }
        return f * r * du[1] * (1 - d.coolingBleed());
    }

    /**
     * Fuel flow (kg/s) at which the quasi-steady compressor uses a given surge margin at a spool speed in flight (the
     * acceleration limit of a fuel control); 0 if even no fuel uses more.
     */
    public static double fuelForSurgeMargin(Sizing s, RunningLines lines, Flight flight, double speedFraction, double used) {
        Design d = s.design();
        LineAt line = lines.at(flight.mach());
        Spool sp = spool(s, line, flight, speedFraction);
        if (sp.face() <= 0) { return 0; }
        double goal = line.airflowAt(line.speedAtPressureRatio(sp.pc() / (1 + used))) / sp.face();
        double[] du = ducts(s, sp, flight, flowRatio(sp, goal));
        double scale = du[2] / ducts(s, sp, flight, 1)[2];
        double t3 = sp.t3(), cold = passes(s, sp, 0, t3) * scale;
        if (cold <= goal) { return 0; }
        // the turbine's flow goes nearly as 1 / sqrt(T4): iterate T4 by that law
        double t4 = Math.min(2200, t3 * (cold / goal) * (cold / goal)), f = 0;
        for (int k = 0; k < 12; k++) {
            f = fuelAirRatio(d.fuel(), t3, t4, d.burnerEfficiency());
            double ratio = passes(s, sp, f, t4) * scale / goal;
            if (Math.abs(ratio - 1) < 1e-9) { break; }
            t4 = Math.clamp(t4 * ratio * ratio, t3 + 1e-6, 2200);
        }
        return f * flowRatio(sp, goal) * du[1] * (1 - d.coolingBleed());
    }
}

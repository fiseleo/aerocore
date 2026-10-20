package dev.mcmod.aerocore.api.engine;

import java.util.List;
import java.util.Objects;

/**
 * Two-spool turbofan: a fan (and booster stages) on the low-pressure spool, driven by the low-pressure turbine; the
 * core (high-pressure compressor, burner, high-pressure turbine) on the other spool. The bypass air leaves through
 * its own nozzle, or joins the core stream in a mixer and leaves through a common nozzle.
 * <ul>
 * <li>Design point (size): the fan's pressure ratio and bypass ratio, the booster's and core's pressure ratios, the
 *   core airflow, the turbine inlet temperature and the efficiencies give the cycle; the turbines' flow areas (each a
 *   nozzle across its pressure ratio), the nozzles' throats and, with a mixer, its two inlets (the core stream entering
 *   at Mach 0.45, the bypass air at the same static pressure) are sized from it.</li>
 * <li>Off design (equilibrium): the turbines' and nozzles' flows and both spools' power balances fix the operating
 *   point for a turbine inlet temperature; the bypass ratio follows. Constant polytropic efficiencies and no
 *   component maps, as in Turbojet: the matching of Mattingly (Elements of Propulsion, ch. 8, the two-spool
 *   turbofan's off-design analysis) with the turbines unchoked where the pressure falls short.</li>
 * <li>Transient (quasiSteady): both spools at their own speeds, each compressor's work and pressure ratio fixed by
 *   its speed; the flows are what the turbines and nozzles pass of the actual gas, and each spool's turbine power less
 *   its compressors' accelerates it. Hotter gas than the running line's passes less core flow at the same speeds,
 *   towards surge: the surge margin used is the HP compressor's pressure ratio over the running line's at the same
 *   corrected flow, less one.</li>
 * </ul>
 * The gas path, its losses and the matching are TwoSpool's (and GasPath's, shared with Turbojet). Rotor speeds are
 * fractions of design; the turbine cooling air rejoins before the low-pressure turbine.
 */
public final class Turbofan {
    private Turbofan() {}

    public enum Exhaust { SEPARATE, MIXED }

    /** Flight Mach numbers the running lines are built at. */
    static final double[] LINE_MACH = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2};

    /**
     * A cycle. core is the gas generator as a turbojet's design: its pressure ratio is the HP compressor's, its airflow
     * the core's (kg/s at design), its compressor efficiency the booster's and HP compressor's (polytropic), its turbine
     * efficiency the HP turbine's, its nozzle the core (or mixed) nozzle, its inlet the engine's. bypassRatio is bypass
     * over core airflow at design (0 makes a two-spool turbojet); the fan's pressure ratio is the same for bypass and
     * core air; bypassPressureRatio the bypass duct's and nozzle's total pressure retained at design,
     * mixerPressureRatio the mixer's (MIXED only).
     */
    public record Design(Turbojet.Design core, double bypassRatio, double fanPressureRatio, double boosterPressureRatio, double fanEfficiency,
                         double lowTurbineEfficiency, double bypassPressureRatio, Exhaust exhaust, double mixerPressureRatio) {
        public Design {
            Objects.requireNonNull(core, "core"); Objects.requireNonNull(exhaust, "exhaust");
            if (!(bypassRatio >= 0) || !(fanPressureRatio >= 1) || !(boosterPressureRatio >= 1)) {
                throw new IllegalArgumentException("Bypass ratio must be nonnegative and the fan's and booster's pressure ratios at least 1");
            }
            if (bypassRatio > 0 && !(fanPressureRatio > 1)) { throw new IllegalArgumentException("A fan that bypasses air must raise its pressure"); }
            for (double e : new double[]{fanEfficiency, lowTurbineEfficiency, bypassPressureRatio, mixerPressureRatio}) {
                if (!(e > 0 && e <= 1)) { throw new IllegalArgumentException("Efficiencies and pressure ratios of losses must be in (0, 1]"); }
            }
        }
        /** Bypass and core airflow at design, kg/s. */
        public double totalAirflowKgPerS() { return core.airflowKgPerS() * (1 + bypassRatio); }
        /** Fan (core side), booster and HP compressor pressure ratios multiplied. */
        public double overallPressureRatio() { return fanPressureRatio * boosterPressureRatio * core.pressureRatio(); }
    }

    /**
     * One operating point. Temperatures and pressures are totals at stations 2 (fan face), 3 (HP compressor delivery),
     * 4 (HP turbine inlet), 45 (between the turbines, cooling air mixed in) and 5 (LP turbine exit). Powers are the
     * turbines' delivered to their spools (after the mechanical losses) and the compressors' absorbed (the fan's
     * including the booster's). solution is the matching's unknowns, to warm-start the next quasi-steady call.
     */
    public record Point(double thrustN, double fuelKgPerS, double airflowKgPerS, double coreAirflowKgPerS, double bypassRatio, double fanPressureRatio,
                        double compressorPressureRatio, double overallPressureRatio, double tt2, double pt2, double tt3, double pt3, double tt4, double tt45,
                        double tt5, double coreThrustN, double bypassThrustN, double lowSpeedFraction, double highSpeedFraction, double fanPowerW,
                        double lowTurbinePowerW, double compressorPowerW, double highTurbinePowerW, double compressorFlowCorrected, double surgeMarginUsed,
                        double[] solution) {
        public double specificThrust() { return thrustN / airflowKgPerS; }
        /** Thrust-specific fuel consumption, kg/(N s). */
        public double tsfc() { return fuelKgPerS / thrustN; }
        /** The fuel-air ratio in the burner (per kilogram of the air that passes it). */
        double burnerFuelAir(double coolingBleed) { return coreAirflowKgPerS > 0 ? fuelKgPerS / (coreAirflowKgPerS * (1 - coolingBleed)) : 0; }
    }

    /** A sized engine: its design, the design point and the flow areas the off-design matching keeps. */
    public static final class Sizing {
        final TwoSpool engine;
        private final Design design;
        private final Point point;
        Sizing(Design design, TwoSpool engine) { this.design = design; this.engine = engine; this.point = Turbofan.point(engine, engine.design, 0); }
        public Design design() { return design; }
        public Turbojet.Flight condition() { return engine.condition; }
        public Point point() { return point; }
        /** HP and LP turbine flow areas (as nozzles), m^2. */
        public double highTurbineThroatM2() { return engine.a4; }
        public double lowTurbineThroatM2() { return engine.a45; }
        /** Core (or mixed) nozzle throat, m^2. */
        public double nozzleThroatM2() { return engine.a8; }
        /** Bypass nozzle throat, or (mixed) the bypass inlet to the mixer, m^2. */
        public double bypassThroatM2() { return engine.aBypass; }
        /** The core stream's inlet to the mixer (MIXED), m^2. */
        public double mixerCoreAreaM2() { return engine.a6; }
        /** Design works per kilogram of the fan, the booster and the HP compressor, J/kg. */
        public double fanWorkJPerKg() { return engine.fanWork; }
        public double boosterWorkJPerKg() { return engine.boosterWork; }
        public double compressorWorkJPerKg() { return engine.compressorWork; }
    }

    /** Running lines at flight Mach numbers from static up, in corrected terms; an opaque table the transient and the control read. */
    public static final class RunningLines {
        final List<TwoSpool.Line> lines;
        RunningLines(List<TwoSpool.Line> lines) { this.lines = List.copyOf(lines); }
        /** The flight Mach numbers with a line. */
        public double[] machs() { return lines.stream().mapToDouble(TwoSpool.Line::mach).toArray(); }
        /** The lowest corrected fan speed of the static line (below it the affinity laws stand in). */
        public double firstLowSpeed() { return lines.getFirst().rows()[0][TwoSpool.C_LOW]; }
    }

    /** Sizes the engine at a design flight condition. */
    public static Sizing size(Design d, Turbojet.Flight flight) {
        boolean bypass = d.bypassRatio() > 0;
        return new Sizing(d, new TwoSpool(d.core(), true, d.bypassRatio(), d.fanPressureRatio(), d.boosterPressureRatio(), d.fanEfficiency(),
                d.lowTurbineEfficiency(), d.bypassPressureRatio(), bypass && d.exhaust() == Exhaust.MIXED, d.mixerPressureRatio(), 1, flight));
    }

    private static Point point(TwoSpool e, TwoSpool.State s, double surge) {
        double airflow = s.core + s.bypass;
        return new Point(s.thrust, s.fuel, airflow, s.core, s.core > 0 ? s.bypass / s.core : 0, s.pt13 / s.pt2, s.pc, s.pt3 / s.pt2, s.t2, s.pt2, s.t3,
                s.pt3, s.t4, s.t45, s.t5, s.coreThrust, s.bypassThrust, Math.sqrt(Math.max(0, s.x[TwoSpool.WL])), Math.sqrt(Math.max(0, s.x[TwoSpool.WH])),
                s.fanPower, s.lowTurbinePower, s.compressorPower, s.highTurbinePower, s.flow25, surge, s.x.clone());
    }

    /** Equilibrium operating point of a sized engine at a flight condition and turbine inlet temperature (K). */
    public static Point equilibrium(Sizing s, Turbojet.Flight flight, double turbineInletK) {
        return point(s.engine, s.engine.equilibrium(flight, turbineInletK, 1, null), 0);
    }

    /** Running lines at LINE_MACH (at the design condition's ambient state), up to the last Mach number with a line. */
    public static RunningLines runningLines(Sizing s) { return new RunningLines(s.engine.lines(LINE_MACH)); }

    /** Corrected-to-actual factors at a flight condition for a running-line row: {sqrt(theta2), delta2}. */
    private static double[] reference(Sizing s, Turbojet.Flight flight, double[] row) {
        double pt2 = flight.totalPressure() * GasPath.inletRecovery(s.design.core().inletRecovery(), s.design.core().inlet(), flight.mach(), row[TwoSpool.C_X + TwoSpool.Q]);
        return new double[]{Math.sqrt(flight.totalTemperature() / Turbojet.T_REF), pt2 / Turbojet.P_REF};
    }

    /** Fuel flow on the running line at a fan speed (fraction of design) in flight, kg/s. */
    public static double lineFuel(Sizing s, RunningLines lines, Turbojet.Flight flight, double lowSpeedFraction) {
        double root = Math.sqrt(flight.totalTemperature() / Turbojet.T_REF);
        double[] row = TwoSpool.at(lines.lines, flight.mach(), TwoSpool.C_LOW, Math.max(0, lowSpeedFraction) / root), ref = reference(s, flight, row);
        return row[TwoSpool.C_FUEL] * ref[1] * ref[0];
    }

    /** HP spool speed (fraction of design) on the running line at a fan speed in flight. */
    public static double lineHighSpeed(RunningLines lines, Turbojet.Flight flight, double lowSpeedFraction) {
        double root = Math.sqrt(flight.totalTemperature() / Turbojet.T_REF);
        return TwoSpool.at(lines.lines, flight.mach(), TwoSpool.C_LOW, Math.max(0, lowSpeedFraction) / root)[TwoSpool.C_HIGH] * root;
    }

    /** Fan speed (fraction of design) on the running line at an HP spool speed in flight (the inverse of lineHighSpeed). */
    public static double lineLowSpeed(RunningLines lines, Turbojet.Flight flight, double highSpeedFraction) {
        double root = Math.sqrt(flight.totalTemperature() / Turbojet.T_REF);
        return TwoSpool.at(lines.lines, flight.mach(), TwoSpool.C_HIGH, Math.max(0, highSpeedFraction) / root)[TwoSpool.C_LOW] * root;
    }

    /**
     * Quasi-steady state at the spools' speeds (fractions of design) and a fuel flow (kg/s) in flight; solution is a
     * previous point's (or null). A spool turning too slowly to pass any core flow gives a point with no flow.
     */
    public static Point quasiSteady(Sizing s, RunningLines lines, Turbojet.Flight flight, double lowSpeedFraction, double highSpeedFraction, double fuelKgPerS,
                                    double[] solution) {
        TwoSpool e = s.engine;
        double fuel = Math.max(0, fuelKgPerS);
        TwoSpool.State st = e.quasiSteady(lines.lines, flight, Math.max(0, lowSpeedFraction), Math.max(0, highSpeedFraction), fuel, 1, solution);
        if (st == null) {
            double tt2 = flight.totalTemperature(), pt2 = flight.totalPressure();
            return new Point(0, fuel, 0, 0, 0, 1, 1, 1, tt2, pt2, tt2, pt2, tt2, tt2, tt2, 0, 0, lowSpeedFraction, highSpeedFraction, 0, 0, 0, 0, 0, 0, null);
        }
        double used = st.pc / TwoSpool.lineCompressorRatio(lines.lines, flight.mach(), st.flow25) - 1;
        return point(e, st, used);
    }

    /**
     * The stream entering the nozzle at an operating point (0.28, for reheat): {total temperature, total pressure after
     * the mixer's and duct's losses, flow (kg/s), fuel-air ratio per kilogram of air}; re-evaluated from the point's
     * solution, so the same numbers the point was made of.
     */
    static double[] nozzleInlet(Sizing s, Point p, Turbojet.Flight flight) {
        if (p.solution() == null) { throw new IllegalArgumentException("A point with no flow has no nozzle stream"); }
        return s.engine.evaluate(flight, p.solution(), Double.NaN, p.fuelKgPerS(), Double.NaN).nozzleInlet;
    }

    /** Fuel flow (kg/s) that would make a turbine inlet temperature (K) at the spool speeds of a quasi-steady point (the acceleration's temperature limit). */
    public static double fuelForTurbineInlet(Sizing s, Point at, double turbineInletK) {
        return s.engine.fuelForTurbineInlet(at.tt3(), at.tt4(), at.burnerFuelAir(s.design.core().coolingBleed()), at.coreAirflowKgPerS(), turbineInletK);
    }

    /** Fuel flow (kg/s) at which the HP compressor would use a surge margin at the spool speeds of a quasi-steady point (an acceleration schedule); 0 if even no fuel uses more. */
    public static double fuelForSurgeMargin(Sizing s, RunningLines lines, Turbojet.Flight flight, Point at, double used) {
        return s.engine.fuelForSurgeMargin(lines.lines, flight.mach(), at.tt3(), at.tt4(), at.burnerFuelAir(s.design.core().coolingBleed()), at.coreAirflowKgPerS(),
                at.compressorPressureRatio(), at.compressorFlowCorrected(), used);
    }
}

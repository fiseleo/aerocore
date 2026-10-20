package dev.mcmod.aerocore.api.engine;

import java.util.List;
import java.util.Objects;

/**
 * Free-turbine shaft engine (turboshaft, or with a reduction gearbox a turboprop): a single-spool gas generator
 * (compressor, burner, turbine) whose gas then drives a power turbine on its own shaft, turning at the speed the load
 * sets; the exhaust leaves with what is left (a little residual thrust).
 * <ul>
 * <li>Design point (size): the gas generator's cycle, the power turbine's efficiency and design speed, and the
 *   exhaust's total pressure over ambient (how far the power turbine expands the gas) give the shaft power; the
 *   turbines' flow areas and the exhaust's are sized from it.</li>
 * <li>Off design (equilibrium): the gas generator's power balance and the turbines' and exhaust's flows fix the
 *   operating point for a turbine inlet temperature and power turbine speed (Saravanamuttoo et al., Gas Turbine
 *   Theory, ch. 8, the free turbine engine). The power turbine's work falls off its best blade-to-spouting speed ratio
 *   (Turbomachinery.turbineSpeedEfficiency: an impulse stage's below it, flatter above): no power with the rotor held but twice the torque,
 *   the free turbine's stall torque.</li>
 * <li>Transient (quasiSteady): the gas generator at its own speed, as Turbofan's spools; the power turbine's torque
 *   goes to the load.</li>
 * </ul>
 * Shaft power and torque are the power turbine's after its mechanical losses; a reduction gearbox is the caller's
 * (Turbomachinery.gearboxEfficiency). The gas path is TwoSpool's.
 */
public final class FreeTurbine {
    private FreeTurbine() {}

    /** Flight Mach numbers the running lines are built at. */
    static final double[] LINE_MACH = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};

    /**
     * A cycle. gasGenerator is the gas generator as a turbojet's design (its nozzle pressure ratio is the exhaust duct's
     * retention; the exhaust is a convergent duct, so convergentDivergent is ignored); the power turbine's polytropic
     * efficiency (at its best speed ratio) and design speed (rpm); the exhaust's total pressure over ambient at design
     * (above 1: the higher, the less the power turbine takes and the more residual thrust).
     */
    public record Design(Turbojet.Design gasGenerator, double powerTurbineEfficiency, double powerTurbineRpm, double exhaustPressureRatio) {
        public Design {
            Objects.requireNonNull(gasGenerator, "gasGenerator");
            if (!(powerTurbineEfficiency > 0 && powerTurbineEfficiency <= 1)) { throw new IllegalArgumentException("Efficiency must be in (0, 1]"); }
            if (!(powerTurbineRpm > 0) || !(exhaustPressureRatio > 1)) { throw new IllegalArgumentException("Power turbine speed must be positive and the exhaust above ambient"); }
        }
        /** The power turbine's design speed, rad/s. */
        public double powerTurbineOmega() { return powerTurbineRpm * Math.PI / 30; }
    }

    /**
     * One operating point. Temperatures and pressures are totals at stations 2 (compressor face), 3 (compressor
     * delivery), 4 (turbine inlet), 45 (between the turbines, cooling air mixed in; the "inter-turbine temperature")
     * and 5 (power turbine exit). solution is the matching's unknowns, to warm-start the next quasi-steady call.
     */
    public record Point(double shaftPowerW, double shaftTorqueNm, double thrustN, double fuelKgPerS, double airflowKgPerS, double pressureRatio, double tt2,
                        double pt2, double tt3, double tt4, double tt45, double tt5, double gasGeneratorSpeedFraction, double powerTurbineSpeedFraction,
                        double compressorPowerW, double turbinePowerW, double compressorFlowCorrected, double surgeMarginUsed, double[] solution) {
        /** Power-specific fuel consumption, kg/J (times 3.6e9 for g/kWh). */
        public double psfc() { return fuelKgPerS / shaftPowerW; }
        double burnerFuelAir(double coolingBleed) { return airflowKgPerS > 0 ? fuelKgPerS / (airflowKgPerS * (1 - coolingBleed)) : 0; }
    }

    /** A sized engine: its design, the design point and the flow areas the off-design matching keeps. */
    public static final class Sizing {
        final TwoSpool engine;
        private final Design design;
        private final Point point;
        Sizing(Design design, TwoSpool engine) { this.design = design; this.engine = engine; this.point = FreeTurbine.point(this, engine.design, 0); }
        public Design design() { return design; }
        public Turbojet.Flight condition() { return engine.condition; }
        public Point point() { return point; }
        public double gasGeneratorTurbineThroatM2() { return engine.a4; }
        public double powerTurbineThroatM2() { return engine.a45; }
        public double exhaustAreaM2() { return engine.a8; }
        /** The compressor's design work, J/kg. */
        public double compressorWorkJPerKg() { return engine.compressorWork; }
    }

    /** Running lines at flight Mach numbers (the power turbine at its best speed ratio), in corrected terms; an opaque table the transient and the control read. */
    public static final class RunningLines {
        final List<TwoSpool.Line> lines;
        RunningLines(List<TwoSpool.Line> lines) { this.lines = List.copyOf(lines); }
        public double[] machs() { return lines.stream().mapToDouble(TwoSpool.Line::mach).toArray(); }
        /** The lowest corrected gas generator speed of the static line (below it the affinity laws stand in). */
        public double firstSpeed() { return lines.getFirst().rows()[0][TwoSpool.C_HIGH]; }
    }

    /** Sizes the engine at a design flight condition. */
    public static Sizing size(Design d, Turbojet.Flight flight) {
        return new Sizing(d, new TwoSpool(d.gasGenerator(), false, 0, 1, 1, 1, d.powerTurbineEfficiency(), 1, false, 1, d.exhaustPressureRatio(), flight));
    }

    private static Point point(Sizing sz, TwoSpool.State s, double surge) {
        return new Point(s.lowTurbinePower, s.shaftTorqueW / sz.design.powerTurbineOmega(), s.thrust, s.fuel, s.core, s.pc, s.t2, s.pt2, s.t3, s.t4, s.t45,
                s.t5, Math.sqrt(Math.max(0, s.x[TwoSpool.WH])), s.powerTurbineSpeed, s.compressorPower, s.highTurbinePower, s.flow25, surge, s.x.clone());
    }

    /** Equilibrium operating point at a flight condition, turbine inlet temperature (K) and power turbine speed (fraction of design; NaN: at its best speed ratio). */
    public static Point equilibrium(Sizing s, Turbojet.Flight flight, double turbineInletK, double powerTurbineSpeedFraction) {
        return point(s, s.engine.equilibrium(flight, turbineInletK, powerTurbineSpeedFraction, null), 0);
    }

    /** Running lines at LINE_MACH (at the design condition's ambient state), up to the last Mach number with a line. */
    public static RunningLines runningLines(Sizing s) { return new RunningLines(s.engine.lines(LINE_MACH)); }

    /** Fuel flow on the running line at a gas generator speed (fraction of design) in flight, kg/s. */
    public static double lineFuel(Sizing s, RunningLines lines, Turbojet.Flight flight, double gasGeneratorSpeedFraction) {
        double root = Math.sqrt(flight.totalTemperature() / Turbojet.T_REF);
        double[] row = TwoSpool.at(lines.lines, flight.mach(), TwoSpool.C_HIGH, Math.max(0, gasGeneratorSpeedFraction) / root);
        double pt2 = flight.totalPressure() * GasPath.inletRecovery(s.design.gasGenerator().inletRecovery(), s.design.gasGenerator().inlet(), flight.mach(),
                row[TwoSpool.C_X + TwoSpool.Q]);
        return row[TwoSpool.C_FUEL] * pt2 / Turbojet.P_REF * root;
    }

    /**
     * Quasi-steady state at the gas generator's and power turbine's speeds (fractions of design) and a fuel flow
     * (kg/s) in flight; solution is a previous point's (or null). A gas generator turning too slowly to pass any flow
     * gives a point with no flow.
     */
    public static Point quasiSteady(Sizing s, RunningLines lines, Turbojet.Flight flight, double gasGeneratorSpeedFraction, double powerTurbineSpeedFraction,
                                    double fuelKgPerS, double[] solution) {
        double fuel = Math.max(0, fuelKgPerS), n = Math.max(0, gasGeneratorSpeedFraction);
        TwoSpool.State st = s.engine.quasiSteady(lines.lines, flight, 0, n, fuel, Math.max(0, powerTurbineSpeedFraction), solution);
        if (st == null) {
            double tt2 = flight.totalTemperature(), pt2 = flight.totalPressure();
            return new Point(0, 0, 0, fuel, 0, 1, tt2, pt2, tt2, tt2, tt2, tt2, n, powerTurbineSpeedFraction, 0, 0, 0, 0, null);
        }
        return point(s, st, st.pc / TwoSpool.lineCompressorRatio(lines.lines, flight.mach(), st.flow25) - 1);
    }

    /** Fuel flow (kg/s) that would make a turbine inlet temperature (K) at the gas generator speed of a quasi-steady point. */
    public static double fuelForTurbineInlet(Sizing s, Point at, double turbineInletK) {
        return s.engine.fuelForTurbineInlet(at.tt3(), at.tt4(), at.burnerFuelAir(s.design.gasGenerator().coolingBleed()), at.airflowKgPerS(), turbineInletK);
    }

    /** Fuel flow (kg/s) at which the compressor would use a surge margin at the gas generator speed of a quasi-steady point; 0 if even no fuel uses more. */
    public static double fuelForSurgeMargin(Sizing s, RunningLines lines, Turbojet.Flight flight, Point at, double used) {
        return s.engine.fuelForSurgeMargin(lines.lines, flight.mach(), at.tt3(), at.tt4(), at.burnerFuelAir(s.design.gasGenerator().coolingBleed()),
                at.airflowKgPerS(), at.pressureRatio(), at.compressorFlowCorrected(), used);
    }
}

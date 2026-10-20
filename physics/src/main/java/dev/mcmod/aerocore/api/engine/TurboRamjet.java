package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.engine.GasPath.Mix;
import dev.mcmod.aerocore.api.gasdynamics.NormalShock;
import dev.mcmod.aerocore.api.gasdynamics.ObliqueShock;

import java.util.Objects;

/**
 * A turbo-ramjet (0.28) of the Pratt and Whitney J58's kind (the SR-71's): a turbojet with an afterburner behind a
 * supersonic spike inlet, which above a Mach number bleeds air from part-way up its compressor through ducts round the
 * engine into the afterburner, so at Mach 3 most of its air bypasses the core and the afterburner works as a ramjet's
 * combustor (Lockheed/P&W J58 characteristics: 148 kg/s, pressure ratio 8.5, 106 kN dry and 151 kN wet at sea level,
 * bleed bypass 20-40 % above Mach 2).
 * <ul>
 * <li>Core: the Turbojet at its equilibrium for a turbine temperature, its face fed at the inlet's recovery.</li>
 * <li>Bypass: a ratio of the core's airflow, opening linearly between two Mach numbers, taken at a share of the
 * compressor's pressure ratio (the bleed stage; the J58's fourth of nine) and losing a duct recovery; it joins the core's
 * exhaust at the afterburner when its pressure is at least the core's (else it is blocked). Streams mix by enthalpy and
 * mass-weighted total pressure.</li>
 * <li>Afterburner and nozzle: Reheat on the joined stream with the core's throat as the dry throat.</li>
 * <li>Inlet: the engine takes at most what the inlet captures at its critical point (more and the recovery falls,
 * supercritical); what it captures and does not take spills, with the momentum lost through the first shock.</li>
 * </ul>
 * Not modelled: the work the front stages put into the bypass air (taken here as free: the J58's bleed served its
 * compressor's matching), the ejector nozzle's secondary air, the spike's translation schedule (the inlet is the
 * SupersonicInlet at its design geometry).
 */
public final class TurboRamjet {
    private TurboRamjet() {}

    /**
     * The core, its afterburner, the inlet and its capture area (m^2), the Mach numbers at which the bypass begins and is
     * full, the bypass over core airflow when full, the bleed's share of the compressor pressure ratio (log), the bypass
     * ducts' recovery.
     */
    public record Design(Turbojet.Design core, Reheat.Design afterburner, SupersonicInlet.Design inlet, double captureAreaM2, double bypassOpenMach,
                         double bypassFullMach, double maxBypassRatio, double bleedShare, double bypassRecovery) {
        public Design {
            Objects.requireNonNull(core, "core"); Objects.requireNonNull(afterburner, "afterburner"); Objects.requireNonNull(inlet, "inlet");
            if (!(captureAreaM2 > 0) || !(bypassFullMach > bypassOpenMach) || !(maxBypassRatio >= 0) || !(bleedShare >= 0 && bleedShare <= 1)
                    || !(bypassRecovery > 0 && bypassRecovery <= 1)) {
                throw new IllegalArgumentException("A turbo-ramjet needs a capture area, a bypass schedule and recoveries in range");
            }
        }
        /** Bypass over core airflow at a flight Mach number. */
        public double bypassRatio(double mach) { return maxBypassRatio * Math.clamp((mach - bypassOpenMach) / (bypassFullMach - bypassOpenMach), 0, 1); }
    }

    /** A sized turbo-ramjet: the design, its core sized at sea level, static, and its inlet's table. */
    public record Sizing(Design design, Turbojet.Sizing core, SupersonicInlet.Table inlet) {}

    /**
     * An operating point: thrust (N), fuel (kg/s, core and afterburner), core and bypass airflow (kg/s), the inlet's
     * recovery and captured share, spillage drag (N), the core's point, the afterburner's, whether the bypass was blocked
     * by the core's higher pressure.
     */
    public record Point(double thrustN, double fuelKgPerS, double coreAirflowKgPerS, double bypassAirflowKgPerS, double recovery, double captureRatio,
                        double spillageDragN, Turbojet.Point core, Reheat.Point reheat, boolean bypassBlocked) {}

    public static Sizing size(Design d) {
        return new Sizing(d, Turbojet.size(d.core(), Turbojet.Flight.sea(0)), SupersonicInlet.Table.of(d.inlet(), Math.max(4, d.inlet().designMach() + 1)));
    }

    /** The engine at a flight condition, turbine temperature (the core in equilibrium) and reheat temperature (at or below the core exhaust's: unlit). */
    public static Point at(Sizing s, Turbojet.Flight flight, double turbineInletK, double reheatK) {
        Design d = s.design();
        double mach = flight.mach();
        SupersonicInlet.Critical c = s.inlet().at(mach);
        double full = flight.pressurePa() / (Gas.AIR.gasConstant() * flight.temperatureK()) * flight.speedMps() * d.captureAreaM2();
        double supply = mach > 1 ? full * c.captureRatio() : Double.POSITIVE_INFINITY;
        double recovery = c.recovery(), beta = d.bypassRatio(mach);
        Turbojet.Point core = null;
        for (int i = 0; i < 4; i++) {
            core = Turbojet.equilibrium(s.core(), flight, turbineInletK, recovery * flight.totalPressure());
            double demand = core.airflowKgPerS() * (1 + beta);
            if (demand <= supply * 1.0001) { break; }
            // supercritical: the recovery falls until the engine takes what the inlet captures (the flows go with pt2)
            recovery *= supply / demand;
        }
        return assemble(s, flight, core, recovery, full, reheatK);
    }

    /**
     * The engine with its core quasi-steady (running lines, a spool speed and the core's fuel flow; the core's face sees
     * its own design's inlet model) and its afterburner at a reheat temperature: the form a part steps.
     */
    public static Point quasiSteady(Sizing s, Turbojet.RunningLines lines, Turbojet.Flight flight, double speedFraction, double coreFuelKgPerS, double reheatK) {
        Turbojet.Point core = Turbojet.quasiSteady(s.core(), lines, flight, speedFraction, coreFuelKgPerS);
        double full = flight.pressurePa() / (Gas.AIR.gasConstant() * flight.temperatureK()) * flight.speedMps() * s.design().captureAreaM2();
        return assemble(s, flight, core, core.pt2() / flight.totalPressure(), full, reheatK);
    }

    /** Bypass, afterburner, nozzle and spillage around a core point. */
    private static Point assemble(Sizing s, Turbojet.Flight flight, Turbojet.Point core, double recovery, double full, double reheatK) {
        Design d = s.design();
        double mach = flight.mach(), v0 = flight.speedMps();
        double coreAir = core.airflowKgPerS(), bypass = d.bypassRatio(mach) * coreAir;
        Turbojet.Design cd = d.core();
        double bleedRatio = Math.pow(Math.max(1, core.pressureRatio()), d.bleedShare());
        double tb = Turbomachinery.compressorExitTemperature(bleedRatio, cd.compressorEfficiency(), core.tt2());
        double ptb = core.pt2() * bleedRatio * d.bypassRecovery();
        double fo = coreAir > 0 ? core.fuelKgPerS() / coreAir : 0, coreFlow = coreAir * (1 + fo);
        double ptCore = GasPath.retained(cd.nozzlePressureRatio(), GasPath.corrected(coreFlow, core.tt5(), core.pt5()) / s.core().tailpipeFlow()) * core.pt5();
        boolean blocked = bypass > 0 && ptb < ptCore;
        Reheat.Point reheat;
        if (bypass <= 0 || blocked) {
            bypass = 0;
            reheat = Reheat.turbojet(s.core(), core, flight, d.afterburner(), reheatK);
        } else {
            double air = coreAir + bypass, fm = core.fuelKgPerS() / air;
            Mix out = new Mix(cd.fuel(), fo), mixed = new Mix(cd.fuel(), fm);
            double ht = (coreFlow * out.h(core.tt5()) + bypass * Gas.AIR.enthalpy(tb)) / (coreFlow + bypass);
            double tt = mixed.tAtH(ht), pt = (coreFlow * ptCore + bypass * ptb) / (coreFlow + bypass);
            reheat = Reheat.lit(cd.fuel(), air, fm, tt, pt, s.core().nozzleThroatM2(), cd.convergentDivergent(), cd.exitAreaRatio(), flight, d.afterburner(),
                    reheatK);
        }
        double taken = coreAir + bypass, spill = 0, captured = Double.NaN;
        if (mach > 1) {
            double supply = full * s.inlet().at(mach).captureRatio();
            spill = Math.max(0, full - Math.min(supply, taken)) * (v0 - firstShockSpeed(d.inlet(), flight));
            captured = taken / full;
        }
        return new Point(reheat.thrustN() - spill, core.fuelKgPerS() + reheat.fuelKgPerS(), coreAir, bypass, recovery, captured, spill, core, reheat, blocked);
    }

    private static double firstShockSpeed(SupersonicInlet.Design in, Turbojet.Flight flight) {
        double mach = flight.mach(), v0 = flight.speedMps();
        try {
            if (in.kind() == SupersonicInlet.Kind.RAMPS) {
                ObliqueShock o = ObliqueShock.of(mach, in.angles()[0], 1.4, false);
                return v0 * o.machBehind() * Math.sqrt(o.temperatureRatio()) / mach;
            }
            if (in.kind() == SupersonicInlet.Kind.CONE) {
                ObliqueShock o = ObliqueShock.of(mach, in.angles()[0], 1.4, false);
                return v0 * o.machBehind() * Math.sqrt(o.temperatureRatio()) / mach;
            }
        } catch (IllegalArgumentException detached) { /* a bow shock */ }
        NormalShock n = NormalShock.of(mach, 1.4);
        return v0 * n.machBehind() * Math.sqrt(n.temperatureRatio()) / mach;
    }
}

package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.engine.GasPath.Mix;
import dev.mcmod.aerocore.api.gasdynamics.Rayleigh;

import java.util.Objects;

/**
 * An afterburner (reheat, 0.28): a second burner between the turbine (or, in a mixed turbofan, the mixer) and the
 * nozzle, burning the engine's own fuel in the oxygen the gas still carries, with a variable nozzle throat that opens so
 * the engine upstream runs as it does dry (Mattingly, Elements of Propulsion, ch. 7 and 8; Saravanamuttoo et al., Gas
 * Turbine Theory, 3.4). The dry engine is the Turbojet or Turbofan as it is: its nozzle pressure ratio holds the
 * afterburner duct's and flameholders' cold loss. Lit:
 * <ul>
 * <li>Energy: the gas is a mass blend of air and the fuel's stoichiometric products, whose enthalpy per kilogram of air
 * (1 - f AFR) h_air + f (1 + AFR) h_products is linear in the fuel-air ratio f, so the added fuel for a reheat
 * temperature follows in closed form; at most stoichiometric (lean gas only).</li>
 * <li>Pressure: heat added in a constant-area duct entered at the inlet Mach number loses total pressure as Rayleigh
 * flow (the "fundamental" loss), gamma at the mean temperature; a temperature that would choke the duct thermally is not
 * reached.</li>
 * <li>Nozzle: the throat opens to pass the hotter, heavier flow at the same upstream pressure (keeping the core on its dry
 * operating point), up to the largest throat its petals reach; past that the reheat temperature is held down instead
 * (the control trims the fuel). The exit area ratio of a convergent-divergent nozzle is kept.</li>
 * </ul>
 * Not modelled: combustion instability (screech, rumble), the light-off pressure pulse, the core's rematch when the
 * nozzle cannot open enough, afterburner cooling-liner air.
 */
public final class Reheat {
    private Reheat() {}

    /** Largest reheat temperature, burner efficiency, Mach number entering the burner, and largest throat over the dry throat. */
    public record Design(double maxTemperatureK, double efficiency, double inletMach, double maxThroatRatio) {
        public Design {
            if (!(maxTemperatureK > 0) || !(efficiency > 0 && efficiency <= 1) || !(inletMach > 0 && inletMach < 0.6) || !(maxThroatRatio >= 1)) {
                throw new IllegalArgumentException("Reheat needs a temperature, an efficiency in (0, 1], an inlet Mach in (0, 0.6) and a throat ratio of at least 1");
            }
        }
    }

    /**
     * A lit afterburner's result: the engine's thrust (N, the whole engine), fuel burned in the afterburner (kg/s), the
     * reheat temperature reached (K), the throat over the dry throat, the exit speed (m/s), and whether the temperature was
     * held down by thermal choking or by the throat's travel.
     */
    public record Point(double thrustN, double fuelKgPerS, double temperatureK, double throatRatio, double exitSpeedMps, boolean thermallyLimited,
                        boolean throatLimited) {}

    /** A turbojet's operating point (dry) with its afterburner lit to a reheat temperature. */
    public static Point turbojet(Turbojet.Sizing s, Turbojet.Point dry, Turbojet.Flight flight, Design r, double temperatureK) {
        Turbojet.Design d = s.design();
        double fo = dry.airflowKgPerS() > 0 ? dry.fuelKgPerS() / dry.airflowKgPerS() : 0;
        double flow = dry.airflowKgPerS() * (1 + fo);
        double pt = GasPath.retained(d.nozzlePressureRatio(), GasPath.corrected(flow, dry.tt5(), dry.pt5()) / s.tailpipeFlow()) * dry.pt5();
        return lit(d.fuel(), dry.airflowKgPerS(), fo, dry.tt5(), pt, s.nozzleThroatM2(), d.convergentDivergent(), d.exitAreaRatio(), flight, r, temperatureK);
    }

    /** A mixed-exhaust turbofan's operating point (dry) with its afterburner lit to a reheat temperature. */
    public static Point turbofan(Turbofan.Sizing s, Turbofan.Point dry, Turbojet.Flight flight, Design r, double temperatureK) {
        if (s.design().exhaust() != Turbofan.Exhaust.MIXED) { throw new IllegalArgumentException("An afterburning turbofan mixes its streams first"); }
        double[] in = Turbofan.nozzleInlet(s, dry, flight);   // {tt, pt (after the mixer's and duct's losses), flow, fuel-air ratio per kg of air}
        double air = in[2] / (1 + in[3]);
        Turbojet.Design core = s.design().core();
        return lit(core.fuel(), air, in[3], in[0], in[1], s.nozzleThroatM2(), core.convergentDivergent(), core.exitAreaRatio(), flight, r, temperatureK);
    }

    /** Fuel-air ratio (per kg of air) after adding fuel to gas at fuel-air fo and tt so it reaches t7 (closed form; at most stoichiometric). */
    static double reheatFuelAir(Fuel fuel, double fo, double tt, double t7, double eta) {
        Gas p = GasPath.stoichiometric(fuel), a = Gas.AIR;
        double afr = fuel.stoichiometricAirFuel();
        double before = (1 - fo * afr) * a.enthalpy(tt) + fo * (1 + afr) * p.enthalpy(tt);
        double atT7 = (1 - fo * afr) * a.enthalpy(t7) + fo * (1 + afr) * p.enthalpy(t7);
        double df = (atT7 - before) / (eta * fuel.lowerHeatingValueJPerKg() + afr * a.enthalpy(t7) - (1 + afr) * p.enthalpy(t7));
        return Math.clamp(fo + Math.max(0, df), fo, 1 / afr);
    }

    /** Reheat temperature for a fuel-air ratio (inverse of reheatFuelAir), K. */
    static double reheatTemperature(Fuel fuel, double fo, double tt, double f7, double eta) {
        double lo = tt, hi = 3500;
        for (int i = 0; i < 80 && hi - lo > 1e-6; i++) {
            double t = 0.5 * (lo + hi);
            if (reheatFuelAir(fuel, fo, tt, t, eta) < f7) { lo = t; } else { hi = t; }
        }
        return 0.5 * (lo + hi);
    }

    /** The afterburner on a stream: air (kg/s) carrying fuel-air fo at total (tt, pt) entering; nozzle of dry throat a8. */
    static Point lit(Fuel fuel, double air, double fo, double tt, double pt, double a8, boolean cd, double exitRatio, Turbojet.Flight flight, Design r,
                     double temperatureK) {
        Objects.requireNonNull(fuel, "fuel");
        if (!(air > 0) || !(pt > flight.pressurePa())) {
            // a stopped or windmilling engine with no pressure to expand: no jet, only the air it takes in slowed to rest
            return new Point(-(air > 0 ? air : 0) * flight.speedMps(), 0, tt, 1, 0, false, false);
        }
        double target = Math.clamp(temperatureK, tt, r.maxTemperatureK());
        boolean thermal = false, throat = false;
        // thermal choking: the Rayleigh limit at the inlet Mach caps the total-temperature ratio
        double gIn = new Mix(fuel, fo).gamma(tt);
        double limit = tt / Rayleigh.totalTemperatureRatio(r.inletMach(), gIn);
        if (target > 0.999 * limit) { target = 0.999 * limit; thermal = true; }
        // the stoichiometric cap
        double fStoich = 1 / fuel.stoichiometricAirFuel();
        if (reheatFuelAir(fuel, fo, tt, target, r.efficiency()) >= fStoich - 1e-12) {
            target = reheatTemperature(fuel, fo, tt, fStoich, r.efficiency());
        }
        double[] st = state(fuel, air, fo, tt, pt, flight, r, target, a8);
        if (st[0] > r.maxThroatRatio()) {
            // the throat cannot open that far: the hottest reheat it can pass at the core's pressure
            double lo = tt, hi = target;
            for (int i = 0; i < 80 && hi - lo > 1e-4; i++) {
                double t = 0.5 * (lo + hi);
                if (state(fuel, air, fo, tt, pt, flight, r, t, a8)[0] > r.maxThroatRatio()) { hi = t; } else { lo = t; }
            }
            target = lo;
            throat = true;
            st = state(fuel, air, fo, tt, pt, flight, r, target, a8);
        }
        double f7 = reheatFuelAir(fuel, fo, tt, target, r.efficiency());
        Mix hot = new Mix(fuel, f7);
        double flow7 = air * (1 + f7), pt7 = st[1];
        double[] nz = GasPath.nozzle(hot, target, pt7, flight.pressurePa(), a8 * st[0], cd, exitRatio);
        double thrust = flow7 * nz[1] - air * flight.speedMps() + (nz[2] - flight.pressurePa()) * nz[3];
        return new Point(thrust, air * (f7 - fo), target, st[0], nz[1], thermal, throat);
    }

    /** {throat over dry throat, total pressure after the burner} with reheat to t7. */
    private static double[] state(Fuel fuel, double air, double fo, double tt, double pt, Turbojet.Flight flight, Design r, double t7, double a8) {
        double f7 = reheatFuelAir(fuel, fo, tt, t7, r.efficiency());
        Mix hot = new Mix(fuel, f7);
        double gMean = hot.gamma(0.5 * (tt + t7));
        double m7 = Rayleigh.machAtTotalTemperatureRatio(Math.min(1, Rayleigh.totalTemperatureRatio(r.inletMach(), gMean) * t7 / tt), gMean, false);
        double pt7 = pt * Rayleigh.totalPressureRatio(m7, gMean) / Rayleigh.totalPressureRatio(r.inletMach(), gMean);
        double throat = GasPath.throatFor(hot, air * (1 + f7), t7, pt7, flight.pressurePa());
        return new double[]{throat / a8, pt7};
    }
}

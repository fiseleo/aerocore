package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.vehicle.SampleEngines;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class TurbojetTest {
    /** Seitzman's worked example (Georgia Tech AE4803, "Engine off-design performance"): kerosene at 42.8 MJ/kg. */
    private static final Fuel SEITZMAN_FUEL = new Fuel("kerosene 42.8", 12, 23, 0, 42.8e6, 360e3, 15, 15, 800, Fuel.GASOLINE_FLAME);
    private static final Turbojet.Design SEITZMAN = new Turbojet.Design(25, 70.6, 0.90, 1773, 0.98, 0.99, 0.90, 0.995, 0, 0.97, false, 1, 0.98,
            Turbojet.Inlet.PITOT, SEITZMAN_FUEL);

    private static Turbojet.Design engine(double pressureRatio, double airflow, double t4) {
        return new Turbojet.Design(pressureRatio, airflow, 0.87, t4, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 0.99, Turbojet.Inlet.PITOT, Fuel.KEROSENE);
    }

    @Test void theEntropyFunctionGivesTheIsentropeOfAnIndependentIntegration() {
        // dlnT = R / cp(T) dlnp with cp straight from the species polynomials (RK4), against phi(T2) = phi(T1) + R ln(p2/p1)
        Gas air = Gas.AIR;
        java.util.function.DoubleUnaryOperator slope = t -> {
            double cp = 0;
            for (Map.Entry<Species, Double> e : air.molesPerKg().entrySet()) { cp += e.getValue() * e.getKey().cp(t); }
            return air.gasConstant() / cp * t;
        };
        for (double[] c : new double[][]{{300, 10}, {600, 0.2}, {1500, 1 / 8.0}}) {
            double t = c[0], h = Math.log(c[1]) / 4000;
            for (int i = 0; i < 4000; i++) {
                double k1 = slope.applyAsDouble(t), k2 = slope.applyAsDouble(t + h / 2 * k1), k3 = slope.applyAsDouble(t + h / 2 * k2), k4 = slope.applyAsDouble(t + h * k3);
                t += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            }
            double phi = air.entropyFunction(c[0]) + air.gasConstant() * Math.log(c[1]);
            assertEquals(t, air.temperatureAtEntropy(phi), 0.05, "from " + c[0] + " K by " + c[1]);
        }
        // and cp is the slope of the enthalpy
        for (double t : new double[]{300, 800, 1600}) { assertEquals(air.cp(t), (air.enthalpy(t + 0.5) - air.enthalpy(t - 0.5)), 0.01 * air.cp(t)); }
    }

    @Test void seitzmansWorkedExampleIsReproducedOnDesignAndInCruise() {
        Turbojet.Sizing s = Turbojet.size(SEITZMAN, new Turbojet.Flight(288, 100000, 0));
        Turbojet.Point d = s.point();
        // The notes use constant cp (1.4 / 1.3) and neglect the fuel's mass: thrust 71 kN, exhaust 665 m/s, Tt5 1330 K.
        assertEquals(71e3, d.thrustN(), 0.08 * 71e3);
        assertEquals(665, d.exitSpeedMps(), 0.02 * 665);
        assertEquals(1330, d.tt5(), 0.05 * 1330);
        // Cruise at 9 km, 258 m/s, T4 1648 K, recovery 0.995 of free-stream: 21.2, 31.7 kg/s, 5754 rpm corrected, 24 kN, 641 m/s.
        Turbojet.Flight cruise = new Turbojet.Flight(258, 33000, 258);
        Turbojet.Point c = Turbojet.equilibrium(s, cruise, 1648, cruise.totalPressure() * 0.995);
        assertEquals(21.2, c.pressureRatio(), 0.02 * 21.2);
        assertEquals(31.7, c.airflowKgPerS(), 0.02 * 31.7);
        assertEquals(5754, c.speedFraction() / Math.sqrt(c.tt2() / 288.15) * 6000, 0.01 * 5754);
        assertEquals(641, c.exitSpeedMps(), 0.02 * 641);
        // the off-design change is what the matching decides: cruise over design thrust as in the notes (24 / 71)
        assertEquals(24 / 71.0, c.thrustN() / d.thrustN(), 0.03 * 24 / 71.0);
    }

    @Test void theEquilibriumAtTheDesignTemperatureIsTheDesignPoint() {
        for (Turbojet.Sizing s : new Turbojet.Sizing[]{Turbojet.size(SEITZMAN, new Turbojet.Flight(288, 100000, 0)),
                SampleEngines.turbojet().sizing(), SampleEngines.modelTurbojet().sizing()}) {
            Turbojet.Point d = s.point(), e = Turbojet.equilibrium(s, s.condition(), s.design().turbineInletK(), Double.NaN);
            assertEquals(1, e.speedFraction(), 1e-6);
            assertEquals(d.pressureRatio(), e.pressureRatio(), 1e-6 * d.pressureRatio());
            assertEquals(d.airflowKgPerS(), e.airflowKgPerS(), 1e-6 * d.airflowKgPerS());
            assertEquals(d.thrustN(), e.thrustN(), 1e-5 * d.thrustN());
            assertEquals(d.compressorPowerW(), d.turbinePowerW(), 1e-6 * d.compressorPowerW(), "the spool balances");
        }
        assertFalse(SampleEngines.modelTurbojet().sizing().point().nozzleChoked(), "the model jet exhausts subsonically");
    }

    @Test void realTurbojetsAtSeaLevelStatic() {
        // J79-GE-17 (pressure ratio 13.5, 77 kg/s, T4 1210 K): 52.8 kN, 23.8 g/(kN s)
        Turbojet.Point j79 = Turbojet.size(engine(13.5, 77, 1210), Turbojet.Flight.sea(0)).point();
        assertEquals(52.8e3, j79.thrustN(), 0.05 * 52.8e3);
        assertEquals(23.8e-6, j79.tsfc(), 0.1 * 23.8e-6);
        // J85-GE-21 (8.3, 24 kg/s, 1253 K): 16.0 kN. The model's 17.7 kN is the cost of giving a small engine the large
        // engine's component efficiencies; its published fuel consumption is quoted inconsistently and is not tested.
        Turbojet.Point j85 = Turbojet.size(engine(8.3, 24, 1253), Turbojet.Flight.sea(0)).point();
        assertEquals(16.0e3, j85.thrustN(), 0.15 * 16.0e3);
    }

    @Test void inletRecoveryLosesANormalShockOrFollowsTheMilSpec() {
        Turbojet.Design pitot = new Turbojet.Design(8, 20, 0.87, 1250, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 1, Turbojet.Inlet.PITOT, Fuel.KEROSENE);
        Turbojet.Design ramp = new Turbojet.Design(8, 20, 0.87, 1250, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 1, Turbojet.Inlet.RAMP, Fuel.KEROSENE);
        assertEquals(0.7209, Turbojet.inletRecovery(pitot, 2), 1e-4, "normal shock tables at Mach 2");
        assertEquals(0.9989, Turbojet.inletRecovery(pitot, 1.1), 1e-4);
        assertEquals(1 - 0.075, Turbojet.inletRecovery(ramp, 2), 1e-12, "MIL-E-5008B at Mach 2");
        assertEquals(1, Turbojet.inletRecovery(ramp, 0.8), 0);
        // the subsonic duct loses with the square of the corrected flow
        Turbojet.Design duct = engine(8, 20, 1250);
        assertEquals(0.99, Turbojet.inletRecovery(duct, 0.5), 1e-12);
        assertEquals(1 - 0.01 / 4, Turbojet.inletRecovery(duct, 0.5, 0.5), 1e-12);
    }

    @Test void theRunningLinesPassThroughTheDesignPointAndRiseWithSpeed() {
        Turbojet.Sizing s = SampleEngines.turbojet().sizing();
        Turbojet.RunningLines lines = Turbojet.runningLines(s);
        assertEquals(0, lines.mach()[0]);
        assertTrue(lines.mach().length >= 8, "lines up to Mach " + lines.mach()[lines.mach().length - 1]);
        Turbojet.RunningLine line = lines.lines()[0];
        double delta = s.point().pt2() / Turbojet.P_REF;
        assertEquals(8, line.pressureRatioAt(1), 1e-3);
        assertEquals(20 / delta, line.airflowAt(1), 1e-3);
        assertEquals(s.point().fuelKgPerS() / delta, line.fuelAt(1), 1e-4);
        for (int i = 1; i < line.speed().length; i++) {
            assertTrue(line.speed()[i] > line.speed()[i - 1] && line.pressureRatio()[i] > line.pressureRatio()[i - 1]
                    && line.airflow()[i] > line.airflow()[i - 1] && line.fuel()[i] > line.fuel()[i - 1], "row " + i);
        }
        assertTrue(line.speed()[0] < 0.45, "the line reaches low power: " + line.speed()[0]);
        // the model jet's unchoked nozzle gains pressure ratio from ram: at the same corrected speed it passes more air
        Turbojet.RunningLines model = Turbojet.runningLines(SampleEngines.modelTurbojet().sizing());
        assertTrue(model.at(0.6).airflowAt(0.9) > 1.02 * model.at(0).airflowAt(0.9));
        assertTrue(model.lines()[0].speed()[0] < 0.5, "a low pressure ratio engine has a running line too: " + model.lines()[0].speed()[0]);
    }

    @Test void quasiSteadyBalancesOnTheRunningLineAndOverFuellingHeadsForSurge() {
        for (Turbojet.Sizing s : new Turbojet.Sizing[]{SampleEngines.turbojet().sizing(), SampleEngines.modelTurbojet().sizing()}) {
            Turbojet.RunningLines lines = Turbojet.runningLines(s);
            for (Turbojet.Flight f : new Turbojet.Flight[]{Turbojet.Flight.sea(0), Turbojet.Flight.sea(100)}) {
                for (double n : new double[]{0.7, 1.0}) {
                    double fuel = Turbojet.lineFuel(s, lines, f, n);
                    Turbojet.Point on = Turbojet.quasiSteady(s, lines, f, n, fuel);
                    assertEquals(0, excess(on), 0.01, "balanced on the line at " + n);
                    assertEquals(0, on.surgeMarginUsed(), 0.005);
                    Turbojet.Point rich = Turbojet.quasiSteady(s, lines, f, n, 1.1 * fuel), lean = Turbojet.quasiSteady(s, lines, f, n, 0.9 * fuel);
                    assertTrue(excess(rich) > 0.02 && rich.surgeMarginUsed() > 0.01 && rich.airflowKgPerS() < on.airflowKgPerS());
                    assertTrue(excess(lean) < -0.02 && lean.surgeMarginUsed() < 0);
                    // the control's limits invert the quasi-steady state
                    double hot = Turbojet.fuelForTurbineInlet(s, lines, f, n, on.tt4() + 60);
                    assertEquals(on.tt4() + 60, Turbojet.quasiSteady(s, lines, f, n, hot).tt4(), 0.5);
                    double edge = Turbojet.fuelForSurgeMargin(s, lines, f, n, 0.1);
                    assertEquals(0.1, Turbojet.quasiSteady(s, lines, f, n, edge).surgeMarginUsed(), 0.002);
                }
            }
        }
    }

    @Test void aStoppedEngineMakesNoThrustAndAConvergentDivergentNozzleHelpsAtHighPressureRatio() {
        Turbojet.Sizing s = SampleEngines.turbojet().sizing();
        Turbojet.Point still = Turbojet.quasiSteady(s, Turbojet.runningLines(s), Turbojet.Flight.sea(0), 0, 0);
        assertEquals(0, still.thrustN(), 1e-9);
        // Seitzman's design point exhausts at 7 times ambient: expanding it further gains thrust
        Turbojet.Design cd = new Turbojet.Design(25, 70.6, 0.90, 1773, 0.98, 0.99, 0.90, 0.995, 0, 0.97, true, 1.5, 0.98, Turbojet.Inlet.PITOT, SEITZMAN_FUEL);
        Turbojet.Flight f = new Turbojet.Flight(288, 100000, 0);
        assertTrue(Turbojet.size(cd, f).point().thrustN() > 1.03 * Turbojet.size(SEITZMAN, f).point().thrustN());
        // a divergence of 3 at a nozzle pressure ratio of about 2.4 would expand far below ambient: the jet separates and
        // leaves at ambient, fully expanded, which still beats the convergent nozzle's under-expanded jet
        Turbojet.Point convergent = Turbojet.size(engine(8, 20, 1250), Turbojet.Flight.sea(0)).point();
        Turbojet.Design overCd = new Turbojet.Design(8, 20, 0.87, 1250, 0.95, 0.98, 0.88, 0.99, 0, 0.98, true, 3, 0.99, Turbojet.Inlet.PITOT, Fuel.KEROSENE);
        Turbojet.Point sep = Turbojet.size(overCd, Turbojet.Flight.sea(0)).point();
        assertTrue(convergent.nozzleChoked() && convergent.exitPressurePa() > 101325);
        assertEquals(101325, sep.exitPressurePa(), 1e-6);
        assertTrue(sep.thrustN() > convergent.thrustN() && sep.thrustN() < 1.05 * convergent.thrustN());
    }

    private static double excess(Turbojet.Point p) { return (p.turbinePowerW() - p.compressorPowerW()) / p.compressorPowerW(); }
}

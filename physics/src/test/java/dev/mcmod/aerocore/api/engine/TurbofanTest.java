package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TurbofanTest {
    /**
     * A CFM56-7B27-class separate-flow fan: 355 kg/s (783 lb/s), bypass ratio 5.1, overall pressure ratio 32.6 (fan 1.62,
     * booster 1.75, HP compressor 11.5), T4 1630 K, 10 % cooling air.
     */
    static final Turbofan.Design CFM56_CLASS = new Turbofan.Design(new Turbojet.Design(11.5, 355 / 6.1, 0.905, 1630, 0.95, 0.995, 0.89, 0.99, 0.10, 0.99, false, 1,
            0.995, Turbojet.Inlet.PITOT, Fuel.KEROSENE), 5.1, 1.62, 1.75, 0.90, 0.91, 0.975, Turbofan.Exhaust.SEPARATE, 1);
    /** An FJ44-1A-class small business-jet fan with a mixed exhaust: 28 kg/s, bypass ratio 3.3, overall 12.8, T4 1320 K. */
    static final Turbofan.Design FJ44_CLASS = small(Turbofan.Exhaust.MIXED, 0.98);

    static Turbofan.Design small(Turbofan.Exhaust exhaust, double mixer) {
        return new Turbofan.Design(new Turbojet.Design(5.5, 28 / 4.3, 0.86, 1320, 0.95, 0.99, 0.85, 0.99, 0.03, 0.98, false, 1, 0.99, Turbojet.Inlet.PITOT,
                Fuel.KEROSENE), 3.3, 1.55, 1.5, 0.87, 0.86, 0.97, exhaust, mixer);
    }

    static final Turbojet.Flight STATIC = Turbojet.Flight.sea(0);
    /** 35,000 ft (ISA: 218.8 K, 23.84 kPa) at Mach 0.78. */
    static final Turbojet.Flight CRUISE = new Turbojet.Flight(218.8, 23842, 0.78 * Math.sqrt(1.4 * 287.05 * 218.8));

    private static double excess(double turbine, double compressor) { return (turbine - compressor) / compressor; }

    @Test void theEquilibriumAtTheDesignTemperatureIsTheDesignPoint() {
        for (Turbofan.Design d : new Turbofan.Design[]{CFM56_CLASS, FJ44_CLASS, small(Turbofan.Exhaust.SEPARATE, 1)}) {
            Turbofan.Sizing s = Turbofan.size(d, STATIC);
            Turbofan.Point p = s.point(), e = Turbofan.equilibrium(s, STATIC, d.core().turbineInletK());
            assertEquals(1, e.lowSpeedFraction(), 1e-6);
            assertEquals(1, e.highSpeedFraction(), 1e-6);
            assertEquals(p.thrustN(), e.thrustN(), 1e-5 * p.thrustN());
            assertEquals(d.bypassRatio(), e.bypassRatio(), 1e-5);
            assertEquals(d.totalAirflowKgPerS(), p.airflowKgPerS(), 1e-9);
            assertEquals(d.overallPressureRatio(), p.overallPressureRatio(), 1e-9);
            assertEquals(0, excess(p.highTurbinePowerW(), p.compressorPowerW()), 1e-9, "the HP spool balances");
            assertEquals(0, excess(p.lowTurbinePowerW(), p.fanPowerW()), 1e-9, "the LP spool balances");
        }
    }

    @Test void aCfm56ClassFanMakesItsThrustAtItsFuelConsumption() {
        Turbofan.Sizing s = Turbofan.size(CFM56_CLASS, STATIC);
        Turbofan.Point p = s.point();
        // CFM56-7B27: 121.4 kN (27,300 lbf) takeoff; static TSFC of such fans about 0.33-0.40 lb/(lbf h), 9.3-11.3 g/(kN s)
        assertEquals(121.4e3, p.thrustN(), 0.05 * 121.4e3);
        assertTrue(p.tsfc() > 9.3e-6 && p.tsfc() < 11.3e-6, "TSFC " + p.tsfc());
        assertTrue(p.bypassThrustN() > 2 * p.coreThrustN(), "a high bypass fan makes most of its thrust with the fan");
        // cruise at 35,000 ft, Mach 0.78: about 0.6 lb/(lbf h) (17 g/(kN s)); ram drag takes most of the thrust
        Turbofan.Point c = Turbofan.equilibrium(s, CRUISE, 1400);
        assertTrue(c.tsfc() > 15e-6 && c.tsfc() < 19e-6, "cruise TSFC " + c.tsfc());
        assertTrue(c.thrustN() > 0.12 * p.thrustN() && c.thrustN() < 0.3 * p.thrustN(), "cruise thrust " + c.thrustN());
    }

    @Test void aSmallMixedFanAndWhatTheMixerDoes() {
        Turbofan.Point p = Turbofan.size(FJ44_CLASS, STATIC).point();
        // FJ44-1A: 8.45 kN (1,900 lbf), about 0.46 lb/(lbf h) (13 g/(kN s))
        assertEquals(8.45e3, p.thrustN(), 0.08 * 8.45e3);
        assertEquals(13e-6, p.tsfc(), 0.1 * 13e-6);
        // mixing the streams without loss gains a little thrust (the hot core's energy shared with the bypass air); a real mixer's loss can eat it
        double separate = Turbofan.size(small(Turbofan.Exhaust.SEPARATE, 1), STATIC).point().thrustN();
        double ideal = Turbofan.size(small(Turbofan.Exhaust.MIXED, 1), STATIC).point().thrustN();
        assertTrue(ideal > separate && ideal < 1.03 * separate, ideal + " vs " + separate);
        assertTrue(p.thrustN() < ideal);
    }

    @Test void throttlingBackSlowsBothSpoolsAndRaisesTheBypassRatio() {
        Turbofan.Sizing s = Turbofan.size(CFM56_CLASS, STATIC);
        Turbofan.Point last = s.point();
        for (double t4 = 1550; t4 >= 1000; t4 -= 110) {
            Turbofan.Point p = Turbofan.equilibrium(s, STATIC, t4);
            assertTrue(p.thrustN() < last.thrustN() && p.lowSpeedFraction() < last.lowSpeedFraction() && p.highSpeedFraction() < last.highSpeedFraction(), "at " + t4);
            assertTrue(p.bypassRatio() > last.bypassRatio(), "at " + t4);
            // the fan slows more than the core: the core's spool holds speed better at part power
            assertTrue(p.lowSpeedFraction() < p.highSpeedFraction());
            last = p;
        }
    }

    @Test void runningLinesPassThroughTheDesignPoint() {
        Turbofan.Sizing s = Turbofan.size(CFM56_CLASS, STATIC);
        Turbofan.RunningLines lines = Turbofan.runningLines(s);
        assertEquals(0, lines.machs()[0]);
        assertTrue(lines.machs().length >= 5, "lines up to Mach " + lines.machs()[lines.machs().length - 1]);
        assertEquals(s.point().fuelKgPerS(), Turbofan.lineFuel(s, lines, STATIC, 1), 1e-3 * s.point().fuelKgPerS());
        assertEquals(1, Turbofan.lineHighSpeed(lines, STATIC, 1), 1e-3);
        assertTrue(lines.firstLowSpeed() < 0.3, "the line reaches idle: " + lines.firstLowSpeed());
        double last = 0;
        for (double n = 0.1; n <= 1.0; n += 0.05) {
            double fuel = Turbofan.lineFuel(s, lines, STATIC, n);
            assertTrue(fuel > last, "fuel rises with fan speed at " + n);
            last = fuel;
        }
    }

    @Test void quasiSteadyBalancesOnTheRunningLineAndOverFuellingHeadsForSurge() {
        for (Turbofan.Design d : new Turbofan.Design[]{CFM56_CLASS, FJ44_CLASS}) {
            Turbofan.Sizing s = Turbofan.size(d, STATIC);
            Turbofan.RunningLines lines = Turbofan.runningLines(s);
            for (Turbojet.Flight f : new Turbojet.Flight[]{STATIC, Turbojet.Flight.sea(100)}) {
                for (double n : new double[]{0.5, 0.8, 1.0}) {
                    double fuel = Turbofan.lineFuel(s, lines, f, n), high = Turbofan.lineHighSpeed(lines, f, n);
                    Turbofan.Point on = Turbofan.quasiSteady(s, lines, f, n, high, fuel, null);
                    assertEquals(0, excess(on.highTurbinePowerW(), on.compressorPowerW()), 0.01, "HP balanced on the line at " + n);
                    assertEquals(0, excess(on.lowTurbinePowerW(), on.fanPowerW()), 0.01, "LP balanced on the line at " + n);
                    assertEquals(0, on.surgeMarginUsed(), 0.005);
                    Turbofan.Point rich = Turbofan.quasiSteady(s, lines, f, n, high, 1.1 * fuel, on.solution());
                    Turbofan.Point lean = Turbofan.quasiSteady(s, lines, f, n, high, 0.9 * fuel, on.solution());
                    assertTrue(excess(rich.highTurbinePowerW(), rich.compressorPowerW()) > 0.02 && rich.surgeMarginUsed() > 0.01);
                    assertTrue(rich.coreAirflowKgPerS() < on.coreAirflowKgPerS(), "hotter gas passes less core air");
                    assertTrue(excess(lean.highTurbinePowerW(), lean.compressorPowerW()) < -0.02 && lean.surgeMarginUsed() < 0);
                    // the warm start lands on the same state as the cold one
                    Turbofan.Point again = Turbofan.quasiSteady(s, lines, f, n, high, fuel, rich.solution());
                    assertEquals(on.thrustN(), again.thrustN(), 1e-6 * on.thrustN());
                    // the control's limits nearly invert the quasi-steady state (the HP turbine taken as choked)
                    double hot = Turbofan.fuelForTurbineInlet(s, on, on.tt4() + 60);
                    assertEquals(on.tt4() + 60, Turbofan.quasiSteady(s, lines, f, n, high, hot, on.solution()).tt4(), 5);
                    double edge = Turbofan.fuelForSurgeMargin(s, lines, f, on, 0.1);
                    assertEquals(0.1, Turbofan.quasiSteady(s, lines, f, n, high, edge, on.solution()).surgeMarginUsed(), 0.025);
                }
            }
        }
    }

    @Test void aStoppedFanMakesNothingAndACrankedOneNeedsItsStarter() {
        Turbofan.Sizing s = Turbofan.size(CFM56_CLASS, STATIC);
        Turbofan.RunningLines lines = Turbofan.runningLines(s);
        Turbofan.Point still = Turbofan.quasiSteady(s, lines, STATIC, 0, 0, 0, null);
        assertEquals(0, still.thrustN(), 1e-9);
        assertEquals(0, still.airflowKgPerS(), 1e-9);
        for (double n : new double[]{0.1, 0.2, 0.3}) {
            Turbofan.Point crank = Turbofan.quasiSteady(s, lines, STATIC, 0.3 * n, n, 0, null);
            assertTrue(crank.airflowKgPerS() > 0 && crank.highTurbinePowerW() < 0.8 * crank.compressorPowerW(), "cold gas cannot turn the core at " + n);
        }
        // windmilling in flight: ram pushes air through and the turbines turn the stopped spools
        Turbofan.Point wind = Turbofan.quasiSteady(s, lines, Turbojet.Flight.sea(100), 0, 0, 0, null);
        assertTrue(wind.airflowKgPerS() > 0 && wind.lowTurbinePowerW() > 0 && wind.thrustN() < 0, "windmilling drags");
    }

    @Test void fanAndGearboxRelations() {
        // a 1.55 m fan at 5,175 rpm: 420 m/s tip speed; a pressure ratio of 1.62 then takes a stage loading of about 0.3
        double tip = Turbomachinery.tipSpeed(1.55, 5175);
        assertEquals(420, tip, 1);
        double loading = Turbomachinery.fanStageLoading(1.62, tip, 0.9, 288.15);
        assertTrue(loading > 0.25 && loading < 0.35, "loading " + loading);
        assertEquals(1.62, Turbomachinery.fanStagePressureRatio(tip, loading, 0.9, 288.15), 1e-6);
        // Mach 0.6 axial inflow: a supersonic relative tip (about 1.4)
        double ts = 288.15 / 1.072, sound = Math.sqrt(1.4 * Gas.AIR.gasConstant() * ts);
        assertEquals(Math.hypot(tip, 0.6 * sound) / sound, Turbomachinery.relativeTipMach(tip, 0.6, 288.15), 1e-12);
        assertTrue(Turbomachinery.relativeTipMach(tip, 0.6, 288.15) > 1.35);
        // the face Mach number inverts the mass flow function (241 kg/(s m^2) corrected at Mach 1)
        assertEquals(1, Turbomachinery.faceMach(250, 1), 0);
        double m = Turbomachinery.faceMach(150, 1);
        double flux = 101325 / Math.sqrt(288.15) * Math.sqrt(1.4 / Gas.AIR.gasConstant()) * m * Math.pow(1 + 0.2 * m * m, -3);
        assertEquals(150, flux, 1e-6);
        // a free turbine held still: no power, twice the torque of its best speed
        assertEquals(0, Turbomachinery.turbineSpeedEfficiency(0));
        assertEquals(1, Turbomachinery.turbineSpeedEfficiency(1));
        assertEquals(0.75, Turbomachinery.turbineSpeedEfficiency(0.5), 1e-12);
        assertEquals(0, Turbomachinery.turbineSpeedEfficiency(1 + Math.sqrt(2)), 1e-12, "past its best a turbine falls off more gently, to nothing at 2.4 times");
        assertEquals(2 * 0.985 * 15, Turbomachinery.gearboxOutputTorque(2, 15, Turbomachinery.gearboxEfficiency(1)), 1e-12);
        // MIL-E-5008B above Mach 5
        Turbojet.Design ramp = new Turbojet.Design(8, 20, 0.87, 1250, 0.95, 0.98, 0.88, 0.99, 0, 0.98, false, 1, 1, Turbojet.Inlet.RAMP, Fuel.KEROSENE);
        assertEquals(800 / (Math.pow(6, 4) + 935), Turbojet.inletRecovery(ramp, 6), 1e-12);
        assertEquals(Turbojet.inletRecovery(ramp, 5 - 1e-9), Turbojet.inletRecovery(ramp, 5 + 1e-9), 1e-3, "continuous at Mach 5");
    }
}

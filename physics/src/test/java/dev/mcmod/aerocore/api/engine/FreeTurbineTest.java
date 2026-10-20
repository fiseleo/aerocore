package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class FreeTurbineTest {
    /** A PT6A-27-class turboprop core: pressure ratio 6.7 (three axial stages and a centrifugal), 2.95 kg/s, T4 1200 K, uncooled; power turbine at 33,000 rpm. */
    static final FreeTurbine.Design PT6_CLASS = new FreeTurbine.Design(new Turbojet.Design(6.7, 2.95, 0.84, 1200, 0.95, 0.99, 0.85, 0.99, 0, 0.97, false, 1, 0.97,
            Turbojet.Inlet.PITOT, Fuel.KEROSENE), 0.86, 33000, 1.05);
    /** A T700-GE-701C-class turboshaft: pressure ratio 17, 4.9 kg/s, T4 1530 K, 8 % cooling air; power turbine at 20,900 rpm. */
    static final FreeTurbine.Design T700_CLASS = new FreeTurbine.Design(new Turbojet.Design(17, 4.9, 0.85, 1530, 0.95, 0.995, 0.85, 0.99, 0.08, 0.98, false, 1, 0.98,
            Turbojet.Inlet.PITOT, Fuel.KEROSENE), 0.87, 20900, 1.04);
    static final Turbojet.Flight STATIC = Turbojet.Flight.sea(0);

    @Test void theEquilibriumAtTheDesignTemperatureIsTheDesignPoint() {
        for (FreeTurbine.Design d : new FreeTurbine.Design[]{PT6_CLASS, T700_CLASS}) {
            FreeTurbine.Sizing s = FreeTurbine.size(d, STATIC);
            FreeTurbine.Point p = s.point(), e = FreeTurbine.equilibrium(s, STATIC, d.gasGenerator().turbineInletK(), 1);
            assertEquals(1, e.gasGeneratorSpeedFraction(), 1e-6);
            assertEquals(p.shaftPowerW(), e.shaftPowerW(), 1e-5 * p.shaftPowerW());
            assertEquals(p.compressorPowerW(), p.turbinePowerW(), 1e-6 * p.compressorPowerW(), "the gas generator balances");
            assertEquals(p.shaftPowerW(), p.shaftTorqueNm() * d.powerTurbineOmega(), 1e-6 * p.shaftPowerW());
        }
    }

    @Test void realShaftEnginesAtSeaLevelStatic() {
        // PT6A-27: 680 shp (507 kW) and 90 lbf (400 N) of jet thrust at 0.603 lb per equivalent shp-hour, i.e. 386 g/kWh of shaft power
        FreeTurbine.Point pt6 = FreeTurbine.size(PT6_CLASS, STATIC).point();
        assertEquals(507e3, pt6.shaftPowerW(), 0.08 * 507e3);
        assertEquals(386, pt6.psfc() * 3.6e9, 0.08 * 386);
        assertEquals(400, pt6.thrustN(), 0.4 * 400);
        // T700-GE-701C: 1,890 shp (1,409 kW) at 0.462 lb/(shp h) (281 g/kWh), GE's data sheet
        FreeTurbine.Point t700 = FreeTurbine.size(T700_CLASS, STATIC).point();
        assertEquals(1409e3, t700.shaftPowerW(), 0.08 * 1409e3);
        assertEquals(281, t700.psfc() * 3.6e9, 0.08 * 281);
    }

    @Test void theFreeTurbineGivesTwiceItsTorqueWhenHeld() {
        FreeTurbine.Sizing s = FreeTurbine.size(T700_CLASS, STATIC);
        FreeTurbine.RunningLines lines = FreeTurbine.runningLines(s);
        FreeTurbine.Point d = s.point();
        FreeTurbine.Point held = FreeTurbine.quasiSteady(s, lines, STATIC, 1, 0, d.fuelKgPerS(), d.solution());
        assertEquals(0, held.shaftPowerW(), 1e-6);
        assertEquals(2, held.shaftTorqueNm() / d.shaftTorqueNm(), 0.05);
        assertTrue(held.tt5() > d.tt5(), "the held turbine takes no work: hotter exhaust");
        double best = 0, bestSpeed = 0;
        for (double n = 0.2; n <= 1.8; n += 0.05) {
            double power = FreeTurbine.quasiSteady(s, lines, STATIC, 1, n, d.fuelKgPerS(), d.solution()).shaftPowerW();
            if (power > best) { best = power; bestSpeed = n; }
        }
        assertEquals(1, bestSpeed, 0.1, "the best speed is the design's");
    }

    @Test void powerFallsWithTurbineTemperatureAndAltitude() {
        FreeTurbine.Sizing s = FreeTurbine.size(PT6_CLASS, STATIC);
        FreeTurbine.Point last = s.point();
        for (double t4 = 1150; t4 >= 950; t4 -= 50) {
            FreeTurbine.Point p = FreeTurbine.equilibrium(s, STATIC, t4, 1);
            assertTrue(p.shaftPowerW() < last.shaftPowerW() && p.gasGeneratorSpeedFraction() < last.gasGeneratorSpeedFraction() && p.psfc() > last.psfc(), "at " + t4);
            last = p;
        }
        // 3 km, 120 m/s: thinner air gives less power at the same turbine temperature
        FreeTurbine.Point high = FreeTurbine.equilibrium(s, new Turbojet.Flight(268.65, 70108, 120), 1200, 1);
        assertTrue(high.shaftPowerW() < 0.95 * s.point().shaftPowerW() && high.shaftPowerW() > 0.6 * s.point().shaftPowerW(), "at 3 km " + high.shaftPowerW());
    }

    @Test void quasiSteadyBalancesOnTheRunningLineAndOverFuellingHeadsForSurge() {
        for (FreeTurbine.Design d : new FreeTurbine.Design[]{PT6_CLASS, T700_CLASS}) {
            FreeTurbine.Sizing s = FreeTurbine.size(d, STATIC);
            FreeTurbine.RunningLines lines = FreeTurbine.runningLines(s);
            assertTrue(lines.firstSpeed() < 0.8, "the line reaches ground idle: " + lines.firstSpeed());
            for (double n : new double[]{0.85, 1.0}) {
                double fuel = FreeTurbine.lineFuel(s, lines, STATIC, n);
                FreeTurbine.Point on = FreeTurbine.quasiSteady(s, lines, STATIC, n, 1, fuel, null);
                assertEquals(0, (on.turbinePowerW() - on.compressorPowerW()) / on.compressorPowerW(), 0.01, "balanced on the line at " + n);
                FreeTurbine.Point rich = FreeTurbine.quasiSteady(s, lines, STATIC, n, 1, 1.1 * fuel, on.solution());
                assertTrue(rich.turbinePowerW() > 1.02 * rich.compressorPowerW() && rich.surgeMarginUsed() > 0.01 && rich.shaftPowerW() > on.shaftPowerW());
                double hot = FreeTurbine.fuelForTurbineInlet(s, on, on.tt4() + 50);
                assertEquals(on.tt4() + 50, FreeTurbine.quasiSteady(s, lines, STATIC, n, 1, hot, on.solution()).tt4(), 5);
                double edge = FreeTurbine.fuelForSurgeMargin(s, lines, STATIC, on, 0.1);
                assertEquals(0.1, FreeTurbine.quasiSteady(s, lines, STATIC, n, 1, edge, on.solution()).surgeMarginUsed(), 0.025);
            }
        }
    }
}

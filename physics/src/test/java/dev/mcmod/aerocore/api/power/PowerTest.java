package dev.mcmod.aerocore.api.power;

import dev.mcmod.aerocore.api.engine.Fuel;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Power calculations (0.26) against the standards' tables and textbook worked examples. */
class PowerTest {
    @Test void electricMachinesFollowTheIe3TableAndPeakBelowRating() {
        assertEquals(0.904, ElectricMachine.ie3Efficiency(7500), 1e-12);
        assertEquals(0.954, ElectricMachine.ie3Efficiency(110000), 1e-12);
        ElectricMachine m = ElectricMachine.of(50000);
        assertEquals(ElectricMachine.ie3Efficiency(50000), m.efficiency(50000), 1e-12);
        // the efficiency peaks where fixed and copper losses are equal: sqrt(0.4 / 0.6) of rating
        double best = Math.sqrt(m.fixedLossW() / m.copperLossW()) * m.ratedW();
        assertTrue(m.efficiency(best) > m.efficiency(m.ratedW()) && m.efficiency(best) > m.efficiency(0.5 * best));
        // a 95 % machine split 2 % fixed, 3 % copper: 95.2 % at rating
        ElectricMachine g = new ElectricMachine(100000, 2000, 3000);
        assertEquals(100000 / 105000.0, g.efficiency(100000), 1e-12);
        // motor and generator are inverses
        assertEquals(60000, g.outputForInput(g.inputForOutput(60000)), 1e-6);
        assertEquals(0, g.outputForInput(1500), 0);
    }

    @Test void batteriesAndCables() {
        Battery b = Battery.nickelIron(30);
        assertEquals(108e6, b.capacityJ(), 1e-6);
        assertEquals(0.7, b.roundTrip(), 0.01);
        assertEquals(6000, b.maxPowerW(), 1e-9);
        assertEquals(1200, b.massKg(), 1e-9);
        // IEC 60364-5-52 B.52.4, method C: 6 mm2 carries 41 A; at 400 V and pf 0.9 that is 25.6 kW
        assertEquals(41, Cable.ampacityA(6), 1e-12);
        assertEquals(25.6e3, Cable.powerW(6, 400, 0.9), 0.1e3);
        assertEquals(223, Cable.ampacityA(95), 1e-12);
        // 3 x 41 A over 50 m of 6 mm2: 3 x 41^2 x 0.0172 x 50 / 6 = 723 W
        assertEquals(723, Cable.lossW(41, 50, 6), 1);
    }

    @Test void beltsAndShafts() {
        // leather 100 x 6 mm at 15 m/s on equal pulleys: T1 1500 N, Tc 131 N, e^(0.35 pi) = 3.0 -> about 13.7 kW
        assertEquals(Math.PI, Belt.wrapRad(0.6, 0.6, 3), 1e-12);
        assertEquals(13.7e3, Belt.leatherW(0.1, 0.006, 15, Math.PI), 0.1e3);
        // a smaller wrap on unequal pulleys carries less
        assertTrue(Belt.leatherW(0.1, 0.006, 15, Belt.wrapRad(1.2, 0.3, 2)) < Belt.leatherW(0.1, 0.006, 15, Math.PI));
        // past the speed where the centrifugal tension eats the allowable, nothing
        assertEquals(0, Belt.leatherW(0.1, 0.006, 60, Math.PI), 0);
        // a 60 mm line shaft at 300 rpm carries 28 MPa x pi 0.06^3 / 16 x 31.4 rad/s = 37 kW
        assertEquals(37.3e3, Shafting.capacityW(0.06, 300 * Math.PI / 30), 0.2e3);
        assertEquals(22.2, Shafting.kgPerM(0.06), 0.1);
        // a bearing carrying 2 kN on a 60 mm journal at 300 rpm loses 0.02 x 2000 x 0.03 x 31.4 = 37.7 W
        assertEquals(37.7, Shafting.bearingLossW(2000, 0.06, 300 * Math.PI / 30), 0.1);
    }

    @Test void theSteamCycleMatchesTheSteamTables() {
        // 1 MPa saturated to 1 atm: x2 = 0.873, w = 388.6 kJ/kg less 0.94 of pumping, q = 2357 kJ/kg: 16.4 %
        SteamEngine s = new SteamEngine(1e6, 1, 1, 1);
        assertEquals(387.7e3, s.rankineWorkJPerKg(), 0.5e3);
        assertEquals(0.1645, s.rankineEfficiency(), 0.001);
        // higher pressure, better cycle
        assertTrue(new SteamEngine(2e6, 1, 1, 1).rankineEfficiency() > s.rankineEfficiency());
        // the mill engine: about 6 % of the coal's heat as work; 15 kW takes about 9.5 g/s of coal (34 kg an hour)
        double overall = SteamEngine.MILL.overallEfficiency();
        assertTrue(overall > 0.05 && overall < 0.10, "overall " + overall);
        assertEquals(15000 / (overall * SolidFuel.COAL.heatingValueJPerKg()), SteamEngine.MILL.fuelKgPerS(15000, SolidFuel.COAL.heatingValueJPerKg()), 1e-12);
        assertEquals(179.88, SteamEngine.saturation(1e6).temperatureC(), 1e-9);
    }

    @Test void dieselAndFuelOil() {
        // C12.3H22.2: 14.5 kg of air per kg (Heywood: about 14.5 for diesel)
        assertEquals(14.5, Fuel.DIESEL.stoichiometricAirFuel(), 0.2);
        assertEquals(Fuel.DIESEL, Fuel.byName("diesel"));
        assertEquals(Fuel.FUEL_OIL, Fuel.byName("fuel_oil"));
        assertEquals(95, Fuel.byName("gasoline").researchOctane(), 0);
        assertNull(Fuel.byName("whale oil"));
    }

    @Test void pipesAndPumps() {
        // water at 20 C, 2 l/s through 100 m of 50 mm steel pipe: v 1.02 m/s, Re 50 900, Swamee-Jain f 0.0238, drop about 24.8 kPa (by hand)
        double drop = PipeFlow.pressureDropPa(0.002, 0.05, 100, PipeFlow.STEEL_ROUGHNESS_M, 998, 1.0e-3, 0);
        assertEquals(24.8e3, drop, 0.6e3);
        assertEquals(0.002, PipeFlow.flowM3PerS(drop, 0.05, 100, PipeFlow.STEEL_ROUGHNESS_M, 998, 1.0e-3, 0), 1e-8);
        // laminar oil (0.1 Pa s): Hagen-Poiseuille 128 mu L Q / (pi D^4)
        double q = 2e-4, poiseuille = 128 * 0.1 * 100 * q / (Math.PI * Math.pow(0.05, 4));
        assertEquals(poiseuille, PipeFlow.pressureDropPa(q, 0.05, 100, PipeFlow.STEEL_ROUGHNESS_M, 870, 0.1, 0), 1e-6 * poiseuille);
        // fittings add K velocity heads; more drop, less flow
        assertTrue(PipeFlow.pressureDropPa(0.002, 0.05, 100, PipeFlow.STEEL_ROUGHNESS_M, 998, 1e-3, 5) > drop);
        // a pump for 2 l/s at 30 m: its curve passes the duty, its power peaks in efficiency there
        Pump p = Pump.forDuty(0.002, 30, 0.6);
        assertEquals(30, p.headM(0.002), 1e-9);
        assertEquals(45, p.headM(0), 1e-9);
        assertEquals(0.6, p.efficiency(0.002), 1e-9);
        assertEquals(998 * 9.80665 * 0.002 * 30 / 0.6, p.shaftPowerW(0.002, 998), 1e-6);
        // into a system of 10 m static lift and the 100 m pipe: the duty point balances
        java.util.function.DoubleUnaryOperator system = flow -> 10 + PipeFlow.pressureDropPa(flow, 0.05, 100, PipeFlow.STEEL_ROUGHNESS_M, 998, 1e-3, 2) / (998 * 9.80665);
        double duty = p.dutyFlowM3PerS(system);
        assertEquals(p.headM(duty), system.applyAsDouble(duty), 1e-3);
        assertTrue(duty > 0.002 && duty < p.runoutFlowM3PerS(), "duty " + duty);
        assertEquals(0, p.dutyFlowM3PerS(flow -> 50), 0, "more lift than its shutoff head: no flow");
    }
}

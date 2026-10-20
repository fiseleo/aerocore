package dev.mcmod.aerocore.api.engine;

import dev.mcmod.aerocore.api.ElectricMotor;
import dev.mcmod.aerocore.api.HeatTransfer;
import dev.mcmod.aerocore.api.PropellerModels;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class EngineCalculationsTest {
    private static final double BORE = 0.086, STROKE = 0.086;

    @Test void compressionRatioComesFromChamberGasketDeckAndCrown() {
        double area = Math.PI * BORE * BORE / 4, swept = area * STROKE;
        // a chamber that, with a 0.8 mm gasket and the crown flush with the deck, leaves a tenth of the swept volume: 11:1
        double chamber = swept / 10 - area * 0.0008;
        assertEquals(swept / 10, PistonGeometry.clearanceVolume(chamber, BORE, 0.0008, 0, 0), 1e-15);
        assertEquals(11, PistonGeometry.compressionRatio(swept, swept / 10), 1e-12);
        // a 5 cm^3 dome takes 5 cm^3 away; a crown 0.5 mm down the bore adds its column
        assertEquals(swept / 10 - 5e-6, PistonGeometry.clearanceVolume(chamber, BORE, 0.0008, 0, 5e-6), 1e-15);
        assertEquals(swept / 10 + area * 0.0005, PistonGeometry.clearanceVolume(chamber, BORE, 0.0008, 0.0005, 0), 1e-15);
        // deck height 0.214 m = half stroke 0.043 + rod 0.143 + compression height 0.028: flush
        assertEquals(0, PistonGeometry.deckClearance(0.214, STROKE, 0.143, 0.028), 1e-15);
    }

    @Test void fourValvesOf35And30MillimetresFitAnInclinedHeadButNotAParallelOne() {
        // By hand: rows of circles 3 mm apart with 1.5 mm to the wall reach too far for parallel valves in an 86 mm bore;
        // a 40 degree included angle narrows them across the bore to cos 20 deg of their diameter.
        assertTrue(PistonGeometry.valvesFit(BORE, 2, 0.0355, 2, 0.0305, 40, 0.0015, 0.003));
        assertFalse(PistonGeometry.valvesFit(BORE, 2, 0.0355, 2, 0.0305, 0, 0.0015, 0.003));
        assertFalse(PistonGeometry.valvesFit(BORE, 2, 0.045, 2, 0.040, 40, 0.0015, 0.003), "valves wider than half the bore");
    }

    @Test void chamberSurfaceAndFlameTravelHaveTheirClosedForms() {
        double a = BORE / 2;
        // a hemispherical chamber (volume 2/3 pi a^3) has twice the bore's area
        assertEquals(2, PistonGeometry.domeChamberAreaFactor(BORE, 2.0 / 3 * Math.PI * a * a * a), 1e-9);
        // a flat chamber of height h adds its wall 2 pi a h
        double h = 0.004;
        assertEquals(1 + 2 * h / a, PistonGeometry.flatChamberAreaFactor(BORE, Math.PI * a * a * h), 1e-12);
        // a central plug reaches the wall at a; one offset by d reaches a + d; two at +-d along the cam axis reach sqrt(a^2 + d^2)
        assertEquals(a, PistonGeometry.flameTravel(BORE, 1, 0), 1e-12);
        assertEquals(a + 0.01, PistonGeometry.flameTravel(BORE, 1, 0.01), 1e-6);
        // (the bore edge is sampled every degree, half a degree off the axes: about 1e-4 m short of the worst point here)
        assertEquals(Math.hypot(a, 0.01), PistonGeometry.flameTravel(BORE, 2, 0.01), 2e-4);
        // crank system: the crank, the rods' rotating share and half the reciprocating mass at the crank radius
        assertEquals(0.1 + 4 * (1 - 0.3) * 0.5 * 0.043 * 0.043 + 4 * 0.5 * 0.4 * 0.043 * 0.043,
                PistonGeometry.crankSystemInertia(0.1, 4, 0.5, 0.3, 0.4, 0.043), 1e-15);
    }

    @Test void strengthLimitsFollowTheirFormulas() {
        // simply supported plate: p = 8 t^2 sigma / (3 (3 + nu) a^2)
        assertEquals(8 * 0.006 * 0.006 * 110e6 / (3 * 3.33 * 0.043 * 0.043), EngineStrength.crownPressureLimit(0.086, 0.006, 110e6, 0.33), 1e-3);
        // the rod's speed: sigma A = m r omega^2 (1 + r / l)
        double omega = EngineStrength.rodSpeedLimit(480e6, 1.5e-4, 0.5, 0.043, 0.143);
        assertEquals(480e6 * 1.5e-4, 0.5 * 0.043 * omega * omega * (1 + 0.043 / 0.143), 1e-6);
    }

    @Test void runTimeRelations() {
        assertEquals(1e6 * 2e-3 / (4 * Math.PI), EngineRuntime.torqueFromMep(1e6, 2e-3), 1e-12);
        double x = 0.8 * 12;
        assertEquals(101325 + 178 * x * x, EngineRuntime.exhaustPressure(101325, 0.8 * 101325, 12, 1), 1e-9);
        assertEquals(2 * 1.2 * 3 * 3 / 2, EngineRuntime.filterDrop(2, 1.2 * 3 * 1, 1.2, 1), 1e-12);
        // with a constant draw equal to the throttle's flow at 0.8 of ambient (not choked), a long step settles the manifold there
        double cdA = 1e-3, ambient = 101325, t = 298;
        double draw = ValveFlow.massFlow(cdA, ambient, t, 0.8 * ambient, 287.05, 1.4);
        double ratio = EngineRuntime.manifoldRatio(1, 1e6, 1e-6, ambient, ambient, cdA, t, 287.05, 1.4, r -> draw);
        assertEquals(0.8, ratio, 1e-6);
        // an injector at four times its rated pressure flows twice its static flow
        assertEquals(2e-6, ValveFlow.injectorFlow(1e-6, 3e5, 12e5), 1e-18);
    }

    @Test void turbomachineryRelations() {
        assertEquals(1 - 0.63 * Math.PI / 12, Turbomachinery.slipFactor(12), 1e-15);
        assertEquals(3.3 / 8 * 4430 * 300 * 300, Turbomachinery.discStress(4430, 300), 1e-6);
        assertEquals(0, Turbomachinery.compressorWork(1, 0.87, 288.15), 1e-9);
        // a constant-cp estimate (1005 J/(kg K), gamma 1.4): T3 = T2 8^(0.4 / (1.4 e)); variable properties are within 2 %
        double t3 = 288.15 * Math.pow(8, 0.4 / 1.4 / 0.87);
        assertEquals(1005 * (t3 - 288.15), Turbomachinery.compressorWork(8, 0.87, 288.15), 0.02 * 1005 * (t3 - 288.15));
        assertEquals(Turbomachinery.compressorWork(8, 0.87, 288.15), Gas.AIR.enthalpy(Turbomachinery.compressorExitTemperature(8, 0.87, 288.15)) - Gas.AIR.enthalpy(288.15), 1e-9);
        // cooled blades: metal = gas - eps (gas - coolant); the hottest gas puts the metal at its service temperature
        assertEquals(1320, Turbomachinery.bladeMetalTemperature(1500, 600, 0.2), 1e-12);
        double hottest = Turbomachinery.maxGasTemperature(1253, 600, 0.2);
        assertEquals(1253, Turbomachinery.bladeMetalTemperature(hottest, 600, 0.2), 1e-9);
        // a centrifugal eye is 0.6 of the tip diameter
        assertEquals(Math.PI / 4 * 0.06 * 0.06 * (1 - 0.09), Turbomachinery.faceArea(true, 0.1, 0.3), 1e-15);
        assertEquals(300 / (2 * 100.0 * 100), Turbomachinery.stageLoading(300, 2, 100), 1e-15);
        assertEquals(3 * Turbomachinery.slipFactor(20) * 400 * 400, Turbomachinery.slipLimitedWork(3, 20, 400), 1e-9);
        assertEquals(Math.PI * 0.4 * 15000 / 60, Turbomachinery.tipSpeed(0.4, 15000), 1e-12);
        assertEquals((5e6 - 4e6) / 1000, Turbomachinery.spoolTorque(5e6, 4e6, 1000), 1e-12);
    }

    @Test void aFinnedCoreRejectsAtMostTheAirsCapacityTimesTheTemperatureDifference() {
        double rho = 1.2, mu = 1.8e-5, k = 0.026, speed = 5, frontal = 0.3, dt = 60;
        double limit = rho * speed * frontal * 1005 * dt;
        double small = HeatTransfer.finnedCoreRejection(1, frontal, 0.03, speed, 360, 300, rho, mu, k);
        double huge = HeatTransfer.finnedCoreRejection(1e6, frontal, 0.03, speed, 360, 300, rho, mu, k);
        assertTrue(small > 0 && small < huge);
        assertEquals(limit, huge, 1e-9 * limit);
        assertEquals(0, HeatTransfer.finnedCoreRejection(1, frontal, 0.03, speed, 290, 300, rho, mu, k));
    }

    @Test void anElectricMotorHasConstantTorqueThenConstantPower() {
        ElectricMotor m = new ElectricMotor(200, 20000, 3000);
        assertEquals(200, m.torque(1, 50), 1e-12, "below the base speed of 100 rad/s");
        assertEquals(20000 / 200.0, m.torque(1, 200), 1e-12, "above it, constant power");
        assertEquals(0, m.torque(1, 3000 * Math.PI / 30 + 1), "beyond the top speed");
        assertEquals(-20000 / 400.0, m.torque(-1, 400), 1e-12, "braking beyond the top speed, limited by power");
        assertEquals(1000 / 0.9, ElectricMotor.electricalPower(1000, 0.9), 1e-9);
        assertEquals(-900, ElectricMotor.electricalPower(-1000, 0.9), 1e-9);
    }

    @Test void propellerModelsChooseTheBSeriesInsideItsRange() {
        assertEquals(0.70 * Math.PI / (2 * 4 * (1 - 0.18)), PropellerModels.chordRatio(4, 0.70, 0.18), 1e-15);
        assertTrue(PropellerModels.inBSeriesRange(4, 0.70, 1.4));
        assertFalse(PropellerModels.inBSeriesRange(5, 0.8, 1.8));
        assertNotNull(PropellerModels.fourQuadrant(4, 0.70, 1.4, 0.18));
        assertNotNull(PropellerModels.fourQuadrant(5, 0.8, 1.8, 0.18));
    }

    @Test void theMapInterpolatesBilinearly() {
        double[] rpms = {1000, 2000, 4000}, pressures = {0.2, 0.6, 1.0};
        EngineMap.Point[][] points = new EngineMap.Point[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            double imep = 100 * rpms[i] + 1e6 * pressures[j];
            points[i][j] = new EngineMap.Point(rpms[i], pressures[j], imep, 0, 0, 0, 0, 0, 0, false, false, 0, 1, 0, 0);
        }
        EngineMap map = new EngineMap(rpms, pressures, points);
        // linear in each variable: exact anywhere inside, clamped outside
        assertEquals(100 * 3000 + 1e6 * 0.8, map.value(3000, 0.8, EngineMap.Point::netImepPa), 1e-6);
        assertEquals(100 * 1500 + 1e6 * 0.3, map.value(1500, 0.3, EngineMap.Point::netImepPa), 1e-6);
        assertEquals(100 * 4000 + 1e6 * 1.0, map.value(6000, 1.3, EngineMap.Point::netImepPa), 1e-6);
    }
}

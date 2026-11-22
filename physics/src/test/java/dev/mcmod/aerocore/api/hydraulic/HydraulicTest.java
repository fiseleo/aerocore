package dev.mcmod.aerocore.api.hydraulic;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** The hydraulic drive calculations against hand sums (checked in Python), the textbook formulas and their own bounds. */
class HydraulicTest {
    private static final double RPM_1800 = 1800 * Hydraulics.RPM_RAD_PER_S;

    @Test void aCylinderMovesAtFlowOverAreaAndPushesWithPressureTimesArea() {
        // 100 mm bore, 70 mm rod at 25 MPa and 1.5 L/s
        Cylinder c = Cylinder.of(0.100, 0.070, 1.2);
        assertEquals(7.853981633974483e-3, c.headAreaM2(), 1e-15);
        assertEquals(4.005530633326987e-3, c.rodSideAreaM2(), 1e-15);
        assertEquals(c.headAreaM2() - c.rodSideAreaM2(), c.rodAreaM2(), 1e-15);
        // v = Q / A: 0.191 m/s out, 0.374 m/s in, 0.390 m/s regenerative
        assertEquals(0.1909859317102744, c.extendingVelocityMps(1.5e-3), 1e-12);
        assertEquals(0.37448221903975365, c.retractingVelocityMps(1.5e-3), 1e-12);
        assertEquals(0.38976720757198857, c.regenerativeVelocityMps(1.5e-3), 1e-12);
        // F = p A: 196.3 kN push, 100.1 kN pull at standstill; with seal friction 0.95 of that once moving
        assertEquals(196349.5408493621, c.forceN(25e6, 0, 0), 1e-6);
        assertEquals(-100138.26583317468, c.forceN(0, 25e6, 0), 1e-6);
        assertEquals(186532.06380689397, c.maxPushN(25e6), 1e-6);
        assertEquals(95131.35254151595, c.maxPullN(25e6), 1e-6);
        assertEquals(c.maxPushN(25e6), c.forceN(25e6, 0, 0.19), 1e-6);
        assertEquals(-c.maxPullN(25e6), c.forceN(0, 25e6, -0.37), 1e-6);
        // a regenerative circuit pushes with p times the rod's cross-section only
        assertEquals(25e6 * c.rodAreaM2(), c.forceN(25e6, 25e6, 0), 1e-6);
        // the supply and the return: retracting sends back A_head / A_rod side times what it takes
        assertEquals(1.5e-3, c.supplyFlowM3PerS(c.extendingVelocityMps(1.5e-3)), 1e-15);
        assertEquals(1.5e-3, c.supplyFlowM3PerS(-c.retractingVelocityMps(1.5e-3)), 1e-15);
        assertEquals(1.5e-3 * c.headAreaM2() / c.rodSideAreaM2(), c.returnFlowM3PerS(-c.retractingVelocityMps(1.5e-3)), 1e-15);
        // the pressure for a force is the exact inverse, moving either way and with an overrunning load
        double[] velocities = {-0.3, -5e-4, 0, 2e-4, 0.3};
        double[][] pressures = {{25e6, 0}, {12e6, 3e6}, {1e5, 20e6}, {-5e4, 1e6}, {0, 0}};
        for (double v : velocities) {
            for (double[] p : pressures) {
                double f = c.forceN(p[0], p[1], v);
                assertEquals(p[0], c.headPressurePa(f, p[1], v), 1e-6 * (1 + Math.abs(p[0])));
                assertEquals(p[1], c.rodPressurePa(f, p[0], v), 1e-6 * (1 + Math.abs(p[1])));
            }
        }
        // a boom's pair: twice the force, half the speed
        Cylinder pair = new Cylinder(0.100, 0.070, 1.2, 2);
        assertEquals(2 * c.maxPushN(25e6), pair.maxPushN(25e6), 1e-6);
        assertEquals(c.extendingVelocityMps(1.5e-3) / 2, pair.extendingVelocityMps(1.5e-3), 1e-15);
        // shut in at mid stroke with next to no dead volume: B A / x on each side (24 MN/m), stiffer towards an end
        double k = c.stiffnessNPerM(0.6, 1.2e9, 1e-12);
        assertEquals(1.2e9 * (c.headAreaM2() / 0.6 + c.rodSideAreaM2() / 0.6), k, 1e-6 * k);
        assertTrue(c.stiffnessNPerM(0.1, 1.2e9, 1e-4) > c.stiffnessNPerM(0.6, 1.2e9, 1e-4));
        assertThrows(IllegalArgumentException.class, () -> Cylinder.of(0.1, 0.1, 1));
    }

    @Test void aPumpAtItsSettingDeliversNothingWhenCompensatedAndFullTorqueWhenFixed() {
        double d = 100 * Hydraulics.CM3_M3, p = 25e6;
        Pump fixed = Pump.axialPiston(d, Pump.Control.FIXED, p, 0);
        double drag = fixed.dragNmPerRadPerS() * RPM_1800;
        // T = D p / (2 pi eta_hm) + c w: 432.5 + 10.7 N m
        assertEquals(d * p / (2 * Math.PI * Hydraulics.AXIAL_PISTON_HYDROMECHANICAL) + drag, fixed.torqueNm(RPM_1800, p), 1e-9);
        assertEquals(443.18147057766487, fixed.torqueNm(RPM_1800, p), 1e-9);
        // eta_v falls linearly: 1 - 0.05 x 25 / 28 at 25 MPa, 2.87 L/s at 1800 rpm
        assertEquals(1 - 0.05 * 25 / 28, fixed.volumetricEfficiencyAt(p), 1e-12);
        assertEquals(0.0028660714285714288, fixed.flowM3PerS(RPM_1800, p), 1e-15);
        // above the setting the relief holds it
        assertEquals(fixed.torqueNm(RPM_1800, p), fixed.torqueNm(RPM_1800, 2 * p), 0);

        Pump compensated = Pump.axialPiston(d, Pump.Control.PRESSURE_COMPENSATED, p, 0);
        assertEquals(0, compensated.flowM3PerS(RPM_1800, p), 0);
        assertEquals(0, compensated.flowM3PerS(RPM_1800, 1.2 * p), 0);
        // at the setting it strokes only to make up its leakage: a few per cent of the fixed pump's torque
        double leak = 0.05 * 25 / 28;
        assertEquals(leak, compensated.displacementFraction(RPM_1800, p), 1e-12);
        assertEquals(30.00263442585972, compensated.torqueNm(RPM_1800, p), 1e-9);
        assertTrue(compensated.torqueNm(RPM_1800, p) < 0.1 * fixed.torqueNm(RPM_1800, p));
        // below the band it is the fixed pump; half-way down the band half the flow
        double pBelow = p * (1 - Pump.COMPENSATOR_BAND) - 1;
        assertEquals(fixed.flowM3PerS(RPM_1800, pBelow), compensated.flowM3PerS(RPM_1800, pBelow), 1e-15);
        double pHalf = p * (1 - Pump.COMPENSATOR_BAND / 2);
        assertEquals(0.5 * fixed.flowM3PerS(RPM_1800, pHalf), compensated.flowM3PerS(RPM_1800, pHalf), 1e-12);
        // a gear pump is leakier and takes more torque for the same work
        Pump gear = Pump.gear(d, 20e6);
        assertEquals(1 - 0.10, gear.volumetricEfficiencyAt(20e6), 1e-12);
        assertTrue(gear.heatW(RPM_1800, 20e6) > fixed.heatW(RPM_1800, 20e6));
    }

    @Test void aPowerLimitedPumpFollowsTheHyperbolaAndIsContinuousAcrossTheCorner() {
        double d = 100 * Hydraulics.CM3_M3, power = 60e3, pMax = 30e6;
        // 3 L/s geometric at 1800 rpm, so the corner is at 20 MPa
        Pump pump = Pump.axialPiston(d, Pump.Control.POWER_LIMITED, pMax, power);
        double geometric = d * RPM_1800 / (2 * Math.PI);
        assertEquals(1, pump.displacementFraction(RPM_1800, 19e6), 0);
        for (double p = 20.5e6; p <= pMax * (1 - Pump.COMPENSATOR_BAND); p += 0.5e6) {
            // what the controller balances: p x the geometric flow
            assertEquals(power, p * pump.displacementFraction(RPM_1800, p) * geometric, 1e-6 * power);
            // what reaches the circuit: that less the leakage power p lambda D w / (2 pi), 4 to 8 % of it here
            double delivered = p * pump.flowM3PerS(RPM_1800, p);
            assertEquals(power - p * (1 - pump.volumetricEfficiencyAt(p)) * geometric, delivered, 1e-6 * power);
            assertTrue(delivered < power && delivered > 0.9 * power, "delivered " + delivered + " at " + p);
            // and what the shaft takes: the limit over eta_hm plus the drag
            assertEquals(power / Hydraulics.AXIAL_PISTON_HYDROMECHANICAL + pump.dragNmPerRadPerS() * RPM_1800 * RPM_1800,
                    pump.shaftPowerW(RPM_1800, p), 1e-6 * power);
        }
        // a pump without leakage delivers on the hyperbola itself, to within 1 %
        Pump tight = new Pump(d, Pump.Control.POWER_LIMITED, pMax, power, 1, 0.92, 28e6, 0);
        for (double p = 21e6; p <= 29e6; p += 1e6) { assertEquals(power, p * tight.flowM3PerS(RPM_1800, p), 0.01 * power); }
        // the limit follows the speed: at half speed the corner moves to 40 MPa, past the setting
        assertEquals(1, pump.displacementFraction(RPM_1800 / 2, 25e6), 0);
        // continuous and never rising across the corner and the compensator band: no step larger than the slopes give
        double previous = pump.flowM3PerS(RPM_1800, 15e6);
        for (double p = 15e6 + 1e3; p <= pMax; p += 1e3) {
            double q = pump.flowM3PerS(RPM_1800, p);
            assertTrue(q <= previous + 1e-15, "rises at " + p);
            assertTrue(previous - q < 1e-5, "jumps at " + p);
            previous = q;
        }
        assertEquals(0, previous, 0);
        double corner = power / geometric;
        assertEquals(pump.flowM3PerS(RPM_1800, corner * (1 - 1e-9)), pump.flowM3PerS(RPM_1800, corner * (1 + 1e-9)), 1e-10);
    }

    @Test void flowSharingKeepsTheRatiosAndTheTotal() {
        double l = 1e-3;
        double[] requests = {1 * l, 2 * l, 3 * l};
        // the pump gives half of what is asked: everyone gets half
        double[] out = new double[3];
        assertEquals(0.5, FlowShare.share(3 * l, requests, out), 1e-12);
        assertArrayEquals(new double[] {0.5 * l, 1 * l, 1.5 * l}, out, 1e-15);
        assertEquals(3 * l, out[0] + out[1] + out[2], 1e-15);
        assertEquals(out[1] / out[0], requests[1] / requests[0], 1e-12);
        assertEquals(out[2] / out[0], requests[2] / requests[0], 1e-12);
        // enough for all: what they ask, the factor 1
        assertEquals(1, FlowShare.share(10 * l, requests, out), 0);
        assertArrayEquals(requests, out, 0);
        // continuous where the supply just meets the demand
        double[] below = FlowShare.share(6 * l * (1 - 1e-9), requests), above = FlowShare.share(6 * l * (1 + 1e-9), requests);
        for (int i = 0; i < 3; i++) { assertEquals(above[i], below[i], 1e-11); }
        // nothing to share, a pump turning backwards, nothing asked
        assertArrayEquals(new double[3], FlowShare.share(0, requests), 0);
        assertArrayEquals(new double[3], FlowShare.share(-2 * l, requests), 0);
        assertArrayEquals(new double[3], FlowShare.share(5 * l, new double[3]), 0);
        // in place
        double[] inPlace = requests.clone();
        FlowShare.share(3 * l, inPlace, inPlace);
        assertArrayEquals(new double[] {0.5 * l, 1 * l, 1.5 * l}, inPlace, 1e-15);
        assertThrows(IllegalArgumentException.class, () -> FlowShare.share(1, new double[] {-1e-3}));
        assertThrows(IllegalArgumentException.class, () -> FlowShare.share(Double.NaN, requests));
    }

    @Test void thePriorityFunctionIsServedFirst() {
        double l = 1e-3;
        // steering 1 L/s, boom 2, arm 3 on a 3 L/s pump: steering in full, the other two share 2 L/s in their ratio
        double[] requests = {1 * l, 2 * l, 3 * l};
        double[] out = new double[3];
        assertEquals(0.4, FlowShare.prioritised(3 * l, requests, 1, out), 1e-12);
        assertArrayEquals(new double[] {1 * l, 0.8 * l, 1.2 * l}, out, 1e-15);
        // less than steering wants: steering gets it all
        assertArrayEquals(new double[] {0.6 * l, 0, 0}, FlowShare.prioritised(0.6 * l, requests, 1), 1e-15);
        // plenty: everyone in full
        assertArrayEquals(requests, FlowShare.prioritised(10 * l, requests, 1), 0);
        // no priority is plain sharing; two in order serve the second before the rest
        assertArrayEquals(FlowShare.share(3 * l, requests), FlowShare.prioritised(3 * l, requests, 0), 0);
        assertArrayEquals(new double[] {1 * l, 2 * l, 0.5 * l}, FlowShare.prioritised(3.5 * l, requests, 2), 1e-15);
        // continuous as the supply passes what steering wants
        assertEquals(FlowShare.prioritised(l * (1 - 1e-9), requests, 1)[1], FlowShare.prioritised(l * (1 + 1e-9), requests, 1)[1], 1e-11);
        assertThrows(IllegalArgumentException.class, () -> FlowShare.prioritised(l, requests, 4));
    }

    @Test void aHydrostatGearsByDisplacementAndStopsAtItsRelief() {
        // 50 cc pump, 80 cc motor, 300 bar relief
        Hydrostat drive = new Hydrostat(Pump.axialPiston(50 * Hydraulics.CM3_M3, Pump.Control.FIXED, 30e6, 0),
                Motor.axialPiston(80 * Hydraulics.CM3_M3));
        assertEquals(50.0 / 80, drive.speedRatio(1, 1, 0), 1e-15);
        // at 20 MPa both units leak: 0.625 x 0.9643 x 0.9638
        assertEquals(0.5808423913043478, drive.speedRatio(1, 1, 20e6), 1e-12);
        assertEquals(drive.speedRatio(1, 1, 20e6), drive.speedRatio(1, 1, -20e6), 0);
        assertEquals(-drive.speedRatio(1, 1, 20e6), drive.speedRatio(-1, 1, 20e6), 0);
        assertEquals(2 * drive.speedRatio(0.5, 1, 20e6), drive.speedRatio(0.5, 0.5, 20e6), 1e-12);
        // the torque limit: eta_hm D_m p_relief / (2 pi) = 351.4 N m, twice that for twice the stroke
        assertEquals(351.4141143469049, drive.outputTorqueLimitNm(1), 1e-9);
        assertEquals(0.5 * 351.4141143469049, drive.outputTorqueLimitNm(0.5), 1e-9);
        // 200 N m at 100 rad/s: 17.46 MPa in the loop, 151.0 N m on the engine
        assertEquals(17461380.804863922, drive.loopPressurePa(100, 1, 200), 1e-3);
        assertEquals(151.03614501652342, drive.inputTorqueNm(1, 100, 1, 200), 1e-9);
        // past the limit the relief holds: the engine sees the pump at the relief
        assertEquals(30e6, drive.loopPressurePa(10, 1, 1000), 0);
        assertEquals(259.49175504113373, drive.inputTorqueNm(1, 10, 1, 1000), 1e-9);
        // braking downhill: the loop drives the engine, with less torque than driving takes
        double braking = drive.inputTorqueNm(1, 100, 1, -200);
        assertTrue(braking < 0 && -braking < drive.inputTorqueNm(1, 100, 1, 200));
        // continuous through neutral
        assertEquals(drive.inputTorqueNm(-1e-9, 100, 1, 200), drive.inputTorqueNm(1e-9, 100, 1, 200), 1e-6);
        assertThrows(IllegalArgumentException.class, () -> drive.speedRatio(1, 0, 0));
    }

    @Test void anOrificeFollowsTheSquareRootLawAndPassesSmoothlyThroughZero() {
        HydraulicFluid oil = HydraulicFluid.ISO_VG46_50C;
        double cd = HydraulicFluid.SHARP_EDGED_CD, a = 1e-5;
        assertEquals(0.611, cd, 1e-3);
        // a 3.6 mm hole turns laminar near 12 Pa
        assertEquals(11.72, oil.orificeTransitionPa(cd, a), 0.01);
        // Q = Cd A sqrt(2 dp / rho): 0.295 L/s at 1 MPa
        assertEquals(cd * a * Math.sqrt(2 * 1e6 / 860), oil.orificeFlowM3PerS(cd, a, 1e6), 1e-9 * 2.95e-4);
        assertEquals(2.9466e-4, oil.orificeFlowM3PerS(cd, a, 1e6), 1e-8);
        assertEquals(-oil.orificeFlowM3PerS(cd, a, 1e6), oil.orificeFlowM3PerS(cd, a, -1e6), 0);
        // within 0.3 % of the square root ten transitions up
        double pt = oil.orificeTransitionPa(cd, a);
        assertEquals(cd * a * Math.sqrt(2 * 10 * pt / 860), oil.orificeFlowM3PerS(cd, a, 10 * pt), 0.003 * cd * a * Math.sqrt(2 * 10 * pt / 860));
        // through zero: nothing at zero, a finite laminar slope, increasing everywhere
        assertEquals(0, oil.orificeFlowM3PerS(cd, a, 0), 0);
        double slope = cd * a * Math.sqrt(2 / 860.0) / Math.sqrt(pt);
        assertEquals(slope, oil.orificeFlowM3PerS(cd, a, 1e-3) / 1e-3, 1e-6 * slope);
        assertEquals(slope, oil.orificeFlowM3PerS(cd, a, -1e-3) / -1e-3, 1e-6 * slope);
        double previous = oil.orificeFlowM3PerS(cd, a, -1e3);
        for (double dp = -1e3 + 0.5; dp <= 1e3; dp += 0.5) {
            double q = oil.orificeFlowM3PerS(cd, a, dp);
            assertTrue(q > previous, "not increasing at " + dp);
            previous = q;
        }
        // the pressure for a flow is the exact inverse
        for (double dp : new double[] {-5e6, -10, -1e-3, 0, 1e-3, 5, 1e6, 35e6}) {
            assertEquals(dp, oil.orificePressureDropPa(cd, a, oil.orificeFlowM3PerS(cd, a, dp)), 1e-9 * Math.abs(dp) + 1e-12);
        }
        // a shut valve passes nothing; the caller's own transition
        assertEquals(0, oil.orificeFlowM3PerS(cd, 0, 1e6), 0);
        assertEquals(cd * a * Math.sqrt(2 / 860.0) * 1e5 / Math.sqrt(Math.sqrt(1e10 + 1e8)), oil.orificeFlowM3PerS(cd, a, 1e5, 1e4), 1e-15);
        // a quarter of a per cent of air at 10 MPa takes the oil's 1.5 GPa to the 1.2 GPa of a working circuit
        assertEquals(1.2e9, HydraulicFluid.effectiveBulkModulusPa(1.5e9, 0.002356975833333334, 10e6 + 101325), 1);
        assertEquals(1.5e9, HydraulicFluid.effectiveBulkModulusPa(1.5e9, 0, 101325), 1e-6);
        assertEquals(oil.densityKgPerM3() * 30e-6, oil.dynamicViscosityPaS(), 1e-12);
    }

    @Test void aMotorTurnsAtFlowOverDisplacementAndGivesTorqueByPressure() {
        Motor m = Motor.axialPiston(80 * Hydraulics.CM3_M3);
        // 2 L/s at 20 MPa: 2 pi Q / D eta_v = 157.1 x 0.9638 = 151.4 rad/s
        assertEquals(151.3883416403777, m.speedRadPerS(2e-3, 20e6), 1e-9);
        assertEquals(0.9637681159420289, m.volumetricEfficiencyAt(20e6), 1e-12);
        assertEquals(Hydraulics.AXIAL_PISTON_VOLUMETRIC, m.volumetricEfficiencyAt(Hydraulics.AXIAL_PISTON_RATED_PA), 1e-12);
        // eta_hm D dp / (2 pi) - c w = 234.3 - 6.8 N m
        assertEquals(227.46729269050488, m.torqueNm(150, 20e6), 1e-9);
        assertEquals(Hydraulics.AXIAL_PISTON_HYDROMECHANICAL * 80e-6 * 30e6 / (2 * Math.PI), m.maxTorqueNm(30e6), 1e-9);
        // the inverses hold in every quadrant, at standstill and in the friction ramp
        double[] speeds = {-200, -0.5, 0, 0.3, 150};
        double[] drops = {-2e7, -1e5, 0, 1e5, 2e7};
        for (double w : speeds) {
            for (double dp : drops) {
                assertEquals(w, m.speedRadPerS(m.flowM3PerS(w, dp), dp), 1e-9 * (1 + Math.abs(w)));
                assertEquals(dp, m.pressureDropPa(w, m.torqueNm(w, dp)), 1e-6 * (1 + Math.abs(dp)));
            }
        }
        // a load driving the motor as a pump takes more torque than the oil gives driving it
        assertTrue(-m.torqueNm(150, -20e6) > m.torqueNm(150, 20e6));
        // the ideal relations
        assertEquals(2 * Math.PI * 2e-3 / 80e-6, Hydraulics.speedRadPerS(2e-3, 80e-6), 1e-9);
        assertEquals(80e-6 * 20e6 / (2 * Math.PI), Hydraulics.torqueNm(20e6, 80e-6), 1e-9);
        assertEquals(20e6, Hydraulics.pressureForTorquePa(Hydraulics.torqueNm(20e6, 80e-6), 80e-6), 1e-6);
        assertEquals(25e6, Hydraulics.pressureForForcePa(25e6 * 7.85e-3, 7.85e-3), 1e-6);
        assertEquals(37.5e3, Hydraulics.powerW(25e6, 1.5e-3), 1e-9);
    }

    @Test void noMachineMakesPowerOutOfNothing() {
        Pump[] pumps = {
                Pump.gear(40e-6, 20e6),
                Pump.axialPiston(100e-6, Pump.Control.PRESSURE_COMPENSATED, 28e6, 0),
                Pump.axialPiston(100e-6, Pump.Control.POWER_LIMITED, 35e6, 50e3)};
        for (Pump pump : pumps) {
            for (double w = -300; w <= 300; w += 7.3) {
                for (double p = 0; p <= 36e6; p += 0.9e6) {
                    double heat = pump.heatW(w, p);
                    assertTrue(heat >= -1e-9, pump.control() + " heat " + heat + " at " + w + ", " + p);
                    if (w > 0) { assertTrue(pump.hydraulicPowerW(w, p) <= pump.shaftPowerW(w, p) + 1e-9); }
                }
            }
        }
        Motor m = Motor.gear(32e-6);
        for (double w = -400; w <= 400; w += 3.7) {
            for (double dp = -25e6; dp <= 25e6; dp += 0.7e6) {
                assertTrue(m.heatW(w, dp) >= -1e-9, "motor heat at " + w + ", " + dp);
                if (w * dp > 0) { assertTrue(m.shaftPowerW(w, dp) <= m.hydraulicPowerW(w, dp) + 1e-9); }
            }
        }
        // a hydrostat gives out less than the engine puts in (the motor at 100 rad/s, the pump as fast as that needs;
        // 400 N m is 34.5 MPa, inside the relief; overall 0.78 at best)
        Hydrostat drive = new Hydrostat(Pump.axialPiston(50e-6, Pump.Control.FIXED, 35e6, 0), Motor.axialPiston(80e-6));
        for (double torque = 10; torque <= 400; torque += 10) {
            double dp = drive.loopPressurePa(100, 1, torque);
            double pumpSpeed = 100 / drive.speedRatio(1, 1, dp);
            double in = drive.inputTorqueNm(1, 100, 1, torque) * pumpSpeed;
            assertTrue(torque * 100 < 0.8 * in, "hydrostat at " + torque);
        }
        // seal friction always opposes the motion
        Cylinder c = Cylinder.of(0.08, 0.045, 0.6);
        for (double v = -0.2; v <= 0.2; v += 0.01) {
            double friction = c.forceN(18e6, 2e6, v) - c.forceN(18e6, 2e6, 0);
            assertTrue(friction * v <= 0);
        }
    }

    @Test void nothingIsNaNOverASweepIncludingZero() {
        double[] speeds = {-1e5, -300, -1, -1e-12, 0, 1e-12, 0.5, 1, 300, 1e5};
        double[] pressures = {-1e8, -1, 0, 1e-12, 1, 1e5, 2.5e7, 3.5e7, 1e9};
        Pump[] pumps = {
                Pump.gear(40e-6, 20e6),
                Pump.axialPiston(100e-6, Pump.Control.FIXED, 28e6, 0),
                Pump.axialPiston(100e-6, Pump.Control.PRESSURE_COMPENSATED, 28e6, 0),
                Pump.axialPiston(100e-6, Pump.Control.POWER_LIMITED, 35e6, 50e3)};
        Motor motor = Motor.axialPiston(80e-6);
        Hydrostat drive = new Hydrostat(pumps[1], motor);
        Cylinder c = Cylinder.of(0.1, 0.07, 1.2);
        HydraulicFluid oil = HydraulicFluid.ISO_VG46_50C_IN_CIRCUIT;
        for (double w : speeds) {
            for (double p : pressures) {
                for (Pump pump : pumps) {
                    finite(pump.displacementFraction(w, p), pump.flowM3PerS(w, p), pump.torqueNm(w, p), pump.shaftPowerW(w, p),
                            pump.hydraulicPowerW(w, p), pump.heatW(w, p), pump.volumetricEfficiencyAt(p));
                }
                finite(motor.flowM3PerS(w, p), motor.speedRadPerS(w * 1e-5, p), motor.torqueNm(w, p), motor.pressureDropPa(w, p),
                        motor.heatW(w, p), motor.volumetricEfficiencyAt(p));
                finite(drive.speedRatio(0, 1, p), drive.speedRatio(-1, 0.2, p), drive.loopPressurePa(w, 0.3, p), drive.inputTorqueNm(0, w, 1, p));
                finite(c.forceN(p, p * 0.5, w), c.headPressurePa(p, p, w), c.rodPressurePa(-p, p, w), c.supplyFlowM3PerS(w),
                        c.returnFlowM3PerS(w), c.stiffnessNPerM(w, 1.2e9, 1e-4));
                finite(oil.orificeFlowM3PerS(0.6, 1e-5, p), oil.orificeFlowM3PerS(0, 1e-5, p), oil.orificePressureDropPa(0.6, 1e-5, w * 1e-6));
            }
        }
        // a stopped pump delivers nothing; the torque is continuous through standstill
        for (Pump pump : pumps) {
            assertEquals(0, pump.flowM3PerS(0, 2e7), 0);
            assertEquals(pump.torqueNm(-1e-9, 2e7), pump.torqueNm(1e-9, 2e7), 1e-6);
            assertEquals(pump.flowM3PerS(-1e-9, 2e7), pump.flowM3PerS(1e-9, 2e7), 1e-12);
        }
        assertEquals(motor.torqueNm(-1e-9, 2e7), motor.torqueNm(1e-9, 2e7), 1e-6);
        // non-finite inputs are refused
        assertThrows(IllegalArgumentException.class, () -> pumps[0].flowM3PerS(Double.NaN, 1e6));
        assertThrows(IllegalArgumentException.class, () -> pumps[0].torqueNm(100, Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> motor.speedRadPerS(Double.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> c.forceN(1e6, 0, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> oil.orificeFlowM3PerS(0.6, 1e-5, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> Pump.axialPiston(100e-6, Pump.Control.POWER_LIMITED, 28e6, 0));
        assertThrows(IllegalArgumentException.class, () -> new HydraulicFluid(860, -1, 30e-6));
    }

    @Test void thePumpIsContinuousInSpeedAndPressure() {
        Pump pump = Pump.axialPiston(100e-6, Pump.Control.PRESSURE_COMPENSATED, 28e6, 0);
        // through standstill and the friction ramp: no step in torque or flow
        double previousTorque = pump.torqueNm(-3, 20e6), previousFlow = pump.flowM3PerS(-3, 20e6);
        for (double w = -3 + 1e-3; w <= 3; w += 1e-3) {
            double t = pump.torqueNm(w, 20e6), q = pump.flowM3PerS(w, 20e6);
            assertTrue(Math.abs(t - previousTorque) < 0.1, "torque steps at " + w);
            assertTrue(q >= previousFlow, "flow falls with speed at " + w);
            previousTorque = t;
            previousFlow = q;
        }
        // through the compensator band: the torque turns over but does not jump
        previousTorque = pump.torqueNm(RPM_1800, 26e6);
        for (double p = 26e6 + 1e3; p <= 28e6; p += 1e3) {
            double t = pump.torqueNm(RPM_1800, p);
            assertTrue(Math.abs(t - previousTorque) < 1, "torque steps at " + p);
            previousTorque = t;
        }
        // from the tank's pressure up the fixed pump's flow only falls, its torque only rises
        Pump fixed = Pump.gear(40e-6, 20e6);
        for (double p = 0; p < 20e6; p += 1e5) {
            assertTrue(fixed.flowM3PerS(RPM_1800, p + 1e5) < fixed.flowM3PerS(RPM_1800, p));
            assertTrue(fixed.torqueNm(RPM_1800, p + 1e5) > fixed.torqueNm(RPM_1800, p));
        }
    }

    private static void finite(double... values) {
        for (int i = 0; i < values.length; i++) { assertTrue(Double.isFinite(values[i]), "value " + i + " is " + values[i]); }
    }
}

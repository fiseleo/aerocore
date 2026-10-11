package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import java.util.Map;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class EngineModelTest {
    @Test void speciesMatchJanafAtRoomTemperature() {
        // JANAF (NIST-JANAF thermochemical tables, 4th ed.) cp at 298.15 K, J/(mol K), and enthalpies of formation, kJ/mol.
        assertEquals(29.124, Species.N2.cp(298.15), 0.15);
        assertEquals(29.376, Species.O2.cp(298.15), 0.15);
        assertEquals(37.129, Species.CO2.cp(298.15), 0.2);
        assertEquals(33.588, Species.H2O.cp(298.15), 0.17);
        assertEquals(29.142, Species.CO.cp(298.15), 0.15);
        assertEquals(28.836, Species.H2.cp(298.15), 0.15);
        assertEquals(-393.51, Species.CO2.enthalpy(298.15) / 1000, 1.0);
        assertEquals(-241.83, Species.H2O.enthalpy(298.15) / 1000, 0.7);
        assertEquals(-110.53, Species.CO.enthalpy(298.15) / 1000, 0.4);
        assertEquals(0, Species.N2.enthalpy(298.15) / 1000, 0.05);
    }

    @Test void polynomialsJoinAtTheirMiddleTemperature() {
        for (Species s : Species.values()) {
            assertEquals(s.cp(999.999), s.cp(1000.001), 0.005 * s.cp(1000), s.name());
            assertEquals(s.enthalpy(999.999), s.enthalpy(1000.001), 0.002 * Math.abs(s.enthalpy(1000)) + 20, s.name());
        }
    }

    @Test void airHasItsTextbookGasConstantAndGamma() {
        assertEquals(287.05, Gas.AIR.gasConstant(), 0.15);
        assertEquals(1.400, Gas.AIR.cp(300) / Gas.AIR.cv(300), 0.002);
        assertEquals(0, Gas.AIR.energy(298.15), 0.01);
        Random random = new Random(3);
        for (int i = 0; i < 200; i++) {
            double t = 250 + 3500 * random.nextDouble();
            assertEquals(t, Gas.AIR.temperature(Gas.AIR.energy(t)), 1e-6);
        }
    }

    @Test void stoichiometricAirFuelRatiosAreHeywoods() {
        // Heywood (1988) appendix D: gasoline 14.6, methanol 6.47, propane 15.7, isooctane 15.1.
        assertEquals(14.6, Fuel.gasoline(95, 85).stoichiometricAirFuel(), 0.15);
        assertEquals(6.47, Fuel.METHANOL.stoichiometricAirFuel(), 0.05);
        assertEquals(15.7, Fuel.PROPANE.stoichiometricAirFuel(), 0.1);
        assertEquals(15.1, Fuel.ISOOCTANE.stoichiometricAirFuel(), 0.1);
    }

    @Test void productsConserveTheElementsAndHoldTheWaterGasShift() {
        for (Fuel fuel : new Fuel[]{Fuel.gasoline(95, 85), Fuel.METHANOL, Fuel.PROPANE}) {
            for (double lambda : new double[]{0.65, 0.8, 0.95, 1.0, 1.2, 1.6}) {
                Map<Species, Double> p = fuel.productMoles(lambda);
                double c = p.getOrDefault(Species.CO2, 0.0) + p.getOrDefault(Species.CO, 0.0);
                double h = 2 * p.getOrDefault(Species.H2O, 0.0) + 2 * p.getOrDefault(Species.H2, 0.0);
                double o = 2 * p.getOrDefault(Species.CO2, 0.0) + p.getOrDefault(Species.CO, 0.0) + p.getOrDefault(Species.H2O, 0.0) + 2 * p.getOrDefault(Species.O2, 0.0);
                double oSupplied = 2 * fuel.stoichiometricOxygen() * lambda + fuel.oxygen();
                assertEquals(fuel.carbon(), c, 1e-12);
                assertEquals(fuel.hydrogen(), h, 1e-12);
                assertEquals(oSupplied, o, 1e-12);
                assertEquals(Species.AIR_N2_PER_O2 * fuel.stoichiometricOxygen() * lambda, p.get(Species.N2), 1e-12);
                assertEquals(Species.AIR_AR_PER_O2 * fuel.stoichiometricOxygen() * lambda, p.get(Species.AR), 1e-12);
                if (lambda < 1) {
                    double k = p.get(Species.CO) * p.get(Species.H2O) / (p.get(Species.CO2) * p.get(Species.H2));
                    assertEquals(Fuel.WATER_GAS_K, k, 1e-9, fuel.name() + " lambda " + lambda);
                }
            }
        }
    }

    @Test void richMixturesReleaseLessOfTheFuelsEnergy() {
        Fuel gasoline = Fuel.gasoline(95, 85);
        assertEquals(gasoline.lowerHeatingValueJPerKg(), gasoline.heatRelease(1), 1e-6);
        assertEquals(1, gasoline.heatRelease(0.9999) / gasoline.lowerHeatingValueJPerKg(), 0.002);
        double previous = 1;
        for (double lambda = 0.95; lambda >= 0.6; lambda -= 0.05) {
            double fraction = gasoline.heatRelease(lambda) / gasoline.lowerHeatingValueJPerKg();
            assertTrue(fraction < previous);
            previous = fraction;
        }
        // Worked by hand for CH1.87 at lambda 0.8: O balance CO2 + H2O = 1.348 per C, K = 3.5 gives CO2 0.575, CO 0.425,
        // H2O 0.773, H2 0.162; their heats of combustion (283.0, 241.8 kJ/mol) leave 159.5 kJ per 13.90 g unreleased.
        assertEquals(1 - 159.5 / 13.896 / 44.0, gasoline.heatRelease(0.8) / gasoline.lowerHeatingValueJPerKg(), 0.004);
    }

    @Test void crankKinematicsMatchTheirDefinitions() {
        CrankKinematics k = new CrankKinematics(0.086, 0.086, 0.139, 0.0000500);
        assertEquals(k.clearanceM3(), k.volume(0), 1e-15);
        assertEquals(k.clearanceM3() + k.displacement(), k.volume(Math.PI), 1e-12);
        for (double theta = 0.1; theta < 6.2; theta += 0.37) {
            double h = 1e-6;
            assertEquals((k.volume(theta + h) - k.volume(theta - h)) / (2 * h), k.volumeRate(theta), 1e-9);
            double s0 = k.pinHeight(theta - h), s1 = k.pinHeight(theta), s2 = k.pinHeight(theta + h);
            assertEquals((s0 - 2 * s1 + s2) / (h * h), k.accelerationPerOmegaSquared(theta), 2e-4);
        }
        double a = 0.043;
        assertEquals(-a * (1 + a / 0.139), k.accelerationPerOmegaSquared(0), 1e-12);
    }

    @Test void valveLiftIsSmoothAndItsFloatSpeedIsTheWeakestPointOfTheLobe() {
        ValveLift lift = new ValveLift(Math.toRadians(-250), Math.toRadians(240), 0.010, 3);
        assertEquals(0.010, lift.lift(Math.toRadians(-250)), 1e-15);
        assertEquals(0, lift.lift(Math.toRadians(-250 + 121)), 0);
        assertEquals(0, lift.lift(Math.toRadians(-250 + 720 + 121)), 0);
        double omega = 500, h = 1e-5;
        for (double deg = -365; deg < -135; deg += 7.3) {
            double th = Math.toRadians(deg);
            double fd = (lift.lift(th - h) - 2 * lift.lift(th) + lift.lift(th + h)) / (h * h) * omega * omega;
            assertEquals(fd, lift.acceleration(th, omega), 1e-3 * Math.abs(fd) + 2, "at " + deg);
        }
        // independent: scan crank angles for the first point where spring force falls below the needed force
        double preload = 250, rate = 40000, mass = 0.08, found = 0;
        for (double w = 50; w < 3000; w += 0.5) {
            boolean floats = false;
            for (double deg = -370; deg <= -130; deg += 0.05) {
                double th = Math.toRadians(deg), acc = lift.acceleration(th, w);
                if (acc < 0 && mass * -acc > preload + rate * lift.lift(th)) { floats = true; break; }
            }
            if (floats) { found = w; break; }
        }
        assertEquals(found, lift.floatSpeed(preload, rate, mass), 1.0);
        assertEquals(Math.toRadians(240) * Math.pow(1 - Math.cbrt(0.1), 1 / 3.0), lift.durationAbove(0.001), 1e-12);
    }

    @Test void orificeFlowChokesAndReducesToBernoulliForSmallDrops() {
        double r = 287, gamma = 1.4, t0 = 300, p0 = 100000, area = 1e-4;
        double critical = area * p0 / Math.sqrt(r * t0) * Math.sqrt(gamma) * Math.pow(2 / (gamma + 1), (gamma + 1) / (2 * (gamma - 1)));
        assertEquals(critical, ValveFlow.massFlow(area, p0, t0, 30000, r, gamma), 1e-12);
        assertEquals(critical, ValveFlow.massFlow(area, p0, t0, 52000, r, gamma), 1e-12);
        double dp = 200, rho = p0 / (r * t0);
        assertEquals(area * Math.sqrt(2 * rho * dp), ValveFlow.massFlow(area, p0, t0, p0 - dp, r, gamma), 0.002 * area * Math.sqrt(2 * rho * dp));
        assertEquals(0, ValveFlow.massFlow(area, p0, t0, p0 + 1, r, gamma), 0);
    }

    @Test void wiebeBurnsNinetyPercentAtItsBurnAngle() {
        assertEquals(0.9, Combustion.burned(37, 37, 3.1), 1e-12);
        assertEquals(0, Combustion.burned(-1, 37, 3.1), 0);
        // Bonatesta's correlation within its fitted range gives the 20-70 degree angles of his figures.
        double angle = Combustion.burnAngle90(4.5, 6.8, 0.12, 25);
        assertTrue(angle > 35 && angle < 60, "burn angle " + angle);
    }

    @Test void douaudEyzatDelayHasItsPublishedForm() {
        double tau = Combustion.ignitionDelay(95, 40 * 101325, 900);
        assertEquals(17.68e-3 * Math.pow(0.95, 3.402) * Math.pow(40, -1.7) * Math.exp(3800.0 / 900), tau, 1e-12);
        assertTrue(Combustion.ignitionDelay(100, 40 * 101325, 900) > tau);
    }

    @Test void oilViscosityScalingIsOneForTheReferenceOil() {
        assertEquals(1, EngineOil.W10_30.frictionScaling(90), 1e-12);
        assertTrue(EngineOil.W15_40.frictionScaling(20) > 2 * EngineOil.W15_40.frictionScaling(90));
    }

    /** A 3.0 l V6 like the one of Sandoval's figures 4-9 and 4-15 (bearing sizes are estimates). */
    static Friction.Engine v6() {
        return new Friction.Engine(0.089, 0.0795, 6, 10, 4, 0.0675, 0.022, 0.053, 0.020, 16, 24, 0.0085,
                Friction.Valvetrain.DOHC_FINGER_FOLLOWER, true, 1, 1);
    }

    @Test void frictionOfAV6IsInSandovalsRangeAndRoughlyDoublesWhenCold() {
        Friction.Engine e = v6();
        double warm = Friction.fmep(e, 2000, 101325, 101325, EngineOil.W15_40.frictionScaling(90)).total();
        double cold = Friction.fmep(e, 2000, 101325, 101325, EngineOil.W15_40.frictionScaling(20)).total();
        // Sandoval fig. 4-15: about 90 kPa warm and 200 kPa at 20 C (ratio about 2.1) at 2000 rpm, 15W-40.
        assertTrue(warm > 65 && warm < 115, "warm fmep " + warm);
        assertTrue(cold / warm > 1.7 && cold / warm < 2.6, "cold/warm " + cold / warm);
        double fast = Friction.fmep(e, 6000, 101325, 101325, 1).total();
        assertTrue(fast > warm, "friction rises with speed");
    }

    /** A 1.6 l four-valve cylinder (Bonatesta's engine, table 1) for the cycle checks. */
    static EngineCycle.Spec bonatesta(double runnerLength, double lambda) {
        double bore = 0.079, stroke = 0.0814, cr = 11, vd = Math.PI * bore * bore / 4 * stroke;
        CrankKinematics k = new CrankKinematics(bore, stroke, 3.37 * stroke / 2, vd / (cr - 1));
        ValveLift in = new ValveLift(Math.toRadians((-366 - 130) / 2.0), Math.toRadians(236), 0.0073, 3);
        ValveLift ex = new ValveLift(Math.toRadians((135 + 366) / 2.0), Math.toRadians(231), 0.00698, 3);
        return new EngineCycle.Spec(k, 2, 0.0276, 0.0243, 0.0055, 0.7, in, 2, 0.0214, 0.0188, 0.0055, 0.7, ex,
                runnerLength, 0.036, Fuel.gasoline(95, 85), lambda, 1, false, bore / 2, 420, 1.15, 1);
    }

    @Test void adiabaticMotoringIsReversible() {
        EngineCycle.Spec spec = bonatesta(0, 1);
        EngineCycle.Simulation sim = new EngineCycle.Simulation(spec, new EngineCycle.Conditions(3000, 101325, 300, 101325, 20));
        double v = spec.crank().volume(-Math.PI), m = 101325 * v / (Gas.AIR.gasConstant() * 300);
        double[] out = sim.closed(-Math.PI, Math.PI, m, 300, 0, 0, 10, 30, 3, true);
        assertEquals(300, out[1], 0.05);
        assertEquals(0, out[0], 1e-4 * 101325 * spec.crank().displacement());
        // TDC temperature against an independent isentrope: integral of cv dT / T = R ln(V1/V2)
        double[] half = sim.closed(-Math.PI, 0, m, 300, 0, 0, 10, 30, 3, true);
        double target = Gas.AIR.gasConstant() * Math.log(v / spec.crank().volume(0)), t = 300, s = 0;
        while (s < target) { s += Gas.AIR.cv(t + 0.0005) / (t + 0.0005) * 0.001; t += 0.001; }
        assertEquals(t, half[1], 0.2);
    }

    @Test void nearlyInstantAdiabaticCombustionGivesTheFuelAirOttoEfficiency() {
        EngineCycle.Spec spec = bonatesta(0, 1);
        EngineCycle.Simulation sim = new EngineCycle.Simulation(spec, new EngineCycle.Conditions(2000, 101325, 300, 101325, 0));
        CrankKinematics k = spec.crank();
        double v1 = k.volume(-Math.PI), v2 = k.volume(0), t1 = 300;
        double m = 101325 * v1 / (Gas.AIR.gasConstant() * t1);
        Fuel fuel = spec.fuel();
        double fuelKg = m / (fuel.stoichiometricAirFuel() + 1);
        double[] sim1 = sim.closed(-Math.PI, Math.PI, m, t1, 0, fuelKg, 0, 1.5, 2, true);
        // Independent fuel-air Otto cycle: isentropic compression (air), constant-volume energy release, isentropic expansion (products).
        Gas air = Gas.AIR, products = fuel.products(1);
        double t2 = isentrope(air, t1, Math.log(v1 / v2));
        double u3 = air.energy(t2) + fuelKg * fuel.heatRelease(1) / m;
        double t3 = products.temperature(u3);
        double t4 = isentrope(products, t3, -Math.log(v1 / v2));
        double work = m * ((products.energy(t3) - products.energy(t4)) - (air.energy(t2) - air.energy(t1)));
        assertEquals(work, sim1[0], 0.006 * work);
    }

    /** Temperature after an isentropic volume change: integral of cv dT / T = -R ln(V2/V1) = R logRatio. */
    static double isentrope(Gas gas, double t, double logVolumeRatio) {
        double target = gas.gasConstant() * logVolumeRatio, s = 0, dt = logVolumeRatio > 0 ? 0.001 : -0.001;
        while (logVolumeRatio > 0 ? s < target : s > target) { s += gas.cv(t + dt / 2) / (t + dt / 2) * dt; t += dt; }
        return t;
    }

    @Test void aWarmFourValveEngineHasTextbookEfficiencyAndBreathing() {
        EngineCycle.Spec spec = bonatesta(0.35, 0.9);
        EngineCycle.Result r = EngineCycle.run(spec, new EngineCycle.Conditions(3000, 101325, 300, 101325 + 0.178e3 * Math.pow(2 * 0.0814 * 50, 2), 18));
        double vd = spec.crank().displacement();
        // Heywood: net indicated efficiency of SI engines at full load about 0.35-0.42; VE of four-valve engines 0.85-1.0.
        assertTrue(r.indicatedEfficiency(vd) > 0.33 && r.indicatedEfficiency(vd) < 0.44, "eta " + r.indicatedEfficiency(vd));
        assertTrue(r.volumetricEfficiency() > 0.82 && r.volumetricEfficiency() < 1.05, "VE " + r.volumetricEfficiency());
        assertTrue(r.residualFraction() > 0.02 && r.residualFraction() < 0.1, "residual " + r.residualFraction());
        assertTrue(r.peakPressurePa() > 50e5 && r.peakPressurePa() < 100e5, "peak " + r.peakPressurePa());
    }
}

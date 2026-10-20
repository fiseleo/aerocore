package dev.mcmod.aerocore.api.engine;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Compression ignition and turbocharging (0.26): the correlations as published, and a diesel cylinder within the ranges real engines run. */
class DieselTest {
    /** A 2.0 l-class diesel cylinder: 85 x 88 mm, compression 17, two valves each side, a 160 bar peak pressure limit. */
    static EngineCycle.Spec diesel(double lambda) {
        double bore = 0.085, stroke = 0.088, cr = 17, vd = Math.PI * bore * bore / 4 * stroke;
        CrankKinematics k = new CrankKinematics(bore, stroke, 0.145, vd / (cr - 1));
        ValveLift in = new ValveLift(Math.toRadians((-366 - 130) / 2.0), Math.toRadians(236), 0.0085, 3);
        ValveLift ex = new ValveLift(Math.toRadians((135 + 366) / 2.0), Math.toRadians(231), 0.0085, 3);
        return new EngineCycle.Spec(k, 2, 0.028, 0.0245, 0.006, 0.7, in, 2, 0.025, 0.022, 0.006, 0.7, ex,
                0, 0, Fuel.DIESEL, lambda, 0.99, true, bore / 2, 450, 1.1, 1.2, new EngineCycle.CompressionIgnition(16e6));
    }

    static Dyno.Point run(double rpm, double phi, double boost) {
        return Dyno.operate(diesel(1 / phi), rpm, boost * 101325, 320, boost * 101325 + 5000, new Dyno.SparkControl(false, 30, 1));
    }

    @Test void ignitionDelayAndWatsonsBurnAsPublished() {
        // Hardenberg-Hase by hand: S_p 10 m/s, 900 K, 50 bar, CN 50 -> 2.56 exp(8251.2 (1/7482.9 - 1/17190) (21.2/37.6)^0.63) = 3.951 degrees
        assertEquals(3.951, Combustion.dieselIgnitionDelayDeg(10, 900, 50e5, 50), 0.002);
        assertTrue(Combustion.dieselIgnitionDelayDeg(10, 900, 50e5, 35) > Combustion.dieselIgnitionDelayDeg(10, 900, 50e5, 50), "lower cetane, longer delay");
        assertTrue(Combustion.dieselIgnitionDelayDeg(10, 700, 50e5, 50) > Combustion.dieselIgnitionDelayDeg(10, 900, 50e5, 50), "colder, longer");
        assertEquals(Double.POSITIVE_INFINITY, Combustion.dieselIgnitionDelayDeg(10, 900, 12e5, 50), "too little compression: no ignition");
        // Watson: 1 - 0.926 phi^0.37 / tau^0.26; more premixed with a longer delay and a leaner charge
        assertEquals(1 - 0.926 * Math.pow(0.6, 0.37), Combustion.premixedFraction(0.6, 1), 1e-12);
        assertTrue(Combustion.premixedFraction(0.6, 2) > Combustion.premixedFraction(0.6, 1));
        assertTrue(Combustion.premixedFraction(0.3, 1) > Combustion.premixedFraction(0.6, 1));
        Combustion.DieselBurn b = Combustion.DieselBurn.of(0.6, 1, 2000);
        assertEquals(0, b.burned(0), 0);
        assertTrue(b.burned(1) > 0.999);
        double sum = 0, prev = 0;
        for (int i = 1; i <= 4000; i++) {
            double tau = i / 4000.0, x = b.burned(tau);
            assertTrue(x >= prev - 1e-12, "monotone");
            sum += b.rate(tau - 0.5 / 4000) / 4000;
            prev = x;
        }
        assertEquals(b.burned(1), sum, 0.01);
    }

    @Test void aDieselCylinderRunsLikeOne() {
        Dyno.Point p = run(2000, 0.6, 1);
        EngineCycle.Result r = p.cycle();
        double vd = diesel(1).crank().displacement(), eta = r.indicatedEfficiency(vd);
        System.out.printf("diesel 2000 rpm phi 0.6: IMEP %.2f bar, eta_i %.3f, SOI %.1f, delay %.1f deg, premixed %.2f, peak %.0f bar at %.1f, exhaust %.0f K%n",
                r.netImep() / 1e5, eta, p.sparkAdvanceDeg(), r.ignitionDelayDeg(), r.premixedFraction(), r.peakPressurePa() / 1e5, r.peakPressureDeg(),
                r.exhaustTemperatureK());
        // naturally aspirated DI diesels: 7-10 bar IMEP at lambda 1.6-1.7, net indicated efficiency 0.40-0.48 (Heywood ch. 15)
        assertTrue(r.netImep() > 6e5 && r.netImep() < 11e5, "IMEP " + r.netImep());
        assertTrue(eta > 0.38 && eta < 0.52, "indicated efficiency " + eta);
        assertTrue(r.ignitionDelayDeg() > 1 && r.ignitionDelayDeg() < 15, "delay " + r.ignitionDelayDeg());
        assertTrue(r.peakPressurePa() > 5e6 && r.peakPressurePa() <= 16.05e6, "peak " + r.peakPressurePa());
        assertTrue(r.exhaustTemperatureK() > 600 && r.exhaustTemperatureK() < 1100, "exhaust " + r.exhaustTemperatureK());
        assertTrue(!r.knocks(), "a diesel does not knock");
        // less fuel, less work; twice the air at the same phi, about twice the work
        assertTrue(run(2000, 0.3, 1).cycle().netImep() < r.netImep());
        double boosted = run(2000, 0.6, 2).cycle().netImep();
        assertTrue(boosted / r.netImep() > 1.6 && boosted / r.netImep() < 2.4, "boost x2 gives " + boosted / r.netImep());
    }

    @Test void aTurbochargerMatchesItsCompressorWithItsTurbine() {
        Turbocharger t = new Turbocharger(0.05, 12, 0.75, 4e-4, 0.70, 0.95, 2.5, 0.6, 2e-5);
        double w = t.compressorPowerW(0.1, 2, 298);
        assertTrue(w > 8.5e3 && w < 10.5e3, "compressing 0.1 kg/s by 2: " + w);
        assertTrue(t.chargeK(2, 298, 310) < Turbomachinery.compressorExitTemperature(2, 0.75, 298), "the intercooler cools");
        // the turbine gives the compressor exactly its power, with the wastegate letting the rest past
        Turbocharger.Match m = t.match(w, 0.105, 900, 101325);
        assertTrue(m.reached() && m.turbineShare() <= 1);
        assertEquals(w, t.mechanicalEfficiency() * t.turbineFlowKgPerS(m.turbineInletPa(), 900, 101325)
                * t.turbineWorkJPerKg(900, m.turbineInletPa() / 101325), 1e-3 * w);
        // hotter exhaust, more boost; the impeller's speed limits the ratio
        double cool = t.maxBoost(r -> 0.05 * r, r -> 700, 0.04, 101325, 298), hot = t.maxBoost(r -> 0.05 * r, r -> 950, 0.04, 101325, 298);
        assertTrue(hot >= cool && hot > 1, "boost " + cool + " -> " + hot);
        assertTrue(t.pressureRatioAt(150000, 298) > t.pressureRatioAt(80000, 298));
    }

    @Test void aDieselMapIsMonotoneInLoad() {
        DieselMap.Engine e = new DieselMap.Engine() {
            @Override public EngineCycle.Spec spec(double lambda) { return diesel(lambda); }
            @Override public double intakeTemperature(double ambientK, double boostRatio) { return ambientK + 20; }
            @Override public double exhaustPressure(double ambientPa, double intakePa, double rpm, double airKgPerS, double exhaustK) { return ambientPa + 5000; }
            @Override public double fuelLimitKg(double rpm) { return Double.POSITIVE_INFINITY; }
        };
        DieselMap m = DieselMap.build(e, new DieselMap.Calibration(new double[]{1500, 3000}, new double[]{1}, new double[]{0.3, 0.6}, 30, 101325, 300, 4));
        assertTrue(m.value(2000, 1, 0.6, EngineMap.Point::netImepPa) > m.value(2000, 1, 0.3, EngineMap.Point::netImepPa));
        assertEquals(m.at(0, 0, 1).netImepPa(), m.value(1500, 1, 0.6, EngineMap.Point::netImepPa), 1e-6);
    }
}

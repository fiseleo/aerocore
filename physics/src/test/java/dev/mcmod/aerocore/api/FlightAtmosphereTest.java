package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class FlightAtmosphereTest {
    @Test void drydenLowAltitudeFollowsMilF8785C() {
        // 100 ft, W20 = 15 m/s: sigma_w = 1.5, sigma_u = 1.5 / 0.2593^0.4 = 2.574, L_u = 100 / 0.2593^1.2 ft = 505 ft, L_w = 100 ft.
        FlightAtmosphere.Dryden d = FlightAtmosphere.lowAltitude(100 * 0.3048, 15);
        assertEquals(1.5, d.sigmaW(), 1e-12);
        assertEquals(2.574, d.sigmaU(), 1e-3);
        assertEquals(505.4 * 0.3048, d.lengthU(), 0.2);
        assertEquals(30.48, d.lengthW(), 1e-9);
        // Each spectrum integrates to its variance (one-sided over 0..infinity).
        double su = 0, sw = 0, step = 1e-4;
        for (double omega = step / 2; omega < 30; omega += step) { su += d.spectrumU(omega) * step; sw += d.spectrumW(omega) * step; }
        assertEquals(d.sigmaU() * d.sigmaU(), su, 0.01 * d.sigmaU() * d.sigmaU());
        assertEquals(d.sigmaW() * d.sigmaW(), sw, 0.01 * d.sigmaW() * d.sigmaW());
        // Turbulence grows more isotropic with height: sigma_u / sigma_w falls toward 1 by 1000 ft.
        FlightAtmosphere.Dryden high = FlightAtmosphere.lowAltitude(1000 * 0.3048, 15);
        assertEquals(1, high.sigmaU() / high.sigmaW(), 1e-3);
    }

    @Test void drydenGustsHaveTheModelsDeviationsAndCorrelation() {
        FlightAtmosphere.Dryden d = FlightAtmosphere.lowAltitude(50, 12);
        FlightAtmosphere.Gusts gusts = new FlightAtmosphere.Gusts(9);
        double dt = 0.02, speed = 40;
        int n = 400000, lag = (int) Math.round(d.lengthU() / speed / dt);
        double[] u = new double[n];
        double su = 0, sw = 0;
        for (int i = 0; i < n; i++) {
            double[] g = gusts.step(d, speed, dt);
            u[i] = g[0]; su += g[0] * g[0]; sw += g[2] * g[2];
        }
        assertEquals(d.sigmaU(), Math.sqrt(su / n), 0.05 * d.sigmaU(), "sigma_u");
        assertEquals(d.sigmaW(), Math.sqrt(sw / n), 0.05 * d.sigmaW(), "sigma_w");
        // The u gust's autocorrelation at one scale length of travel is 1/e (exponential, Dryden).
        double c = 0;
        for (int i = 0; i + lag < n; i++) { c += u[i] * u[i + lag]; }
        assertEquals(Math.exp(-1), c / (n - lag) / (su / n), 0.05);
    }

    @Test void groundEffectFollowsWieselsberger() {
        // h/b = 0.1: sigma = 0.868 / 1.79 = 0.485, about half the induced drag gone; none beyond h/b = 0.76.
        assertEquals(0.868 / 1.79, FlightAtmosphere.groundInterference(1, 10), 1e-12);
        assertEquals(1 - 0.868 / 1.79, FlightAtmosphere.inducedDragFactor(1, 10), 1e-12);
        assertEquals(1, FlightAtmosphere.inducedDragFactor(8, 10), 1e-12);
        assertEquals(8 / (1 - 0.868 / 1.79), FlightAtmosphere.effectiveAspectRatio(8, 1, 10), 1e-9);
        for (double h = 0; h < 8; h += 0.5) { assertTrue(FlightAtmosphere.inducedDragFactor(h, 10) <= FlightAtmosphere.inducedDragFactor(h + 0.5, 10)); }
    }

    @Test void thermalsScaleWithTheMixedLayerAndCarryNoNetFlux() {
        // A sunny afternoon: 250 W/m^2 sensible heat into a 1200 m mixed layer at 300 K gives w* of about 2 m/s.
        double wStar = FlightAtmosphere.convectiveVelocity(250, 1200, 300);
        assertEquals(2.01, wStar, 0.02);
        assertEquals(0, FlightAtmosphere.convectiveVelocity(-10, 1200, 300), 0);
        // Allen at z/z_i = 0.1: w_c = w* 0.1^(1/3) 0.89, r2 = 0.102 0.1^(1/3) 0.975 z_i.
        assertEquals(wStar * Math.cbrt(0.1) * 0.89, FlightAtmosphere.thermalMeanUpdraft(wStar, 120, 1200), 1e-12);
        assertEquals(0.102 * Math.cbrt(0.1) * 0.975 * 1200, FlightAtmosphere.thermalRadius(120, 1200), 1e-9);
        assertEquals(0, FlightAtmosphere.thermalMeanUpdraft(wStar, 1150, 1200), 0, "No updraft near the inversion");
        // The core rises at 3 w_c, stops at r2, sinks around it, and carries no net flux through a level.
        double r2 = FlightAtmosphere.thermalRadius(120, 1200), flux = 0, dr = 0.01;
        assertEquals(3 * FlightAtmosphere.thermalMeanUpdraft(wStar, 120, 1200), FlightAtmosphere.thermalUpdraft(wStar, 0, 120, 1200), 1e-12);
        assertEquals(0, FlightAtmosphere.thermalUpdraft(wStar, r2, 120, 1200), 1e-12);
        assertTrue(FlightAtmosphere.thermalUpdraft(wStar, 1.5 * r2, 120, 1200) < 0, "Sink ring");
        for (double r = dr / 2; r < 10 * r2; r += dr) { flux += FlightAtmosphere.thermalUpdraft(wStar, r, 120, 1200) * 2 * Math.PI * r * dr; }
        assertEquals(0, flux, 1e-3 * FlightAtmosphere.thermalUpdraft(wStar, 0, 120, 1200) * Math.PI * r2 * r2);
        assertTrue(FlightAtmosphere.thermalSpacing(120, 1200) > 4 * r2, "Thermals stand well apart");
    }

    @Test void ridgeLiftRisesUpWindwardSlopesAndFadesAloft() {
        assertEquals(10 * 0.3, FlightAtmosphere.ridgeLift(10, 0.3, 0, 100), 1e-12);
        assertEquals(3 * Math.exp(-1), FlightAtmosphere.ridgeLift(10, 0.3, 100, 100), 1e-12);
        assertTrue(FlightAtmosphere.ridgeLift(10, -0.3, 0, 100) < 0, "Sink on the lee slope");
    }
}

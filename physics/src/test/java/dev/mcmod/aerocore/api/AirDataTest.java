package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AirDataTest {
    private static Atmosphere.Sample isa(double h) { return Atmosphere.atAltitude(h, Vec3d.ZERO); }

    @Test void pressureAndDensityAltitudeInvertTheStandardAtmosphere() {
        for (double h : new double[]{-500, 0, 300, 5000, 10_999, 11_000, 15_000, 19_999, 20_000, 25_000, 32_000}) {
            assertEquals(h, AirData.pressureAltitudeM(isa(h).pressurePa()), 1e-6, "pressure altitude at " + h);
            assertEquals(h, AirData.densityAltitudeM(isa(h).fluid().densityKgPerM3()), 1e-3, "density altitude at " + h);
        }
    }

    @Test void qnhAndQfeAndTheAltimeter() {
        double qfe = AirData.stationPa(AirData.P0, 300);
        assertEquals(isa(300).pressurePa(), qfe, 1e-9, "in the standard atmosphere QFE is its pressure there");
        assertEquals(AirData.P0, AirData.qnhPa(qfe, 300), 1e-9, "and QNH 1013.25 hPa");
        assertEquals(98_000, AirData.qnhPa(AirData.stationPa(98_000, 1234), 1234), 1e-8);
        assertEquals(300, AirData.altimeterM(qfe, AirData.P0), 1e-6, "set to QNH an altimeter reads the elevation");
        assertEquals(0, AirData.altimeterM(qfe, qfe), 1e-6, "set to QFE it reads zero");
        // the rule of thumb: 27 ft (8.2 m) per hPa near sea level
        double low = AirData.altimeterM(qfe, 100_000);
        assertEquals(300 - 8.3 * 13.25, low, 3, "a setting 13.25 hPa low reads about 110 m low");
    }

    @Test void airspeeds() {
        Atmosphere.Sample sea = isa(0), high = isa(10_000);
        assertEquals(340.294, AirData.A0, 1e-3);
        assertEquals(120, AirData.casMps(120, sea.pressurePa(), sea.temperatureKelvin()), 1e-9, "at sea level CAS is TAS");
        assertEquals(120, AirData.easMps(120, sea.fluid().densityKgPerM3()), 1e-9);
        // 250 m/s true at 10 km: Mach 0.8348, impact pressure 15.30 kPa, CAS 154.1 m/s (hand computed from the pitot formulas)
        double cas = AirData.casMps(250, high.pressurePa(), high.temperatureKelvin());
        assertEquals(0.8348, AirData.mach(250, high.temperatureKelvin()), 1e-4);
        assertEquals(154.1, cas, 0.5);
        assertEquals(250, AirData.tasMps(cas, high.pressurePa(), high.temperatureKelvin()), 1e-6);
        // Rayleigh's pitot: total over static behind the shock at Mach 2 is 5.640 (NACA Report 1135)
        assertEquals(5.6405, AirData.impactPa(2, 1) + 1, 1e-3);
        assertEquals(2, AirData.machFromImpact(AirData.impactPa(2, 30_000), 30_000), 1e-9);
        assertEquals(1, AirData.machFromImpact(AirData.impactPa(1, 50_000), 50_000), 1e-9, "continuous at Mach 1");
        double fast = AirData.casMps(600, high.pressurePa(), high.temperatureKelvin());
        assertEquals(600, AirData.tasMps(fast, high.pressurePa(), high.temperatureKelvin()), 1e-6, "supersonic round trip");
    }

    @Test void windComponentsAndBearings() {
        // course north (toward -Z): a wind from the north (blowing toward +Z) is all headwind
        AirData.Components head = AirData.components(new Vec3d(0, 0, 5), 0);
        assertEquals(5, head.headwindMps(), 1e-12);
        assertEquals(0, head.crosswindMps(), 1e-12);
        // a wind from the east (toward -X) heading north comes from the right
        AirData.Components cross = AirData.components(new Vec3d(-5, 0, 0), 0);
        assertEquals(0, cross.headwindMps(), 1e-12);
        assertEquals(5, cross.crosswindMps(), 1e-12);
        // runway 27 (west) in that east wind: a tailwind
        assertEquals(-5, AirData.components(new Vec3d(-5, 0, 0), Math.toRadians(270)).headwindMps(), 1e-12);
        assertEquals(0, AirData.compassDeg(0, -1), 1e-12);
        assertEquals(90, AirData.compassDeg(1, 0), 1e-12);
        assertEquals(180, AirData.compassDeg(0, 1), 1e-12);
        assertEquals(270, AirData.compassDeg(-1, 0), 1e-12);
        assertEquals(270, AirData.bearingFromDeg(new Vec3d(3, 0, 0)), 1e-12, "blowing east: from the west");
        assertEquals(0, AirData.bearingFromDeg(new Vec3d(0, 0, 2)), 1e-12, "blowing south: from the north");
    }
}

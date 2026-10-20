package dev.mcmod.aerocore.api.light;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AllardTest {
    @Test void theVisibilityIsWhereContrastFallsToFivePercent() {
        for (double v : new double[]{200, 1500, 20_000}) {
            assertEquals(Allard.MOR_CONTRAST, Math.exp(-Allard.extinctionPerM(v) * v), 1e-12);
        }
        assertEquals(2.996 / 1000, Allard.extinctionPerM(1000), 1e-6);
    }

    @Test void inClearAirItIsTheInverseSquare() {
        double e = Allard.illuminanceLux(10_000, 1000, 1e12);
        assertEquals(0.01, e, 1e-9);
    }

    @Test void theThresholdRisesWithTheBackground() {
        assertEquals(6.8e-6, Allard.thresholdLux(Allard.NIGHT_BACKGROUND), 0.1e-6, "ICAO's night floor");
        assertEquals(Allard.thresholdLux(Allard.NIGHT_BACKGROUND), Allard.thresholdLux(0.01), 1e-18, "not lower in darker nights");
        double day = Allard.thresholdLux(Allard.DAY_BACKGROUND);
        assertEquals(7.2e-4, day, 0.1e-4);
        assertTrue(day > 100 * Allard.thresholdLux(Allard.NIGHT_BACKGROUND));
    }

    @Test void runwayLightsAreSeenAsFarAsRunwayVisualRangeTablesGive() {
        // A 10 000 cd high-intensity edge light at night in 800 m MOR is seen about 1.4-1.6 km away (RVR exceeds MOR
        // with bright lights at night: ICAO Doc 9328 rule of thumb RVR ~ 1.5-2 MOR); by day only near the MOR.
        double night = Allard.rangeM(10_000, 800, Allard.thresholdLux(Allard.NIGHT_BACKGROUND));
        double day = Allard.rangeM(10_000, 800, Allard.thresholdLux(Allard.DAY_BACKGROUND));
        System.out.printf("10000 cd in 800 m MOR: seen %.0f m at night, %.0f m by day%n", night, day);
        assertTrue(night > 1.4 * 800 && night < 2.2 * 800);
        assertTrue(day > 0.6 * 800 && day < 1.3 * 800);
        assertTrue(Allard.rangeM(100, 800, Allard.thresholdLux(Allard.NIGHT_BACKGROUND)) < night, "weaker lights are seen less far");
        assertEquals(0, Allard.rangeM(1e-9, 800, 1), 0);
        double edge = Allard.rangeM(10_000, 800, Allard.thresholdLux(Allard.NIGHT_BACKGROUND));
        assertEquals(Allard.thresholdLux(Allard.NIGHT_BACKGROUND), Allard.illuminanceLux(10_000, edge, 800), 1e-9);
    }
}

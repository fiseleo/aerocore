package dev.mcmod.aerocore.api.light;

import dev.mcmod.aerocore.api.Vec3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SectorColoursTest {
    private static final int RED = 0xffff2010, WHITE = 0xffffffff;

    /** A PAPI's four units (ICAO Annex 14 5.3.5: settings 2 deg 30', 2 deg 50', 3 deg 10', 3 deg 30' for a 3 degree path). */
    private static SectorColours[] papi() {
        double[] minutes = {150, 170, 190, 210};
        SectorColours[] units = new SectorColours[4];
        for (int i = 0; i < 4; i++) { units[i] = SectorColours.papi(Math.toRadians(minutes[i] / 60), Math.toRadians(3.0 / 60), RED, WHITE); }
        return units;
    }

    private static int whites(SectorColours[] units, double elevationDeg) {
        int n = 0;
        for (SectorColours u : units) { if (u.colour(0, Math.toRadians(elevationDeg)) == WHITE) { n++; } }
        return n;
    }

    @Test void onThePathAPapiShowsTwoRedTwoWhite() {
        SectorColours[] units = papi();
        assertEquals(2, whites(units, 3.0), "on the 3 degree path");
        assertEquals(4, whites(units, 3.6), "high: all white");
        assertEquals(3, whites(units, 3.3), "slightly high");
        assertEquals(1, whites(units, 2.7), "slightly low");
        assertEquals(0, whites(units, 2.3), "low: all red");
        for (SectorColours u : units) { assertEquals(RED, u.colour(0, Math.toRadians(-5)), "below the horizon red"); }
    }

    @Test void theTransitionIsNarrowAndContinuous() {
        SectorColours u = SectorColours.papi(Math.toRadians(3), Math.toRadians(3.0 / 60), RED, WHITE);
        assertEquals(RED, u.colour(0, Math.toRadians(3 - 2.0 / 60)), "2 minutes below: red");
        assertEquals(WHITE, u.colour(0, Math.toRadians(3 + 2.0 / 60)), "2 minutes above: white");
        int mid = u.colour(0, Math.toRadians(3));
        assertNotEquals(RED, mid);
        assertNotEquals(WHITE, mid);
        int previousGreen = -1;
        for (int i = -40; i <= 40; i++) {
            int g = (u.colour(0, Math.toRadians(3 + i / 600.0)) >> 8) & 255;
            assertTrue(g >= previousGreen, "green rises steadily red to white");
            previousGreen = g;
        }
    }

    @Test void anAzimuthSectorAndALampsBeam() {
        // green to the left of the axis, red to the right
        SectorColours side = new SectorColours(SectorColours.Plane.AZIMUTH, new double[]{0}, new int[]{RED, 0xff00ff00}, 0);
        FieldLamp lamp = FieldLamp.beam(Vec3d.ZERO, Vec3d.X, 180, 20, 5, 1000, WHITE, 0, 0.1).with(side);
        // axis east; north (-z) is to the left looking east
        assertEquals(0xff00ff00, lamp.colour(new Vec3d(1, 0, -1), 0));
        assertEquals(RED, lamp.colour(new Vec3d(1, 0, 1), 0));
        assertEquals(1, lamp.coverage(new Vec3d(1, Math.tan(Math.toRadians(5)), 0)), 1e-12, "on the beam");
        assertEquals(1, lamp.coverage(new Vec3d(0.01, 0.0, 1)), 1e-12, "89 degrees off: within its half-plane");
        assertEquals(0, lamp.coverage(new Vec3d(-1, 0.1, 0)), 1e-12, "behind it");
        assertEquals(0.5, lamp.coverage(new Vec3d(1, Math.tan(Math.toRadians(17.5)), 0)), 1e-9, "half way through the feather above the beam");
        assertEquals(1, FieldLamp.omni(Vec3d.ZERO, 100, WHITE, 0, 0.1).coverage(new Vec3d(-3, -0.5, 2)), 1e-12);
    }
}

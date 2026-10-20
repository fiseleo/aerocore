package dev.mcmod.aerocore.api.aero;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** 0.30: the display approximation of a rotor's wake and outwash stays within momentum theory's bounds. */
class DownwashTest {
    private static final double R = 8.18, VI = 11;   // a UH-60-class rotor in hover

    @Test void theWakeSpeedsUpTowardTwiceTheInducedVelocity() {
        assertEquals(VI, Downwash.wakeSpeed(VI, R, 0), 1e-12, "at the disk");
        double previous = 0;
        for (double z = 0; z <= 20 * R; z += 0.5) {
            double v = Downwash.wakeSpeed(VI, R, z);
            assertTrue(v >= previous - 1e-12 && v <= 2 * VI + 1e-12, "between v_i and 2 v_i, rising: " + v);
            previous = v;
        }
        assertEquals(2 * VI, Downwash.wakeSpeed(VI, R, 1e6), 1e-3, "far down");
    }

    @Test void theOutwashPeaksNearTheRotorsRadiusAndDecaysOutward() {
        double h = R;
        double atR = Downwash.outwash(VI, R, h, R);
        assertTrue(atR > 0 && atR < 2 * VI, "below twice v_i: " + atR);
        double previous = atR;
        for (double r = 1.1 * R; r < 6 * R; r += 0.5) {
            double u = Downwash.outwash(VI, R, h, r);
            assertTrue(u < previous, "decays outward");
            previous = u;
        }
        assertTrue(Downwash.outwash(VI, R, h, 0.5 * R) < atR, "rises from the axis to the radius");
        // about r^-1: doubling the radius halves it roughly
        double ratio = Downwash.outwash(VI, R, h, 4 * R) / Downwash.outwash(VI, R, h, 2 * R);
        assertTrue(ratio > 0.4 && ratio < 0.55, "about r^-1: " + ratio);
    }

    @Test void aRotorHighUpStirsLittle() {
        double low = Downwash.outwash(VI, R, R, 1.2 * R), high = Downwash.outwash(VI, R, 8 * R, 1.2 * R);
        assertTrue(high < 0.1 * low, "four diameters up: " + high + " of " + low);
        assertEquals(0, Downwash.outwash(VI, 0, R, R));
        assertEquals(0, Downwash.outwash(VI, R, -1, R));
    }
}

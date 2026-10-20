package dev.mcmod.aerocore.api.light;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class LightSignalTest {
    @Test void eachFlasherFiresOncePerPeriodInTurn() {
        // 21 flashers twice a second, sampled frame by frame at 37 fps (frames longer than a flasher's slot) and 240 fps
        for (double fps : new double[]{37, 60, 240}) {
            int count = 21;
            double period = 0.5, dt = 1 / fps;
            int[] fired = new int[count];
            double[] firstAt = new double[count];
            java.util.Arrays.fill(firstAt, -1);
            double last = -1e-6;
            for (double t = dt; t <= 10 + 1e-9; t += dt) {
                for (int i = 0; i < count; i++) {
                    LightSignal s = LightSignal.sequence(period, i, count);
                    // a flash counts once even when two frames both overlap it: only the frame its start falls in
                    if (s.peak(last, t) > 0 && s.level(last) == 0) { fired[i]++; if (firstAt[i] < 0) { firstAt[i] = t; } }
                }
                last = t;
            }
            for (int i = 0; i < count; i++) {
                assertEquals(20, fired[i], 1, "flasher " + i + " at " + fps + " fps fires twice a second for 10 s");
            }
            for (int i = 1; i < count; i++) {
                assertTrue(firstAt[i] >= firstAt[i - 1] - 1e-9, "flasher " + i + " fires after the one before it");
            }
        }
    }

    @Test void aFlashShowsInTheFrameItFallsIn() {
        LightSignal s = LightSignal.flash(1.2, 0.02, 0.3);
        assertEquals(0, s.level(0.29));
        assertEquals(1, s.level(0.31));
        assertEquals(0, s.level(0.33));
        assertEquals(1, s.peak(0.29, 0.40), "a frame that spans the flash shows it");
        assertEquals(0, s.peak(0.33, 1.49));
        assertEquals(1, s.peak(1.49, 1.51), "and the next one, a period later");
        assertEquals(1, s.peak(-5, -3.6), "a span a period long or more always holds a flash");
        assertEquals(1, LightSignal.steady().peak(3, 3));
    }

    @Test void aBeaconAlternatesItsColours() {
        LightSignal b = LightSignal.alternate(1.0, 0.2, 0xffffffff, 0xff00ff00);
        assertEquals(0xffffffff, b.colour(0.1, 0xff123456));
        assertEquals(0xff00ff00, b.colour(1.1, 0xff123456));
        assertEquals(0xffffffff, b.colour(2.1, 0xff123456));
        assertEquals(0xff00ff00, b.colour(-0.5, 0xff123456), "before the start too");
        assertEquals(0xff123456, LightSignal.steady().colour(5, 0xff123456));
        assertEquals(1, b.level(2.1));
        assertEquals(0, b.level(2.5));
    }

    @Test void badSignalsAreRefused() {
        assertThrows(IllegalArgumentException.class, () -> LightSignal.flash(1, 2, 0));
        assertThrows(IllegalArgumentException.class, () -> LightSignal.sequence(0.5, 3, 3));
        assertThrows(IllegalArgumentException.class, () -> LightSignal.alternate(1, 0.5));
    }
}

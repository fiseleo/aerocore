package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class BowSprayTest {
    private static final double G = Atmosphere.GRAVITY_MPS2;

    @Test void displacementBowFollowsNoblesseRegimesAndBernoulliBound() {
        // Regime boundary 4.4 tan a / cos a = 1 + F: a = 12.5 deg at F = 0 and 24.9 deg at F = 1.25 (review values).
        double draft = 0.5, speedAt125 = 1.25 * Math.sqrt(G * draft);
        assertFalse(BowSpray.displacementBow(speedAt125, Math.toRadians(24.0), draft, G).breaking());
        assertTrue(BowSpray.displacementBow(speedAt125, Math.toRadians(25.8), draft, G).breaking());
        assertTrue(BowSpray.displacementBow(1e-3, Math.toRadians(12.8), draft, G).breaking());
        assertFalse(BowSpray.displacementBow(1e-3, Math.toRadians(12.2), draft, G).breaking());
        // Crest height of a slender bow and the U^2 / (2g) cap of a blunt one.
        BowSpray slender = BowSpray.displacementBow(3, Math.toRadians(10), draft, G);
        double f = 3 / Math.sqrt(G * draft);
        assertEquals(9 / G * 2.2 * Math.tan(Math.toRadians(10)) / ((1 + f) * Math.cos(Math.toRadians(10))), slender.crestHeightM(), 1e-12);
        BowSpray blunt = BowSpray.displacementBow(4, Math.PI / 2, draft, G);
        assertEquals(16 / (2 * G), blunt.crestHeightM(), 1e-12);
        assertTrue(blunt.breaking());
        // The droplets thrown up reach the crest.
        assertEquals(blunt.crestHeightM(), blunt.verticalMps() * blunt.verticalMps() / (2 * G), 1e-12);
        assertEquals(4, blunt.forwardMps(), 1e-12);
    }

    @Test void planingSprayRootMatchesSavitsky() {
        // Check value: deadrise 20 deg, trim 4 deg -> spray root 16.79 deg to the keel.
        assertEquals(16.79, Math.toDegrees(BowSpray.sprayRootAngle(Math.toRadians(4), Math.toRadians(20))), 0.01);
        BowSpray spray = BowSpray.planing(10, Math.toRadians(4), Math.toRadians(20), G);
        double root = Math.toRadians(16.79), be = Math.atan((1 - 2 / Math.PI) * Math.tan(Math.toRadians(20)));
        double vv = 10 * Math.sin(root) + 10 * Math.cos(root) * Math.sin(root) * Math.sin(be);
        assertEquals(vv, spray.verticalMps(), 0.01);
        assertEquals(vv * vv / (2 * G), spray.crestHeightM(), 0.01);
        assertEquals(BowSpray.NONE, BowSpray.planing(10, 0, Math.toRadians(20), G));
    }
}

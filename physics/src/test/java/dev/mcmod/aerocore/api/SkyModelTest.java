package dev.mcmod.aerocore.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SkyModelTest {
    @Test void clearSkyIsBlueAtTheZenithBrighterAndWhiterTowardTheHorizonAndTheSun() {
        SkyModel.Clear sky = SkyModel.clear(2.5, Math.toRadians(30));
        double[] zenith = sky.xyY(0, Math.toRadians(30));
        assertEquals(sky.zenithLuminance(), zenith[0], 1e-12);
        assertEquals(sky.zenithX(), zenith[1], 1e-12);
        // Zenith luminance of a clear sky with the sun 60 degrees high: several kcd/m^2 (Preetham's fit gives 8.2).
        assertEquals(8.18, sky.zenithLuminance(), 0.05);
        assertTrue(zenith[1] < 0.27 && zenith[2] < 0.29, "Blue zenith: x " + zenith[1] + " y " + zenith[2]);
        double[] horizonAway = sky.xyY(Math.toRadians(85), Math.toRadians(150)), above = sky.xyY(Math.toRadians(45), Math.toRadians(150));
        assertTrue(horizonAway[0] > above[0], "Horizon brightening");
        assertTrue(horizonAway[1] > zenith[1] && horizonAway[2] > zenith[2], "Whiter toward the horizon");
        double[] nearSun = sky.xyY(Math.toRadians(35), Math.toRadians(5)), sideways = sky.xyY(Math.toRadians(35), Math.toRadians(90));
        assertTrue(nearSun[0] > 3 * sideways[0], "Circumsolar brightness: " + nearSun[0] + " vs " + sideways[0]);
        // A low sun reddens the sky near it.
        SkyModel.Clear low = SkyModel.clear(2.5, Math.toRadians(88));
        assertTrue(low.xyY(Math.toRadians(85), Math.toRadians(3))[1] > zenith[1] + 0.03, "Warm sky at a low sun");
        // The hemisphere's mean lies between the darkest sky and the circumsolar glare.
        double mean = sky.meanLuminance();
        assertTrue(mean > sky.xyY(Math.toRadians(45), Math.toRadians(150))[0] && mean < nearSun[0], "Mean luminance " + mean);
        // Hazier air whitens the zenith.
        assertTrue(SkyModel.clear(6, Math.toRadians(30)).zenithX() > sky.zenithX());
    }

    @Test void overcastSkyIsThreeTimesBrighterAtTheZenithThanTheHorizon() {
        assertEquals(1, SkyModel.overcast(0), 1e-12);
        assertEquals(1.0 / 3, SkyModel.overcast(Math.PI / 2), 1e-12);
    }

    @Test void colourConversionKeepsWhiteWhite() {
        double[] rgb = SkyModel.linearRgb(SkyModel.D65_X, SkyModel.D65_Y, 1);
        for (double c : rgb) { assertEquals(1, c, 2e-3); }
        assertEquals(1, SkyModel.encode(1), 1e-12);
        assertEquals(0.7354, SkyModel.encode(0.5), 1e-4);
        assertEquals(12.92 * 0.001, SkyModel.encode(0.001), 1e-12);
    }
}
